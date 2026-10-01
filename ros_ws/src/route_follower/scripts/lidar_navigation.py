#!/usr/bin/env python3
"""Semantic-free local 3D path search and actuator-aware command selection.

One fresh lidar index supplies route-grid occupancy, swept path checks and
the final command envelope. No car boxes, offset joins or frozen shift goals.
"""
import time
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from path_sampling import swept_samples
from predictive_avoidance import departure_floor_limits


class LidarNavigator:
    def __init__(self, guard, horizon=24., budget=.06,async_planning=False):
        self.guard=guard
        self.horizon=horizon
        self.budget=budget
        self.last_path=None
        self.last_plan_stamp=None
        self.last_cloud_stamp=None
        self.executor=ThreadPoolExecutor(max_workers=1) if async_planning else None
        self.future=None;self.submitted_at=None;self.future_stamp=None

    def close(self):
        if self.executor is not None:self.executor.shutdown(wait=False)

    def reset(self):
        self.last_path=None
        self.last_plan_stamp=None
        self.last_cloud_stamp=None

    def _floor_ok(self, path, stations, center, floor_offset):
        limits=departure_floor_limits(stations,path,center,floor_offset,
                                      self.guard.index,self.guard.margin+.1)
        return np.all(path[:,2]<=limits+1e-8)

    def _projection(self, s, xy):
        stations=s+np.arange(-3.,self.horizon+3.1,.5)
        road=np.array([xy(t)[:2] for t in stations])
        segment=np.diff(road,axis=0);length=np.sum(segment*segment,axis=1)
        return stations,road,segment,length

    def _route_stations(self, queries, s, xy, projection=None):
        """Project geometry onto the route, including measured lateral offset.

        Progress from the aircraft along one tangent is not a road station:
        an offset start or curve shifts the floor constraint along the road.
        """
        stations,road,segment,length=self._projection(s,xy) if projection is None else projection
        delta=np.asarray(queries)[:,None,:2]-road[None,:-1,:]
        fractions=np.clip(np.sum(delta*segment,axis=2)/np.maximum(1e-8,length),0.,1.)
        distances=np.sum((delta-fractions[:,:,None]*segment)**2,axis=2)
        if np.any(length>1e-8):distances[:,length<=1e-8]=np.inf
        nearest=np.argmin(distances,axis=1)
        return stations[nearest]+.5*fractions[np.arange(len(nearest)),nearest]

    def path(self, position, s, xy, center, floor_offset,index=None):
        """Minimum-cost forward grid with measured start and swept edges."""
        started=time.monotonic();index=self.guard.index if index is None else index;buffer=self.guard.margin+.1
        projection=self._projection(s,xy)
        stations=s+np.arange(0.,self.horizon+1.,1.)
        nominal=np.array([[*xy(t)[:2],center(t)] for t in stations])
        forwards=np.array([np.asarray(xy(t+.3)[:2])-np.asarray(xy(t-.3)[:2]) for t in stations])
        forwards/=np.maximum(1e-6,np.linalg.norm(forwards,axis=1))[:,None]
        sides=np.column_stack((-forwards[:,1],forwards[:,0],np.zeros(len(stations))))
        current_y=float((position-nominal[0])@sides[0]);current_z=position[2]-nominal[0,2]
        ys=np.unique(np.r_[np.arange(-3.,3.01,.25),current_y])
        zs=np.unique(np.r_[np.arange(-2.,2.01,.25),current_z,
                            [] if floor_offset is None else [floor_offset-.02]])
        yy,zz=np.meshgrid(ys,zs,indexing='ij');shape=yy.shape
        world=nominal[:,None,None,:]+yy[None,:,:,None]*sides[:,None,None,:]
        world[:,:,:,2]+=zz
        iy=int(np.argmin(abs(ys-current_y)));iz=int(np.argmin(abs(zs-current_z)))
        world[0,iy,iz]=position
        # Only distances inside 1.75 m affect occupancy or clearance cost.
        # Exact bounded queries avoid searching far-away surfaces at every cell.
        distances=np.minimum(1.75,index.distance(world.reshape(-1,3),limit=1.75)).reshape(world.shape[:-1])
        valid=distances>=buffer
        floors=departure_floor_limits(np.repeat(stations,yy.size),world.reshape(-1,3),
                                      center,floor_offset,index,buffer).reshape(valid.shape)
        valid&=world[:,:,:,2]<=floors
        penalty=.015*yy**2+.03*zz**2+4.*np.maximum(0.,1.55-distances)**2
        ny,nz=shape
        transitions=[]
        for dy in range(-4,5):
            for dz in range(-1,2):
                dst_y=slice(max(0,dy),min(ny,ny+dy));src_y=slice(max(0,-dy),min(ny,ny-dy))
                dst_z=slice(max(0,dz),min(nz,nz+dz));src_z=slice(max(0,-dz),min(nz,nz-dz))
                dy_value=ys[dst_y]-ys[src_y];dz_value=zs[dst_z]-zs[src_z]
                py,pz=np.meshgrid(np.arange(ny)[src_y],np.arange(nz)[src_z],indexing='ij')
                transitions.append((dst_y,dst_z,src_y,src_z,.18*dy_value[:,None]**2+.4*dz_value[None,:]**2,py,pz))
        attempts=0;rejections=[]
        # Occupancy construction may exceed the retry budget under simulator
        # load. Always attempt one complete, certified path before giving up.
        while attempts<12 and (attempts==0 or time.monotonic()-started<self.budget):
            attempts+=1
            costs=np.full((len(stations),ny,nz),np.inf)
            parent=np.full((len(stations),ny,nz,2),-1,dtype=np.int16)
            if not valid[0,iy,iz]:break
            costs[0,iy,iz]=0.
            last=0
            for i in range(1,len(stations)):
                for dst_y,dst_z,src_y,src_z,shift_cost,py,pz in transitions:
                    trial=costs[i-1,src_y,src_z]+shift_cost
                    old=costs[i,dst_y,dst_z]
                    better=trial<old
                    old[better]=trial[better]
                    parents=parent[i,dst_y,dst_z];parents[better]=np.column_stack((py[better],pz[better]))
                costs[i]+=penalty[i]
                costs[i,~valid[i]]=np.inf
                if not np.any(np.isfinite(costs[i])):break
                last=i
            if last<2:break
            j,k=np.unravel_index(np.argmin(costs[last]),shape)
            nodes=[]
            for i in range(last,-1,-1):
                nodes.append((i,j,k))
                if i:j,k=map(int,parent[i,j,k])
            nodes.reverse();path=np.array([world[node] for node in nodes])
            queries,segments,fractions=swept_samples(path)
            clearance=np.minimum(1.75,index.distance(queries,limit=1.75))
            along=self._route_stations(queries,s,xy,projection)
            floor=departure_floor_limits(along,queries,center,floor_offset,index,buffer)
            failures=np.flatnonzero((clearance<buffer)|(queries[:,2]>floor))
            if not len(failures):
                return path,dict(path_distance=float(last),path_clearance=float(np.min(clearance)),
                                 grid_attempts=attempts,grid_ms=1000.*(time.monotonic()-started))
            bad=min(last,int(segments[failures[0]])+1)
            rejections.append(dict(station=float(along[failures[0]]),
                clearance=float(clearance[failures[0]]),position=queries[failures[0]].tolist()))
            valid[nodes[bad]]=False
        return None,dict(path_distance=0.,grid_attempts=attempts,
                         grid_ms=1000.*(time.monotonic()-started),grid_rejections=rejections,
                         search_timeout=time.monotonic()-started>=self.budget)

    def _commands(self, position, velocity, desired, path, age, s, xy, center, floor_offset,projection=None):
        """Score executed XYZ responses and full stopping paths, not rays."""
        buffer=self.guard.margin+.1
        if projection is None:projection=self._projection(s,xy)
        distances=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
        lookaheads=np.unique(np.minimum(distances[-1],np.array([.5,1.,2.,4.,6.])))
        speed=max(.5,float(np.linalg.norm(desired[:2])))
        commands=[desired.copy()*factor for factor in (1.,.9375,.875,.75,.5,0.)]
        for ahead in lookaheads:
            goal=np.array([np.interp(ahead,distances,path[:,axis]) for axis in range(3)])
            delta=goal-position;horizontal=max(.05,float(np.linalg.norm(delta[:2])))
            for factor in (1.,.75,.5,.25):
                forward=min(speed*factor,max(.5,horizontal*1.5))
                cmd=np.r_[delta[:2]*forward/horizontal,np.clip(delta[2]*forward/horizontal,-4.,4.5)]
                commands.append(cmd)
        commands=np.unique(np.round(commands,6),axis=0)
        best=None;best_score=-np.inf;best_clear=None
        first=np.asarray(xy(s+.3)[:2])-np.asarray(xy(s-.3)[:2]);first/=max(1e-6,np.linalg.norm(first))
        for command in commands:
            samples,extent=self.guard.command_envelope(position,velocity,command,age)
            if extent>self.guard.horizon:continue
            clearance=float(np.min(np.minimum(2.,self.guard.index.distance(samples,limit=2.))))
            if clearance<buffer:continue
            sample_s=self._route_stations(samples,s,xy,projection)
            if not self._floor_ok(samples,sample_s,center,floor_offset):continue
            # Predict the moving response and distance to the whole reachable
            # path. Opposing car layouts can use different lanes in one plan.
            tau=max(self.guard.settling,np.linalg.norm(velocity)/(2*self.guard.braking),
                    np.linalg.norm(command)/(2*self.guard.braking))
            t=.8
            predicted=position+velocity*(self.guard.reaction+age)+command*t+tau*(1.-np.exp(-t/tau))*(velocity-command)
            segments=path[1:]-path[:-1]
            fraction=np.clip(np.sum((predicted-path[:-1])*segments,axis=1)/
                             np.maximum(1e-8,np.sum(segments*segments,axis=1)),0.,1.)
            nearest=path[:-1]+fraction[:,None]*segments
            path_error=float(np.min(np.linalg.norm(nearest-predicted,axis=1)))
            progress=float((predicted-position)[:2]@first)
            aim_distance=min(distances[-1],max(2.,speed*(self.guard.reaction+age+.8)+2.))
            aim=np.array([np.interp(aim_distance,distances,path[:,axis]) for axis in range(3)])
            before=np.linalg.norm(aim-position);after=np.linalg.norm(aim-predicted)
            score=2.*(before-after)+.3*progress-2.*path_error+.12*min(2.,clearance)-.015*np.sum((command-desired)**2)
            if score>best_score:best_score=score;best=command;best_clear=clearance
        return best,best_clear,len(commands)

    def select(self, position, velocity, desired, s, xy, center, points, cloud_stamp,
               pose_stamp, floor_offset=None):
        started=time.monotonic()
        self.guard.set_faces(())
        age,error=self.guard._update(points,cloud_stamp,pose_stamp,position)
        info=dict(source='LIDAR_NAV',active=True,cap=None,plan_id=0,cars=0)
        if error:
            return np.zeros(3),dict(info,feasible=False,command_reason=error,command_scale=0.)
        path=None;details={};projection=self._projection(s,xy)
        if self.executor is not None:
            if self.future is not None and self.future.done():
                completed=self.future;self.future=None
                try:
                    ready,ready_info=completed.result()
                except Exception as error:
                    ready=None;ready_info=dict(planner_error=str(error))
                details.update(ready_info)
                if ready is not None:
                    self.last_path=ready;self.last_plan_stamp=self.future_stamp
            if self.future is None and (self.submitted_at is None or time.monotonic()-self.submitted_at>=.15):
                self.submitted_at=time.monotonic()
                self.future_stamp=pose_stamp
                ss=np.arange(s-1.,s+self.horizon+1.1,.5)
                coordinates=np.array([xy(t)[:2] for t in ss]);heights=np.array([center(t) for t in ss])
                frozen_xy=lambda t,stations=ss,coords=coordinates:tuple(np.interp(t,stations,coords[:,axis]) for axis in (0,1))
                frozen_center=lambda t,stations=ss,z=heights:float(np.interp(t,stations,z))
                self.future=self.executor.submit(self.path,np.asarray(position).copy(),s,frozen_xy,
                    frozen_center,floor_offset,self.guard.index)
        if (self.last_path is not None and cloud_stamp==self.last_cloud_stamp and
                pose_stamp-self.last_plan_stamp<.2) or (self.executor is not None and self.last_path is not None):
            nearest=int(np.argmin(np.sum((self.last_path-position)**2,axis=1)))
            path=np.concatenate(([position],self.last_path[min(nearest+1,len(self.last_path)-1):]))
            samples,_,_=swept_samples(path)
            forward=np.asarray(xy(s+.3)[:2])-np.asarray(xy(s-.3)[:2])
            forward/=max(1e-6,np.linalg.norm(forward))
            ss=self._route_stations(samples,s,xy,projection)
            if (np.min(self.guard.index.distance(samples,limit=1.75))<self.guard.margin+.1 or
                    not self._floor_ok(samples,ss,center,floor_offset)):
                path=None
            else:details.update(path_distance=float((path[-1]-position)[:2]@forward),cached=True)
        if path is None and self.executor is None:
            path,details=self.path(np.asarray(position),s,xy,center,floor_offset)
            self.last_plan_stamp=pose_stamp;self.last_cloud_stamp=cloud_stamp
        info.update(details)
        info['planner_busy']=self.future is not None
        command=None
        if path is not None:
            self.last_path=path
            command,clearance,count=self._commands(position,velocity,np.asarray(desired),path,age,s,xy,center,floor_offset,projection)
            info.update(candidate_count=count,path=path.tolist(),planned_stamp=self.last_plan_stamp,
                        geometry_cloud_stamp=cloud_stamp,plan_age=pose_stamp-self.last_plan_stamp)
            if command is not None:
                # Same envelope, same point index, same floor constraint.
                reason='LIDAR_TRACK' if np.linalg.norm(command[:2])>.1 else 'LIDAR_BRAKE'
                return command,dict(info,feasible=True,command_reason=reason,command_scale=None,
                    command_clearance=clearance,navigation_ms=1000.*(time.monotonic()-started))
        # If already inside a buffer, certify a monotonic escape directly.
        # No persistent target, retreat-pending latch or asynchronous join.
        if np.linalg.norm(velocity)<.8 and self.guard.index.distance([position])[0]<self.guard.margin+.1:
            a,b=np.asarray(xy(s-.3)[:2]),np.asarray(xy(s+.3)[:2])
            forward=(b-a)/max(1e-6,np.linalg.norm(b-a));side=np.r_[-forward[1],forward[0],0.]
            candidates=[position+np.array([0.,0.,z]) for z in (-.25,.25,-.5,.5,-1.,1.)]
            candidates+=[position+y*side+np.array([0.,0.,z]) for y in (-.5,.5,-1.,1.,-2.,2.) for z in (0.,-.5,.5)]
            for target in candidates:
                stations=np.array([s,s])
                if not self._floor_ok(np.array([position,target]),stations,center,floor_offset):continue
                escape=self.guard.escape(position,velocity,target,points,cloud_stamp,pose_stamp)
                if escape is not None:
                    return escape,dict(info,feasible=True,command_reason='LIDAR_ESCAPE',command_scale=None,
                        recovery_target=target.tolist(),navigation_ms=1000.*(time.monotonic()-started))
        command,checked=self.guard.filter_command(position,velocity,np.zeros(3),points,cloud_stamp,pose_stamp)
        return command,dict(info,**checked,feasible=False,navigation_ms=1000.*(time.monotonic()-started))
