#!/usr/bin/env python3
"""Semantic-free local 3D path search and actuator-aware command selection.

One fresh lidar index supplies route-grid occupancy, swept path checks and
the final command envelope. No car boxes, offset joins or frozen shift goals.
"""
import time
from concurrent.futures import ThreadPoolExecutor
import multiprocessing
import numpy as np
from path_sampling import swept_samples
from predictive_avoidance import departure_floor_limits
from lidar_scene import LidarScene


def _plan_snapshot(snapshot):
    """Pure geometry worker: no ROS state or live controller closure."""
    from execution_guard import ExecutionGuard
    from velocity_response import VelocityResponse
    position,s,stations,coordinates,heights,floor_offset,index,margin,horizon,budget,coupled=snapshot
    guard=ExecutionGuard(margin=margin);guard.index=index
    if coupled:guard.response_model=VelocityResponse(native=False)
    navigator=LidarNavigator(guard,horizon=horizon,budget=budget)
    xy=lambda t:tuple(np.interp(t,stations,coordinates[:,axis]) for axis in (0,1))
    center=lambda t:float(np.interp(t,stations,heights))
    return navigator.path(position,s,xy,center,floor_offset,index)


def _geometry_worker(connection):
    try:
        while True:
            snapshot=connection.recv()
            if snapshot is None:break
            try:connection.send((True,_plan_snapshot(snapshot)))
            except Exception as error:connection.send((False,str(error)))
    finally:connection.close()


class GeometryProcess:
    """One geometry process and a receiver thread; no named queue semaphores."""
    def __init__(self,timeout=4.):
        self.timeout=timeout
        self._start()
        self.receiver=ThreadPoolExecutor(max_workers=1)

    def _start(self):
        context=multiprocessing.get_context('spawn')
        self.connection,worker=context.Pipe()
        self.process=context.Process(target=_geometry_worker,args=(worker,),daemon=True)
        self.process.start();worker.close()

    def _solve(self,snapshot):
        if not self.process.is_alive():
            self.connection.close();self._start()
        self.connection.send(snapshot)
        deadline=time.monotonic()+self.timeout
        while not self.connection.poll(.1):
            if not self.process.is_alive():raise RuntimeError('geometry process exited')
            if time.monotonic()>deadline:
                # Discard this entire channel before any following job. A
                # late result must never be mistaken for a newer snapshot.
                self.process.terminate();self.process.join(timeout=.2)
                if self.process.is_alive():self.process.kill();self.process.join(timeout=.2)
                raise RuntimeError('geometry process deadline exceeded')
        success,result=self.connection.recv()
        if not success:raise RuntimeError(result)
        return result

    def submit(self,function,snapshot):
        return self.receiver.submit(self._solve,snapshot)

    def shutdown(self,wait=True):
        self.receiver.shutdown(wait=True)
        if self.process.is_alive():
            self.connection.send(None);self.process.join(timeout=1.)
        if self.process.is_alive():self.process.terminate();self.process.join(timeout=1.)
        self.connection.close()


class LidarNavigator:
    def __init__(self, guard, horizon=24., budget=.06,async_planning=False,process_planning=False):
        self.guard=guard
        self.horizon=horizon
        self.budget=budget
        self.last_path=None
        self.last_plan_stamp=None
        self.last_cloud_stamp=None
        self.process_planning=process_planning
        self.executor=(GeometryProcess()
                       if async_planning and process_planning else
                       ThreadPoolExecutor(max_workers=1) if async_planning else None)
        self.future=None;self.submitted_at=None;self.future_stamp=None
        self.scene=LidarScene()
        self.reference_coordinates=None
        self.road_lateral_limit=2.25

    def close(self):
        if self.executor is not None:self.executor.shutdown(wait=self.process_planning)

    def reset(self):
        self.last_path=None
        self.last_plan_stamp=None
        self.last_cloud_stamp=None
        self.scene.reset()

    def _bound_ok(self,error,timed=False):
        """An existing violation may shrink, never grow or certify hovering."""
        if np.max(error)<=1e-8:return True
        initial=error[0]
        if initial<=0. or np.max(error)>initial+1e-8:return False
        tails=np.array([error[-1]])
        if timed and self.guard.envelope_times is not None:
            times=self.guard.envelope_times
            tails=error[times>=times.max()-.15]
        return np.max(tails)<=max(0.,initial-.01)+1e-8

    def _floor_ok(self, path, stations, center, floor_offset,timed=False):
        if self.guard.response_model is not None:
            heights=np.interp(stations,self.reference_stations,self.reference_heights)
            if not self._bound_ok(path[:,2]-heights-1.25,timed):return False
            if not self._bound_ok(heights-path[:,2]-1.25,timed):return False
            if self.reference_coordinates is not None:
                road=np.column_stack([np.interp(stations,self.reference_stations,self.reference_coordinates[:,axis]) for axis in (0,1)])
                if np.any(np.linalg.norm(path[:,:2]-road,axis=1)>self.road_lateral_limit+1e-8):return False
        if floor_offset is None:return True
        limits=departure_floor_limits(stations,path,center,floor_offset,
                                      self.guard.index,self.guard.margin+.1)
        return self._bound_ok(path[:,2]-limits,timed) if self.guard.response_model is not None else np.all(path[:,2]<=limits+1e-8)

    @staticmethod
    def _stop_profile(path):
        """Follow the bypass height while slowing, including road grade."""
        path=np.asarray(path);segments=np.diff(path[:,:2],axis=0)
        length=np.sum(segments**2,axis=1);vertical=np.diff(path[:,2])
        arc=np.r_[0.,np.cumsum(np.sqrt(length))]
        def target(positions,velocities):
            dx=positions[:,None,0]-path[None,:-1,0]
            dy=positions[:,None,1]-path[None,:-1,1]
            fraction=np.clip((dx*segments[None,:,0]+dy*segments[None,:,1])/np.maximum(1e-8,length),0.,1.)
            dx-=fraction*segments[None,:,0];dy-=fraction*segments[None,:,1]
            distance=dx*dx+dy*dy
            distance[:,length<1e-8]=np.inf
            nearest=np.argmin(distance,axis=1)
            if np.all(length<1e-8):return np.clip(path[-1,2]-positions[:,2],-4.,4.5)
            reference=path[nearest,2]+fraction[np.arange(len(nearest)),nearest]*vertical[nearest]
            along=arc[nearest]+fraction[np.arange(len(nearest)),nearest]*np.sqrt(length[nearest])
            lo=np.maximum(0.,along-2.);hi=np.minimum(arc[-1],along+2.)
            direction=np.column_stack([np.interp(hi,arc,path[:,axis])-np.interp(lo,arc,path[:,axis]) for axis in (0,1)])
            direction/=np.maximum(1e-8,np.linalg.norm(direction,axis=1))[:,None]
            slope=(np.interp(hi,arc,path[:,2])-np.interp(lo,arc,path[:,2]))/np.maximum(1e-8,hi-lo)
            ff=np.sum(velocities[:,:2]*direction,axis=1)*slope
            return np.clip(ff+reference-positions[:,2],-4.,4.5)
        target.path=path
        return target

    @staticmethod
    def _advance_grid(previous,neighbors,shift):
        trial=np.r_[previous.ravel(),np.inf][neighbors]+shift
        best=np.argmin(trial,axis=2)[:,:,None]
        return (np.take_along_axis(trial,best,axis=2)[:,:,0],
                np.take_along_axis(neighbors,best,axis=2)[:,:,0])

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
        queries=np.asarray(queries);result=np.empty(len(queries))
        # Keep the exact comparison against every segment, with bounded
        # working arrays. The whole grid previously allocated a 3-D delta
        # tensor and repeated axis reductions on each planning cycle.
        for start in range(0,len(queries),2048):
            chunk=queries[start:start+2048]
            dx=chunk[:,None,0]-road[None,:-1,0]
            dy=chunk[:,None,1]-road[None,:-1,1]
            fractions=np.clip((dx*segment[None,:,0]+dy*segment[None,:,1])/
                              np.maximum(1e-8,length),0.,1.)
            dx-=fractions*segment[None,:,0]
            dy-=fractions*segment[None,:,1]
            distances=dx*dx+dy*dy
            if np.any(length>1e-8):distances[:,length<=1e-8]=np.inf
            nearest=np.argmin(distances,axis=1)
            result[start:start+len(chunk)]=stations[nearest]+.5*fractions[np.arange(len(nearest)),nearest]
        return result

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
                            [] if floor_offset is None else [floor_offset-.02,floor_offset-.05]])
        yy,zz=np.meshgrid(ys,zs,indexing='ij');shape=yy.shape
        world=nominal[:,None,None,:]+yy[None,:,:,None]*sides[:,None,None,:]
        world[:,:,:,2]+=zz
        iy=int(np.argmin(abs(ys-current_y)));iz=int(np.argmin(abs(zs-current_z)))
        world[0,iy,iz]=position
        node_stations=(self._route_stations(world.reshape(-1,3),s,xy,projection)
                       if floor_offset is not None or self.guard.response_model is not None
                       else np.repeat(stations,yy.size))
        # Only distances inside 1.75 m affect occupancy or clearance cost.
        # Exact bounded queries avoid searching far-away surfaces at every cell.
        distances=np.minimum(1.75,index.distance(world.reshape(-1,3),limit=1.75)).reshape(world.shape[:-1])
        valid=distances>=buffer
        if self.guard.response_model is not None:
            road_z=np.interp(node_stations,stations,nominal[:,2]).reshape(valid.shape)
            valid&=abs(world[:,:,:,2]-road_z)<=1.25
            valid&=abs(yy[None,:,:])<=2.25
            # A small measured tracking overshoot may re-enter the center
            # band; no later node may create a new out-of-band detour.
            valid[0,iy,iz]=(distances[0,iy,iz]>=buffer and abs(current_z)<=1.25)
        floors=departure_floor_limits(node_stations,world.reshape(-1,3),
                                      center,floor_offset,index,buffer).reshape(valid.shape)
        valid&=world[:,:,:,2]<=floors-(.02 if self.guard.response_model is not None else 0.)
        if self.guard.response_model is not None:
            # Recover from measured/updated-bound disagreement. Every later
            # node still satisfies the full bounds and geometry buffer.
            valid[0,iy,iz]=distances[0,iy,iz]>=buffer
        penalty=.015*yy**2+.03*zz**2+4.*np.maximum(0.,1.55-distances)**2
        ny,nz=shape
        transitions=[]
        for dy in range(-5,6):
            for dz in range(-4,5):
                dst_y=slice(max(0,dy),min(ny,ny+dy));src_y=slice(max(0,-dy),min(ny,ny-dy))
                dst_z=slice(max(0,dz),min(nz,nz+dz));src_z=slice(max(0,-dz),min(nz,nz-dz))
                dy_value=ys[dst_y]-ys[src_y];dz_value=zs[dst_z]-zs[src_z]
                physical=(abs(dy_value[:,None])<=1.+1e-8)&(abs(dz_value[None,:])<=.25+1e-8)
                if not np.any(physical):continue
                py,pz=np.meshgrid(np.arange(ny)[src_y],np.arange(nz)[src_z],indexing='ij')
                shift=.18*dy_value[:,None]**2+.4*dz_value[None,:]**2
                shift[~physical]=np.inf
                transitions.append((dst_y,dst_z,src_y,src_z,shift,py,pz))
        neighbors=np.full((ny,nz,len(transitions)),ny*nz,dtype=np.int16)
        shifts=np.full(neighbors.shape,np.inf)
        for choice,(dst_y,dst_z,_,_,shift,py,pz) in enumerate(transitions):
            neighbors[dst_y,dst_z,choice]=py*nz+pz
            shifts[dst_y,dst_z,choice]=shift
        attempts=0;rejections=[];search_started=time.monotonic()
        # Occupancy construction may exceed the retry budget under simulator
        # load. Always attempt one complete, certified path before giving up.
        while attempts<12 and (attempts==0 or time.monotonic()-search_started<self.budget):
            attempts+=1
            costs=np.full((len(stations),ny,nz),np.inf)
            parent=np.full((len(stations),ny,nz),-1,dtype=np.int16)
            if not valid[0,iy,iz]:break
            costs[0,iy,iz]=0.
            last=0
            for i in range(1,len(stations)):
                costs[i],parent[i]=self._advance_grid(costs[i-1],neighbors,shifts)
                costs[i]+=penalty[i]
                costs[i,~valid[i]]=np.inf
                if not np.any(np.isfinite(costs[i])):break
                last=i
            if last<2:break
            j,k=np.unravel_index(np.argmin(costs[last]),shape)
            nodes=[]
            for i in range(last,-1,-1):
                nodes.append((i,j,k))
                if i:j,k=divmod(int(parent[i,j,k]),nz)
            nodes.reverse();path=np.array([world[node] for node in nodes])
            queries,segments,fractions=swept_samples(path)
            clearance=np.minimum(1.75,index.distance(queries,limit=1.75))
            along=None
            failures=np.flatnonzero(clearance<buffer)
            if floor_offset is not None or self.guard.response_model is not None:
                along=self._route_stations(queries,s,xy,projection)
                floor=departure_floor_limits(along,queries,center,floor_offset,index,buffer)
                floor_error=queries[:,2]-floor
                failures=np.flatnonzero((clearance<buffer)|(floor_error>max(0.,floor_error[0])+1e-8))
                if not self._bound_ok(floor_error):failures=np.r_[failures,len(queries)-1]
                if self.guard.response_model is not None:
                    road_height=np.interp(along,stations,nominal[:,2])
                    road_xy=np.column_stack([np.interp(along,projection[0],projection[1][:,axis]) for axis in (0,1)])
                    upper=queries[:,2]-road_height-1.25;lower=road_height-queries[:,2]-1.25
                    failures=np.flatnonzero((clearance<buffer)|(floor_error>max(0.,floor_error[0])+1e-8)|
                        (upper>max(0.,upper[0])+1e-8)|(lower>max(0.,lower[0])+1e-8)|
                        (np.linalg.norm(queries[:,:2]-road_xy,axis=1)>max(2.25,float(np.linalg.norm(position[:2]-nominal[0,:2])))+1e-8))
                    if not all(self._bound_ok(e) for e in (floor_error,upper,lower)):failures=np.r_[failures,len(queries)-1]
            if not len(failures):
                return path,dict(path_distance=float(last),path_clearance=float(np.min(clearance)),
                                 grid_attempts=attempts,grid_ms=1000.*(time.monotonic()-started))
            bad=min(last,int(segments[failures[0]])+1)
            rejections.append(dict(station=float(stations[min(last,int(segments[failures[0]]))]) if along is None else float(along[failures[0]]),
                clearance=float(clearance[failures[0]]),position=queries[failures[0]].tolist()))
            valid[nodes[bad]]=False
        return None,dict(path_distance=0.,grid_attempts=attempts,
                         grid_ms=1000.*(time.monotonic()-started),grid_rejections=rejections,
                         search_timeout=time.monotonic()-search_started>=self.budget)

    def _usable_path(self,path,position,s,xy,center,floor_offset,projection):
        """Keep a certified near prefix while a changed far obstacle replans.

        The final command still checks its entire timed stopping envelope
        against the latest cloud. A short prefix never grants extra speed.
        """
        samples,segments,_=swept_samples(path)
        stations=self._route_stations(samples,s,xy,projection)
        bad=self.guard.index.distance(samples,limit=1.75)<self.guard.margin+.1
        if self.guard.response_model is not None:
            height=np.interp(stations,self.reference_stations,self.reference_heights)
            for error in (samples[:,2]-height-1.25,height-samples[:,2]-1.25):
                bad|=error>max(0.,error[0])+1e-8
            road=np.column_stack([np.interp(stations,self.reference_stations,self.reference_coordinates[:,axis]) for axis in (0,1)])
            bad|=np.linalg.norm(samples[:,:2]-road,axis=1)>self.road_lateral_limit+1e-8
        if floor_offset is not None:
            floor=departure_floor_limits(stations,samples,center,floor_offset,self.guard.index,self.guard.margin+.1)
            error=samples[:,2]-floor
            bad|=error>max(0.,error[0])+1e-8 if self.guard.response_model is not None else error>1e-8
        truncated=False
        if np.any(bad):
            first=int(np.flatnonzero(bad)[0])
            if first==0:return None,{}
            path=np.concatenate((path[:int(segments[first])+1],samples[first-1:first]))
            if np.sum(np.linalg.norm(np.diff(path,axis=0),axis=1))<2.:return None,{}
            samples,_,_=swept_samples(path)
            stations=self._route_stations(samples,s,xy,projection)
            truncated=True
            if np.min(self.guard.index.distance(samples,limit=1.75))<self.guard.margin+.1:return None,{}
        if not self._floor_ok(samples,stations,center,floor_offset):return None,{}
        return path,dict(truncated=truncated)

    def _commands(self, position, velocity, desired, path, age, s, xy, center, floor_offset,projection=None):
        """Score executed XYZ responses and full stopping paths, not rays."""
        buffer=self.guard.margin+.1
        if projection is None:projection=self._projection(s,xy)
        distances=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
        lookaheads=np.unique(np.minimum(distances[-1],np.array([1.,2.,5.,10.,16.] if self.guard.response_model is not None else [.5,1.,2.,4.,6.])))
        speed=max(.5,float(np.linalg.norm(desired[:2])))
        commands=[desired.copy()*factor for factor in ((1.,.5,0.) if self.guard.response_model is not None else (1.,.9375,.875,.75,.5,0.))]
        for ahead in lookaheads:
            goal=np.array([np.interp(ahead,distances,path[:,axis]) for axis in range(3)])
            delta=goal-position;horizontal=max(.05,float(np.linalg.norm(delta[:2])))
            for factor in ((1.,.5) if self.guard.response_model is not None else (1.,.75,.5,.25)):
                forward=min(speed*factor,max(.5,horizontal*1.5))
                cmd=np.r_[delta[:2]*forward/horizontal,np.clip(delta[2]*forward/horizontal,-4.,4.5)]
                commands.append(cmd)
        if self.guard.response_model is not None:
            model=self.guard.response_model
            model.stop_profile=self._stop_profile(path)
            prepared=[]
            for target in commands:
                command=model.prepare(target,velocity,self.guard.pose_stamp)
                # Use measured motion and the XY command actually sent for
                # grade feed-forward, including inertia during counter-braking.
                responding=command.copy()
                responding[:2]=.6*velocity[:2]+.4*command[:2]
                vertical=model.stop_profile(np.array([position]),np.array([responding]))[0]
                command[2]=np.clip(vertical+model.lift_gain*np.sum((command[:2]-velocity[:2])**2),-4.,model.vertical_command_max)
                prepared.append(command)
            commands=prepared
        commands=np.unique(np.round(commands,6),axis=0)
        best=None;best_score=-np.inf;best_clear=None
        first=np.asarray(xy(s+.3)[:2])-np.asarray(xy(s-.3)[:2]);first/=max(1e-6,np.linalg.norm(first))
        envelopes=None
        if self.guard.response_model is not None:
            self.guard.response_model.stop_profile=self._stop_profile(path)
            envelopes=self.guard.response_model.envelopes(position,velocity,commands,self.guard.reaction+age)
        batch_clearances=None
        if envelopes is not None:
            lengths=[len(e[0]) for e in envelopes]
            self.guard.envelope_times=np.concatenate([e[2] for e in envelopes])
            all_samples=np.concatenate([e[0] for e in envelopes])
            all_distances=np.minimum(2.,self.guard.index.distance(all_samples,limit=2.))
            if self.guard.dynamic_scene is not None:
                all_distances=np.minimum(all_distances,self.guard.dynamic_scene.distance(all_samples,self.guard.envelope_times))
            batch_clearances=[float(np.min(d)) for d in np.split(all_distances,np.cumsum(lengths)[:-1])]
        for i,command in enumerate(commands):
            if envelopes is None:
                samples,extent=self.guard.command_envelope(position,velocity,command,age)
            else:
                samples,extent,self.guard.envelope_times=envelopes[i]
            if extent>self.guard.horizon:continue
            clearance=(self.guard.command_clearance(samples,limit=2.) if batch_clearances is None else batch_clearances[i])
            if clearance<buffer:continue
            if floor_offset is not None or self.guard.response_model is not None:
                sample_s=self._route_stations(samples,s,xy,projection)
                if not self._floor_ok(samples,sample_s,center,floor_offset,timed=self.guard.response_model is not None):continue
            # Predict the moving response and distance to the whole reachable
            # path. Opposing car layouts can use different lanes in one plan.
            tau=max(self.guard.settling,np.linalg.norm(velocity)/(2*self.guard.braking),
                    np.linalg.norm(command)/(2*self.guard.braking))
            t=.8
            predicted=position+velocity*(self.guard.reaction+age)+command*t+tau*(1.-np.exp(-t/tau))*(velocity-command)
            if self.guard.response_model is not None:
                predicted=self.guard.response_model.forecast(position,velocity,command,self.guard.reaction+age)
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
        if self.guard.response_model is not None and points is not None:
            forward=np.asarray(xy(s+.3)[:2])-np.asarray(xy(s-.3)[:2])
            forward/=max(1e-6,np.linalg.norm(forward))
            side=np.array([-forward[1],forward[0]])
            relative=np.asarray(points)-position
            along=relative[:,:2]@forward
            lateral=np.asarray(points)[:,:2]-np.asarray(xy(s)[:2])
            across=lateral@side
            sampled=s+np.arange(-25.,25.1,.5)
            heights=np.array([center(t) for t in sampled])
            delta_z=points[:,2]-np.interp(s+along,sampled,heights)
            self.guard.surface_seed_points=np.asarray(points)[(abs(along)<20.)&(abs(across)<3.8)&(abs(delta_z)<1.8)]
        age,error=self.guard._update(points,cloud_stamp,pose_stamp,position)
        info=dict(source='LIDAR_NAV',active=True,cap=None,plan_id=0,cars=0)
        if error:
            return np.zeros(3),dict(info,feasible=False,command_reason=error,command_scale=0.)
        if self.guard.response_model is not None:
            forward=np.asarray(xy(s+.3)[:2])-np.asarray(xy(s-.3)[:2])
            forward/=max(1e-6,np.linalg.norm(forward))
            self.scene.update(points,position,cloud_stamp,pose_stamp,forward,center,s)
            self.guard.dynamic_scene=self.scene
            info.update(self.scene.summary(),response_model='SLEW_COUPLED',
                        response_backend=('native' if self.guard.response_model.native is not None and
                                          self.guard.response_model.native.library is not None else 'python'),
                        lift_gain=self.guard.response_model.lift_gain,
                        vertical_command_max=self.guard.response_model.vertical_command_max,
                        brake_feedback=self.guard.response_model.brake_feedback,
                        deceleration=self.guard.response_model.deceleration)
        path=None;details={};projection=self._projection(s,xy)
        self.reference_stations=projection[0]
        self.reference_coordinates=projection[1]
        self.reference_heights=np.array([center(t) for t in self.reference_stations])
        self.road_lateral_limit=max(2.25,float(np.linalg.norm(np.asarray(position)[:2]-np.asarray(xy(s)[:2]))))
        if self.guard.response_model is not None:
            info['road_recovery']=self.road_lateral_limit>2.25+1e-8
            current=np.asarray([position])
            current_stations=self._route_stations(current,s,xy,projection)
            height=np.interp(current_stations,self.reference_stations,self.reference_heights)
            floor=departure_floor_limits(current_stations,current,center,floor_offset,self.guard.index,self.guard.margin+.1)
            info['bound_recovery']=bool(abs(current[0,2]-height[0])>1.25 or current[0,2]>floor[0]+1e-8)
        self.guard.command_constraint=(None if floor_offset is None and self.guard.response_model is None else lambda samples:self._floor_ok(samples,
            self._route_stations(samples,s,xy,projection),center,floor_offset,timed=self.guard.response_model is not None))
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
                if self.process_planning:
                    snapshot=(np.asarray(position).copy(),s,ss,coordinates,heights,floor_offset,
                        self.guard.index,self.guard.margin,self.horizon,self.budget,self.guard.response_model is not None)
                    self.future=self.executor.submit(_plan_snapshot,snapshot)
                else:
                    self.future=self.executor.submit(self.path,np.asarray(position).copy(),s,frozen_xy,
                        frozen_center,floor_offset,self.guard.index)
        if (self.last_path is not None and cloud_stamp==self.last_cloud_stamp and
                pose_stamp-self.last_plan_stamp<.2) or (self.executor is not None and self.last_path is not None):
            nearest=int(np.argmin(np.sum((self.last_path-position)**2,axis=1)))
            path=np.concatenate(([position],self.last_path[min(nearest+1,len(self.last_path)-1):]))
            forward=np.asarray(xy(s+.3)[:2])-np.asarray(xy(s-.3)[:2])
            forward/=max(1e-6,np.linalg.norm(forward))
            path,prefix_info=self._usable_path(path,position,s,xy,center,floor_offset,projection)
            if path is not None:details.update(prefix_info,path_distance=float((path[-1]-position)[:2]@forward),cached=True)
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
        if self.guard.response_model is not None:
            stopping_path=(self.last_path.copy() if self.last_path is not None else
                np.array([[*xy(t)[:2],center(t)] for t in self.reference_stations]))
            stopping_stations=self._route_stations(stopping_path,s,xy,projection)
            heights=np.interp(stopping_stations,self.reference_stations,self.reference_heights)
            stopping_path[:,2]=np.clip(stopping_path[:,2],heights-1.25,heights+1.25)
            floors=departure_floor_limits(stopping_stations,stopping_path,center,floor_offset,self.guard.index,self.guard.margin+.1)
            stopping_path[:,2]=np.minimum(stopping_path[:,2],floors-.02)
            self.guard.response_model.stop_profile=self._stop_profile(stopping_path)
            vertical=float(self.guard.response_model.stop_profile(np.array([position]),np.array([velocity]))[0])
        else:vertical=0.
        command,checked=self.guard.filter_command(position,velocity,np.array([0.,0.,vertical]),points,cloud_stamp,pose_stamp)
        return command,dict(info,**checked,feasible=False,navigation_ms=1000.*(time.monotonic()-started))
