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
    position,s,stations,coordinates,heights,floor_offset,index,margin,horizon,budget,coupled=snapshot[:11]
    anticipation_distance=snapshot[11] if len(snapshot)>11 else 0.
    path_options=snapshot[12] if len(snapshot)>12 else False
    guard=ExecutionGuard(margin=margin);guard.index=index
    if coupled:guard.response_model=VelocityResponse()
    navigator=LidarNavigator(guard,horizon=horizon,budget=budget,anticipation_distance=anticipation_distance,path_options=path_options)
    xy=lambda t:tuple(np.interp(t,stations,coordinates[:,axis]) for axis in (0,1))
    center=lambda t:float(np.interp(t,stations,heights))
    center.batch=lambda query:np.interp(query,stations,heights)
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
            try:self.connection.send(None)
            except (BrokenPipeError,EOFError,OSError):pass
            self.process.join(timeout=1.)
        if self.process.is_alive():self.process.terminate();self.process.join(timeout=1.)
        self.connection.close()


class LidarNavigator:
    def __init__(self, guard, horizon=24., budget=.06,async_planning=False,process_planning=False,local_replan=False,anticipation_distance=0.,path_options=False):
        self.guard=guard
        self.horizon=horizon
        self.budget=budget
        self.last_path=None
        self.braking_path=None
        self.last_plan_stamp=None
        self.last_cloud_stamp=None
        self.process_planning=process_planning
        self.executor=(GeometryProcess()
                       if async_planning and process_planning else
                       ThreadPoolExecutor(max_workers=1) if async_planning else None)
        self.future=None;self.submitted_at=None;self.future_stamp=None
        self.scene=LidarScene()
        self.reference_coordinates=None
        # Conservative road center prior. Visible wall positions do not
        # identify the hidden force-field boundary at hub transitions.
        self.road_lateral_limit=2.25
        self.prune_candidates=True
        self.height_reserve=.5
        # Response-prefix overshoot allowance for a reference bound the
        # envelope starts inside (measured vertical inertia during reaction
        # latency). Every scenario tail must still return inside the bound.
        self.band_transient_base=.15
        self.band_transient_max=.6
        self.inside_transient=0.
        # Optional base margin (e.g. 1.15) for simulated stop-envelope lag
        # against raw returns. None keeps the full path buffer everywhere.
        # Moving predictions always retain the full buffer.
        self.envelope_margin=None
        # Optional side-pass floor (e.g. 0.9). A tight raw minimum beside or
        # behind the motion may go down to this floor; returns ahead of the
        # trajectory keep the full buffer. None disables the allowance.
        self.side_buffer=None
        self.local_replan=local_replan
        self.anticipation_distance=anticipation_distance
        self.path_options=path_options
        self.last_path_options=[]

    def close(self):
        if self.executor is not None:self.executor.shutdown(wait=self.process_planning)

    def reset(self):
        self.last_path=None
        self.braking_path=None
        self.last_plan_stamp=None
        self.last_cloud_stamp=None
        self.last_path_options=[]
        self.scene.reset()

    def inside_transient_allowance(self,velocity,age):
        """Bounded response-prefix overshoot from measured vertical inertia."""
        latency=self.guard.reaction+max(0.,float(age))
        return min(self.band_transient_max,
                   self.band_transient_base+abs(float(np.asarray(velocity)[2]))*latency)

    def _bound_ok(self,error,timed=False,inside_transient=0.):
        """Recover a reference-bound violation; do not certify hover.

        Timed recovery permits a small transient while the measured residual
        velocity decays. A bound the envelope starts inside may be crossed by
        the measured response prefix (e.g. climbing inertia during reaction
        latency), but only within a bounded allowance and only when every
        scenario tail returns inside the bound. Raw obstacles retain their
        full clearance checks. Geometry and initially satisfied bounds are
        never allowed to create a new, unrecovered violation.
        """
        if np.max(error)<=1e-8:return True
        initial=error[0]
        transient=.001 if timed else 0.
        allowance=max(transient, inside_transient if timed else 0.)
        tails=None
        if timed and self.guard.envelope_times is not None:
            times=self.guard.envelope_times
            starts=np.r_[0,np.flatnonzero(np.diff(times)<0.)+1,len(times)]
            tails=np.concatenate([error[first:last][times[first:last]>=times[first:last].max()-.15]
                                  for first,last in zip(starts[:-1],starts[1:])])
        if initial<=0.:
            if not (timed and inside_transient>0.) or np.max(error)>inside_transient+1e-8:
                return False
            if tails is None:tails=np.array([error[-1]])
            return np.max(tails)<=1e-8
        if np.max(error)>initial+allowance+1e-8:return False
        if tails is None:tails=np.array([error[-1]])
        return np.max(tails)<=max(0.,initial-.01)+1e-8

    def _floor_ok(self, path, stations, center, floor_offset,timed=False):
        if self.guard.response_model is not None:
            heights=np.interp(stations,self.reference_stations,self.reference_heights)
            if not self._bound_ok(path[:,2]-heights-1.25,timed,
                                  inside_transient=self.inside_transient):return False
            if not self._bound_ok(heights-path[:,2]-1.25,timed,
                                  inside_transient=self.inside_transient):return False
            if self.reference_coordinates is not None:
                road=np.column_stack([np.interp(stations,self.reference_stations,self.reference_coordinates[:,axis]) for axis in (0,1)])
                if np.any(np.linalg.norm(path[:,:2]-road,axis=1)>self.road_lateral_limit+1e-8):return False
        if floor_offset is None:return True
        limits=departure_floor_limits(stations,path,center,floor_offset,
                                      self.guard.index,self.guard.margin+.1)
        if self.guard.response_model is None:return np.all(path[:,2]<=limits+1e-8)
        return self._bound_ok(path[:,2]-limits,timed,
                              inside_transient=min(.1,self.inside_transient))

    @staticmethod
    def _command_profile(path):
        """Keep planned altitude when a connector starts at measured XYZ.

        The measured start is needed for geometry, but treating its Z as the
        altitude goal on every cycle erases altitude feedback while slowing.
        Extrapolate the first planned segment back to the measured XY instead.
        """
        profile=np.asarray(path,dtype=float).copy()
        if len(profile)<2:return profile
        profile[0,2]=profile[1,2]
        if len(profile)>2:
            segments=np.diff(profile[1:,:2],axis=0)
            lengths=np.sum(segments**2,axis=1)
            valid=np.flatnonzero(lengths>1e-8)
            if len(valid):
                i=valid[0]+1
                fraction=float((profile[0,:2]-profile[i,:2])@segments[i-1]/lengths[i-1])
                profile[0,2]=profile[i,2]+fraction*(profile[i+1,2]-profile[i,2])
        return profile

    def _bounded_command_profile(self,path,s,xy,center,floor_offset,projection,height_reserve=0.):
        """Keep extrapolated altitude feedback inside the observed safe band.

        A safe geometric connector does not certify its backwards height
        extrapolation. In particular it can point below the departure floor.
        Check the vertical connector at the measured XY before using that
        altitude in every candidate's stopping controller.
        """
        profile=self._command_profile(path)
        if len(profile)<2:return profile
        if height_reserve>0.:
            stations=self._route_stations(profile,s,xy,projection)
            heights=np.interp(stations,self.reference_stations,self.reference_heights)
            band=1.25-height_reserve
            profile[:,2]=np.clip(profile[:,2],heights-band,heights+band)
        station=self._route_stations(profile[:1],s,xy,projection)
        height=float(np.interp(station,self.reference_stations,self.reference_heights)[0])
        floor=float(departure_floor_limits(station,profile[:1],center,floor_offset,
                    self.guard.index,self.guard.margin+.1)[0])
        low=height-1.25+.02;high=min(height+1.25,floor)-.02
        if low<=high:profile[0,2]=np.clip(profile[0,2],low,high)
        vertical=np.array([path[0],profile[0]],dtype=float)
        samples,_,_=swept_samples(vertical)
        stations=self._route_stations(samples,s,xy,projection)
        if (not self._floor_ok(samples,stations,center,floor_offset) or
                self.guard.index.distance(samples,limit=self.guard.margin+.1).min()<self.guard.margin+.1):
            # Preserve the geometrically checked measured anchor if no local
            # vertical connector is certified. Final response checks still
            # decide whether any command can be issued.
            profile[0,2]=path[0,2]
        return profile

    @staticmethod
    def _stop_profile(path,gain=1.):
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
            if np.all(length<1e-8):return np.clip(gain*(path[-1,2]-positions[:,2]),-4.,4.5)
            reference=path[nearest,2]+fraction[np.arange(len(nearest)),nearest]*vertical[nearest]
            along=arc[nearest]+fraction[np.arange(len(nearest)),nearest]*np.sqrt(length[nearest])
            lo=np.maximum(0.,along-2.);hi=np.minimum(arc[-1],along+2.)
            direction=np.column_stack([np.interp(hi,arc,path[:,axis])-np.interp(lo,arc,path[:,axis]) for axis in (0,1)])
            direction/=np.maximum(1e-8,np.linalg.norm(direction,axis=1))[:,None]
            slope=(np.interp(hi,arc,path[:,2])-np.interp(lo,arc,path[:,2]))/np.maximum(1e-8,hi-lo)
            ff=np.sum(velocities[:,:2]*direction,axis=1)*slope
            return np.clip(ff+gain*(reference-positions[:,2]),-4.,4.5)
        target.path=path
        target.height_gain=gain
        return target

    def _tracking_profile(self,path):
        ref=self.reference_stations;coords=self.reference_coordinates
        if self.guard.response_model.height_gain<=1. or ref is None or coords is None:
            return self._stop_profile(path,gain=self.guard.response_model.height_gain)
        xy=lambda t:tuple(np.interp(t,ref,coords[:,a]) for a in (0,1))
        projection=self._projection(ref[0]+1.,xy)
        stations=self._route_stations(path,ref[0]+1.,xy,projection)
        height_path=path.copy()
        height_path[:,:2]=np.column_stack([np.interp(stations,ref,coords[:,a]) for a in (0,1)])
        target=self._stop_profile(height_path,gain=self.guard.response_model.height_gain)
        target.path=path;target.height_path=height_path
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
        model=self.guard.response_model
        if model is not None and model.native is not None and model.native.library is not None:
            return model.native.project(queries,(stations,road,segment,length))
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
        # Prefer clearance before tracking uncertainty consumes the buffer.
        # The 1.25 m feasibility threshold and final response checks remain.
        # Only distances inside the 1.85 m cost target affect this search.
        # Exact bounded queries avoid searching far-away surfaces at every cell.
        distances=np.minimum(1.85,index.distance(world.reshape(-1,3),limit=1.85)).reshape(world.shape[:-1])
        valid=distances>=buffer
        if self.guard.response_model is not None:
            projected_heights=np.array([center(t) for t in projection[0]])
            road_z=np.interp(node_stations,projection[0],projected_heights).reshape(valid.shape)
            road_xy=np.column_stack([np.interp(node_stations,projection[0],projection[1][:,axis])
                                     for axis in (0,1)])
            road_distance=np.linalg.norm(world.reshape(-1,3)[:,:2]-road_xy,axis=1).reshape(valid.shape)
            valid&=abs(world[:,:,:,2]-road_z)<=1.25
            # Tangent offsets are not exact distances to a curved reference.
            # Use the same projected coordinates as swept/final validation,
            # so all Z layers of an invalid lateral node are removed at once.
            valid&=road_distance<=2.25+1e-8
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
        base_penalty=.03*zz**2+4.*np.maximum(0.,1.85-distances)**2
        if self.anticipation_distance>0.:
            # Future occupancy is a soft cost that starts a bypass earlier.
            # Actual nodes, swept edges and full command stops still certify
            # only their own positions with the unchanged measured geometry.
            future_stations=stations+self.anticipation_distance
            future_nominal=np.array([[*xy(t)[:2],center(t)] for t in future_stations])
            future_forward=np.array([np.asarray(xy(t+.3)[:2])-np.asarray(xy(t-.3)[:2]) for t in future_stations])
            future_forward/=np.maximum(1e-6,np.linalg.norm(future_forward,axis=1))[:,None]
            future_side=np.column_stack((-future_forward[:,1],future_forward[:,0],np.zeros(len(stations))))
            probes=future_nominal[:,None,None,:]+yy[None,:,:,None]*future_side[:,None,None,:]
            probes[:,:,:,2]+=zz
            future_clearance=index.distance(probes.reshape(-1,3),limit=1.85).reshape(valid.shape)
            base_penalty+=np.maximum(0.,1.85-future_clearance)**2
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
        self.reference_stations=projection[0];self.reference_coordinates=projection[1]
        self.reference_heights=np.array([center(t) for t in projection[0]])
        self.road_lateral_limit=max(2.25,float(np.linalg.norm(position[:2]-nominal[0,:2])))
        # Share observed occupancy across center/left/right soft objectives.
        # Each branch retains independent edge rejection and full sweep checks.
        choices=[];original_valid=valid.copy()
        for preference in ((None,-1.25,1.25) if self.path_options else (None,)):
            valid=original_valid.copy()
            penalty=base_penalty+(.015*yy**2 if preference is None else .12*(yy-preference)**2)
            attempts=0;rejections=[];edge_costs={};search_started=time.monotonic()
            seed_costs=None;staging_tried=False;force_staging=False
            # Occupancy construction may exceed the retry budget under simulator
            # load. Always attempt one complete, certified path before giving up.
            while attempts<12 and (attempts==0 or force_staging or time.monotonic()-search_started<self.budget):
                force_staging=False
                attempts+=1
                costs=np.full((len(stations),ny,nz),np.inf)
                parent=np.full((len(stations),ny,nz),-1,dtype=np.int16)
                if not valid[0,iy,iz]:break
                costs[0,iy,iz]=0.
                if seed_costs is not None:
                    costs[0]=seed_costs.copy();costs[0,~valid[0]]=np.inf
                last=0
                for i in range(1,len(stations)):
                    costs[i],parent[i]=self._advance_grid(costs[i-1],neighbors,edge_costs.get(i,shifts))
                    costs[i]+=penalty[i]
                    costs[i,~valid[i]]=np.inf
                    if not np.any(np.isfinite(costs[i])):break
                    last=i
                if last<2:
                    if self.guard.response_model is not None and not staging_tried:
                        # The first forward step may be blocked while a measured
                        # in-place connector can reach a forward corridor. Every
                        # connector obeys the same continuous bounds and buffer.
                        staging_tried=True;seed_costs=np.full(shape,np.inf);seed_costs[iy,iz]=0.
                        for j,k in np.argwhere(valid[0]):
                            target=world[0,j,k]
                            distance=float(np.linalg.norm(target-position))
                            if distance<.05:continue
                            connector=swept_samples(np.array([position,target]))[0]
                            connector_s=self._route_stations(connector,s,xy,projection)
                            if not self._floor_ok(connector,connector_s,center,floor_offset):continue
                            if index.distance(connector,limit=buffer).min()<buffer:continue
                            seed_costs[j,k]=.18*(ys[j]-current_y)**2+.4*(zs[k]-current_z)**2+.08*distance
                        if np.count_nonzero(np.isfinite(seed_costs))>1:
                            force_staging=True;continue
                    break
                j,k=np.unravel_index(np.argmin(costs[last]),shape)
                nodes=[]
                for i in range(last,-1,-1):
                    nodes.append((i,j,k))
                    if i:j,k=divmod(int(parent[i,j,k]),nz)
                nodes.reverse();path=np.array([world[node] for node in nodes])
                connector_count=int(np.linalg.norm(path[0]-position)>1e-8)
                if connector_count:path=np.concatenate(([position],path))
                queries,segments,fractions=swept_samples(path)
                clearance=np.minimum(1.75,index.distance(queries,limit=1.75))
                along=None;lateral_bad=None
                failures=np.flatnonzero(clearance<buffer)
                if floor_offset is not None or self.guard.response_model is not None:
                    along=self._route_stations(queries,s,xy,projection)
                    floor=departure_floor_limits(along,queries,center,floor_offset,index,buffer)
                    floor_error=queries[:,2]-floor
                    failures=np.flatnonzero((clearance<buffer)|(floor_error>max(0.,floor_error[0])+1e-8))
                    if not self._bound_ok(floor_error):failures=np.r_[failures,len(queries)-1]
                    if self.guard.response_model is not None:
                        road_height=np.interp(along,projection[0],projected_heights)
                        road_xy=np.column_stack([np.interp(along,projection[0],projection[1][:,axis]) for axis in (0,1)])
                        upper=queries[:,2]-road_height-1.25;lower=road_height-queries[:,2]-1.25
                        lateral_bad=np.linalg.norm(queries[:,:2]-road_xy,axis=1)>max(2.25,float(np.linalg.norm(position[:2]-nominal[0,:2])))+1e-8
                        failures=np.flatnonzero((clearance<buffer)|(floor_error>max(0.,floor_error[0])+1e-8)|
                            (upper>max(0.,upper[0])+1e-8)|(lower>max(0.,lower[0])+1e-8)|
                            lateral_bad)
                        if not all(self._bound_ok(e) for e in (floor_error,upper,lower)):failures=np.r_[failures,len(queries)-1]
                if not len(failures):
                    if not any(np.array_equal(path,previous) for previous,_ in choices):
                        choices.append((path,dict(path_distance=float(last),path_clearance=float(np.min(clearance)),
                                         staging=bool(connector_count),grid_attempts=attempts,grid_ms=1000.*(time.monotonic()-started),path_preference=preference)))
                    break
                bad=max(0,min(last,int(segments[failures[0]])+1-connector_count))
                lateral=(lateral_bad is not None and lateral_bad[failures[0]] and clearance[failures[0]]>=buffer)
                rejections.append(dict(station=float(stations[max(0,min(last,int(segments[failures[0]])-connector_count))]) if along is None else float(along[failures[0]]),
                    clearance=float(clearance[failures[0]]),position=queries[failures[0]].tolist(),
                    reason='ROAD_LATERAL' if lateral else 'GEOMETRY_OR_HEIGHT'))
                if lateral and bad>0:
                    # XY edge geometry is identical across all vertical layers.
                    # Reject that XY transition, retaining the node for another
                    # safe parent, rather than retrying the same arc at every Z.
                    previous_y=nodes[bad-1][1];destination_y=nodes[bad][1]
                    edge_cost=edge_costs.setdefault(bad,shifts.copy())
                    edge_cost[destination_y][neighbors[destination_y]//nz==previous_y]=np.inf
                else:valid[nodes[bad]]=False

        if choices:
            path,info=choices[0]
            info=dict(info,path_options=[dict(detail,path=option.tolist()) for option,detail in choices[1:]],
                      grid_ms=1000.*(time.monotonic()-started))
            return path,info
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

    def _commands(self, position, velocity, desired, path, age, s, xy, center, floor_offset,projection=None,height_reserve=None):
        """Score executed XYZ responses and full stopping paths, not rays."""
        buffer=self.guard.margin+.1
        if height_reserve is None:height_reserve=self.height_reserve
        if projection is None:projection=self._projection(s,xy)
        distances=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
        lookaheads=np.unique(np.minimum(distances[-1],np.array([1.,2.,5.,10.,16.] if self.guard.response_model is not None else [.5,1.,2.,4.,6.])))
        # A map/safety HOLD must not acquire a new forward speed from the
        # geometry lookaheads. Counter-braking remains response-model driven.
        speed=float(np.linalg.norm(desired[:2]))
        model=self.guard.response_model
        levels=((1.,.875,.75,.5,0.) if model is not None and model.coupling_limited else
                (1.,.5,0.) if model is not None else (1.,.9375,.875,.75,.5,0.))
        lookahead_levels=((1.,.75,.5) if model is not None and model.coupling_limited else
                          (1.,.5) if model is not None else (1.,.75,.5,.25))
        commands=[desired.copy()*factor for factor in levels]
        for ahead in lookaheads:
            goal=np.array([np.interp(ahead,distances,path[:,axis]) for axis in range(3)])
            delta=goal-position;horizontal=max(.05,float(np.linalg.norm(delta[:2])))
            for factor in lookahead_levels:
                forward=min(speed*factor,max(.5,horizontal*1.5))
                cmd=np.r_[delta[:2]*forward/horizontal,np.clip(delta[2]*forward/horizontal,-4.,4.5)]
                commands.append(cmd)
        if self.guard.response_model is not None:
            model=self.guard.response_model
            model.stop_profile=self._tracking_profile(self._bounded_command_profile(path,s,xy,center,floor_offset,projection,height_reserve))
            prepared=[]
            for target in commands:
                command=model.prepare(target,velocity,self.guard.pose_stamp,position)
                # Use measured motion and the XY command actually sent for
                # grade feed-forward, including inertia during counter-braking.
                responding=command.copy()
                responding[:2]=.6*velocity[:2]+.4*command[:2]
                vertical=model.stop_profile(np.array([position]),np.array([responding]))[0]
                command[2]=vertical
                command=model.compensate(command,velocity)
                prepared.append(command)
            commands=prepared
        commands=np.unique(np.round(commands,6),axis=0)
        best=None;best_score=-np.inf;best_clear=None;best_index=len(commands)
        first=np.asarray(xy(s+.3)[:2])-np.asarray(xy(s-.3)[:2]);first/=max(1e-6,np.linalg.norm(first))
        envelopes=None
        if self.guard.response_model is not None:
            envelopes=self.guard.response_model.envelopes(position,velocity,commands,self.guard.reaction+age)
        score_terms=[]
        for command in commands:
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
            gain=2.*(before-after)+.3*progress-2.*path_error
            penalty=.015*np.sum((command-desired)**2)
            score_terms.append((gain,penalty,gain+.12*2.-penalty))
        order=range(len(commands))
        pruning=envelopes is not None and self.prune_candidates
        if pruning:order=sorted(order,key=lambda i:(-score_terms[i][2],i))
        for i in order:
            command=commands[i];gain,penalty,upper=score_terms[i]
            # Clearance contributes at most .12*2. Preserve exact arithmetic
            # order and the original first-candidate tie rule. A candidate
            # that cannot beat a fully certified choice needs no point query.
            if pruning and (upper<best_score or (upper==best_score and i>=best_index)):continue
            if envelopes is None:
                samples,extent=self.guard.command_envelope(position,velocity,command,age)
            else:samples,extent,self.guard.envelope_times=envelopes[i]
            if extent>self.guard.horizon:continue
            if floor_offset is not None or self.guard.response_model is not None:
                sample_s=self._route_stations(samples,s,xy,projection)
                if not self._floor_ok(samples,sample_s,center,floor_offset,timed=self.guard.response_model is not None):continue
            raw,dynamic=self.guard.command_clearance_components(samples,limit=2.)
            clearance=min(raw,dynamic)
            if clearance<buffer:
                # The commanded path keeps the full buffer. The simulated
                # stop envelope may dip to the configured base margin when
                # the lag is only against raw returns, or to the side-pass
                # floor when the tight return is beside/behind the motion.
                # Moving predictions always keep the full buffer.
                admitted=bool(dynamic>=buffer and (
                    (self.envelope_margin is not None and raw>=self.envelope_margin) or
                    (self.side_buffer is not None and raw>=self.side_buffer and
                     self.guard.close_pass_ok(samples,self.side_buffer))))
                if not admitted:continue
            score=gain+.12*min(2.,clearance)-penalty
            if score>best_score or (score==best_score and i<best_index):
                best_score=score;best=command;best_clear=clearance;best_index=i
        if best is None and envelopes is not None and height_reserve>0.:
            original=self._bounded_command_profile(path,s,xy,center,floor_offset,projection)
            if np.array_equal(original,self.guard.response_model.stop_profile.path):
                return best,best_clear,len(commands)
            # A near-boundary bypass may require the original height. Retain
            # it only if the same complete response checks certify a command.
            command,clearance,count=self._commands(position,velocity,desired,path,age,
                s,xy,center,floor_offset,projection,height_reserve=0.)
            return command,clearance,len(commands)+count
        return best,best_clear,len(commands)

    def _thread_command(self,position,velocity,desired,age,points,cloud_stamp,pose_stamp):
        """Certified forward creep when the measured pose already violates the
        full buffer.

        The aircraft may keep moving only while no return gets closer than
        min(its current distance, buffer) and every scenario tail improves.
        Same monotone rule as the in-place recovery, applied to forward
        threading instead of forcing a stop-and-escape.
        """
        guard=self.guard;model=guard.response_model
        if model is None or age is None or points is None:
            return None
        targets=[np.asarray(desired,dtype=float)*factor for factor in (1.,.75,.5,.25)]
        for target in targets:
            command=model.prepare(target,velocity,pose_stamp,position)
            vertical=(0. if model.stop_profile is None else
                      float(model.stop_profile(np.array([position]),
                                               np.array([velocity]))[0]))
            command[2]=vertical
            command=model.compensate(command,velocity)
            if float(np.linalg.norm(command[:2]))<.05:
                continue
            if guard.recovery_command_ok(position,velocity,command,points,age):
                return command,dict(feasible=True,command_reason='LIDAR_THREAD',
                                    command_scale=None,
                                    thread_speed=float(np.linalg.norm(command[:2])))
        return None

    def _stationary_escape(self,position,velocity,s,xy,center,points,cloud_stamp,pose_stamp,
                           floor_offset,info,started,departure=False):
        if np.linalg.norm(np.asarray(velocity)[:2])<.8 and self.guard.index.distance([position])[0]<self.guard.margin+.1:
            a,b=np.asarray(xy(s-.3)[:2]),np.asarray(xy(s+.3)[:2])
            forward=(b-a)/max(1e-6,np.linalg.norm(b-a));side=np.r_[-forward[1],forward[0],0.]
            candidates=[position+np.array([0.,0.,z]) for z in (-.25,.25,-.5,.5,-1.,1.,-1.5,1.5,-2.,2.)]
            candidates+=[position+y*side+np.array([0.,0.,z]) for y in (-.5,.5,-1.,1.,-2.,2.) for z in (0.,-.5,.5)]
            # A face across the road cannot be escaped by shifting parallel
            # to it. Check short longitudinal moves against every return too;
            # no persistent retreat target or unchecked backing is introduced.
            along=np.r_[forward,0.]
            candidates+=[position+distance*along+np.array([0.,0.,z])
                         for distance in (-.5,-1.,-1.5,-2.,.5,1.,1.5,2.) for z in (0.,-.5,.5)]
            # Refine nearby endpoints only after the existing coarse moves
            # fail. A narrow road/floor interval can fall between their
            # 0.5 m offsets. Endpoint clearance only prunes the search; every
            # selected move still needs the complete conditional stop check.
            def recovery_targets():
                for target in candidates:yield target,False
                if self.guard.response_model is None or not self.guard.response_model.coupling_limited:return
                fine=np.array([position+a*along+y*side+np.array([0.,0.,z])
                               for a in (-.25,0.,.25)
                               for y in np.arange(-.5,.501,.125)
                               for z in np.arange(-.25,.251,.125)])
                clear=self.guard.index.distance(fine,limit=self.guard.margin+.3)
                eligible=np.flatnonzero(clear>=self.guard.margin+.3)
                costs=np.sum((fine[eligible]-position)**2,axis=1)
                for i in eligible[np.argsort(costs,kind='stable')]:
                    yield fine[i],True
                # A gate crossbar can require simultaneous retreat and
                # descent; axis-only moves approach another part of the
                # same frame. Prune endpoints and the close-return cone
                # together before the unchanged complete response checks.
                combined=np.array([position+a*along+y*side+np.array([0.,0.,z])
                    for a in (-2.,-1.,-.5,0.,.5,1.,2.)
                    for y in (-2.,-1.,-.5,0.,.5,1.,2.)
                    for z in (-2.,-1.5,-1.,-.5,0.,.5,1.,1.5,2.)])
                clear=self.guard.index.distance(combined,limit=self.guard.margin+.3)
                eligible=np.flatnonzero(clear>=self.guard.margin+.3)
                relative=np.asarray(points)-position
                close=relative[np.linalg.norm(relative,axis=1)<self.guard.margin+.1]
                if len(close) and len(eligible):
                    eligible=eligible[np.all((combined[eligible]-position)@close.T<=1e-7,axis=1)]
                costs=np.sum((combined[eligible]-position)**2,axis=1)
                for i in eligible[np.argsort(costs,kind='stable')]:yield combined[i],True
            for target,refined in recovery_targets():
                stations=np.array([s,s])
                if not self._floor_ok(np.array([position,target]),stations,center,floor_offset):continue
                escape=self.guard.escape(position,velocity,target,points,cloud_stamp,pose_stamp)
                if escape is not None:
                    if self.guard.response_model is not None:
                        self.braking_path=self.guard.response_model.stop_profile.path.copy()
                    return escape,dict(info,feasible=True,command_reason='LIDAR_ESCAPE',command_scale=None,
                        recovery_target=target.tolist(),conditional_recovery=self.guard.response_model is not None,
                        recovery_search_refined=refined,
                        navigation_ms=1000.*(time.monotonic()-started))
        return None

    def _brake(self,position,velocity,s,xy,center,points,cloud_stamp,pose_stamp,
               floor_offset,projection,info,started):
        if self.guard.response_model is not None:
            stopping_path=(self.braking_path.copy() if self.braking_path is not None else
                np.array([[*xy(t)[:2],center(t)] for t in self.reference_stations]))
            stopping_stations=self._route_stations(stopping_path,s,xy,projection)
            heights=np.interp(stopping_stations,self.reference_stations,self.reference_heights)
            stopping_path[:,2]=np.clip(stopping_path[:,2],heights-1.25,heights+1.25)
            floors=departure_floor_limits(stopping_stations,stopping_path,center,floor_offset,self.guard.index,self.guard.margin+.1)
            stopping_path[:,2]=np.minimum(stopping_path[:,2],floors-.02)
            self.guard.response_model.stop_profile=self._tracking_profile(stopping_path)
            vertical=float(self.guard.response_model.stop_profile(np.array([position]),np.array([velocity]))[0])
        else:vertical=0.
        command,checked=self.guard.filter_command(position,velocity,np.array([0.,0.,vertical]),points,cloud_stamp,pose_stamp)
        return command,dict(info,**checked,feasible=False,navigation_ms=1000.*(time.monotonic()-started))

    def select(self, position, velocity, desired, s, xy, center, points, cloud_stamp,
               pose_stamp, floor_offset=None, recovery_only=False):
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
        self.inside_transient=(0. if age is None else
            self.inside_transient_allowance(velocity,age))
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
                        height_gain=self.guard.response_model.height_gain,
                        coupling_limited=self.guard.response_model.coupling_limited,
                        xy_error_max=(self.guard.response_model.xy_error_max if np.isfinite(self.guard.response_model.xy_error_max) else None),
                        control_periods=(None if self.guard.response_model.control_periods is None else sorted(set(self.guard.response_model.control_periods))),
                        response_scenarios=len(self.guard.response_model.scenarios),
                        vertical_command_max=self.guard.response_model.vertical_command_max,
                        acceleration=self.guard.response_model.acceleration,
                        latency_motion=('coast-and-applied' if self.guard.response_model.applied_command is not None else 'coast'),
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
        if recovery_only:
            # Departure may start inside a measured buffer. Use the same
            # per-surface, full-stop recovery checks without forward tracking.
            recovered=self._stationary_escape(position,velocity,s,xy,center,points,cloud_stamp,
                pose_stamp,floor_offset,info,started,departure=True)
            if recovered is not None:return recovered
            command,checked=self.guard.filter_command(position,velocity,np.asarray(desired),
                points,cloud_stamp,pose_stamp)
            return command,dict(info,**checked,feasible=False,
                                navigation_ms=1000.*(time.monotonic()-started))
        if self.guard.index.distance([position],limit=self.guard.margin+.1)[0]<self.guard.margin+.1:
            # Every ordinary response includes the measured starting pose.
            # Its full buffer is already violated, so no normal path or
            # command candidate can be certified. Keep the same measured
            # scene, road bounds, conditional recovery and braking checks.
            info.update(initial_buffer_blocked=True,candidate_count=0,path_option_count=0)
            # Keep moving through a passable gate/gap first: a forward creep
            # is legal while it never decreases any close return's distance.
            threaded=self._thread_command(position,velocity,np.asarray(desired),age,
                points,cloud_stamp,pose_stamp)
            if threaded is not None:
                command,detail=threaded
                return command,dict(info,**detail,path_clearance=None,
                                    navigation_ms=1000.*(time.monotonic()-started))
            recovered=self._stationary_escape(position,velocity,s,xy,center,points,cloud_stamp,
                pose_stamp,floor_offset,info,started)
            if recovered is not None:return recovered
            return self._brake(position,velocity,s,xy,center,points,cloud_stamp,pose_stamp,
                floor_offset,projection,info,started)
        if self.executor is not None:
            if self.future is not None and self.future.done():
                completed=self.future;self.future=None
                try:
                    ready,ready_info=completed.result()
                except Exception as error:
                    ready=None;ready_info=dict(planner_error=str(error))
                options=ready_info.pop('path_options',[])
                details.update(ready_info)
                if ready is not None:
                    self.last_path=ready;self.last_plan_stamp=self.future_stamp
                    self.last_path_options=options
            if self.future is None and (self.submitted_at is None or time.monotonic()-self.submitted_at>=.15):
                self.submitted_at=time.monotonic()
                self.future_stamp=pose_stamp
                ss=np.arange(s-1.,s+self.horizon+self.anticipation_distance+1.1,.5)
                coordinates=np.array([xy(t)[:2] for t in ss]);heights=np.array([center(t) for t in ss])
                frozen_xy=lambda t,stations=ss,coords=coordinates:tuple(np.interp(t,stations,coords[:,axis]) for axis in (0,1))
                frozen_center=lambda t,stations=ss,z=heights:float(np.interp(t,stations,z))
                frozen_center.batch=lambda query,stations=ss,z=heights:np.interp(query,stations,z)
                if self.process_planning:
                    snapshot=(np.asarray(position).copy(),s,ss,coordinates,heights,floor_offset,
                        self.guard.index,self.guard.margin,self.horizon,self.budget,self.guard.response_model is not None,
                        self.anticipation_distance,self.path_options)
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
        if path is None and self.executor is not None and self.local_replan:
            # A pending/rejected full snapshot must not freeze a currently
            # clear departure. Rebuild only a short prefix in the live index.
            # Keep this navigator's full reference and complete stop checks.
            local=LidarNavigator(self.guard,horizon=12.,budget=.03,anticipation_distance=self.anticipation_distance)
            path,local_info=local.path(np.asarray(position),s,xy,center,floor_offset)
            details.update(local_replan=True,local_grid_ms=local_info['grid_ms'],
                           local_path_distance=local_info['path_distance'])
            if path is not None:
                details.update(local_info,cached=False,truncated=False)
                self.last_plan_stamp=pose_stamp;self.last_cloud_stamp=cloud_stamp
        info.update(details,anticipation_distance=self.anticipation_distance)
        info['planner_busy']=self.future is not None
        command=None;count=0
        candidates=[]
        if path is not None:candidates.append((path,{}))
        # Alternative geometry is normally built only in the worker. Connect
        # each fallback to the current pose and certify its complete response.
        if self.path_options:
            options=details.pop('path_options',[]) if self.executor is None else self.last_path_options
            for option in options:
                original=np.asarray(option['path'])
                nearest=int(np.argmin(np.sum((original-position)**2,axis=1)))
                alternative=np.concatenate(([position],original[min(nearest+1,len(original)-1):]))
                if not any(np.array_equal(alternative,previous) for previous,_ in candidates):
                    candidates.append((alternative,{key:value for key,value in option.items() if key!='path'}))
        else:details.pop('path_options',None)
        info.pop('path_options',None)
        info['path_option_count']=len(candidates)
        for choice,(candidate,option_info) in enumerate(candidates):
            if choice or path is None:
                candidate,prefix=self._usable_path(candidate,position,s,xy,center,floor_offset,projection)
                if candidate is None:continue
                option_info.update(prefix)
            trial,clearance,trial_count=self._commands(position,velocity,np.asarray(desired),candidate,age,s,xy,center,floor_offset,projection)
            count+=trial_count
            if trial is None:continue
            command=trial;path=candidate;self.last_path=path
            info.update(option_info,candidate_count=count,path=path.tolist(),planned_stamp=self.last_plan_stamp,
                        geometry_cloud_stamp=cloud_stamp,plan_age=pose_stamp-self.last_plan_stamp,
                        alternative_path_used=bool(option_info))
            self.braking_path=(self.guard.response_model.stop_profile.path.copy()
                               if self.guard.response_model is not None else self._command_profile(path))
            info['command_height_reference']=float(self.braking_path[0,2])
            reason='LIDAR_TRACK' if np.linalg.norm(command[:2])>.1 else 'LIDAR_BRAKE'
            return command,dict(info,feasible=True,command_reason=reason,command_scale=None,
                command_clearance=clearance,navigation_ms=1000.*(time.monotonic()-started))
        if path is not None:
            self.last_path=path
            info.update(candidate_count=count,path=path.tolist(),planned_stamp=self.last_plan_stamp,
                        geometry_cloud_stamp=cloud_stamp,plan_age=pose_stamp-self.last_plan_stamp)
        # If already inside a buffer, certify a monotonic escape directly.
        # No persistent target, retreat-pending latch or asynchronous join.
        old_height_target=(None if self.braking_path is None else
            float(position[2])+float(self._stop_profile(self.braking_path)(np.array([position]),np.zeros((1,3)))[0]))
        if (self.guard.response_model is not None and old_height_target is not None and
                np.linalg.norm(velocity)<.8 and abs(float(position[2])-center(s))>=.4 and
                abs(old_height_target-center(s))>=.3):
            # A detour's stopping altitude can differ from the road reference.
            # Once stopped, certify a local vertical return before replacing
            # that profile; do not re-anchor its goal to the measured height.
            recovery=np.array([position,position],dtype=float)
            recovery[-1,2]=center(s)
            recovery_samples,_,_=swept_samples(recovery)
            recovery_stations=self._route_stations(recovery_samples,s,xy,projection)
            if (self.guard.index.distance(recovery_samples,limit=self.guard.margin+.1).min()>=self.guard.margin+.1 and
                    self._floor_ok(recovery_samples,recovery_stations,center,floor_offset)):
                model=self.guard.response_model;previous_profile=model.stop_profile
                model.stop_profile=self._tracking_profile(recovery)
                vertical=float(model.stop_profile(np.array([position]),np.array([velocity]))[0])
                recovered,checked=self.guard.filter_command(position,velocity,np.array([0.,0.,vertical]),
                                                            points,cloud_stamp,pose_stamp)
                if checked['command_reason']=='COMMAND_BRAKING':
                    self.braking_path=recovery
                    return recovered,dict(info,**dict(checked,command_reason='LIDAR_HEIGHT_RECOVERY'),
                        feasible=True,path=recovery.tolist(),height_recovery_target=float(recovery[-1,2]),
                        navigation_ms=1000.*(time.monotonic()-started))
                model.stop_profile=previous_profile
        recovered=self._stationary_escape(position,velocity,s,xy,center,points,cloud_stamp,
            pose_stamp,floor_offset,info,started)
        if recovered is not None:return recovered
        return self._brake(position,velocity,s,xy,center,points,cloud_stamp,pose_stamp,
            floor_offset,projection,info,started)
