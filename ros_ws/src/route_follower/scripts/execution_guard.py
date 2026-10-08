#!/usr/bin/env python3
"""Short-path recertification independent of the expensive route search."""
import math
import time
import numpy as np
from point_index import PointIndex
from predictive_avoidance import PredictiveAvoidance
from path_sampling import swept_samples


class ExecutionGuard:
    def __init__(self, margin=1.15, braking=4., reaction=.35, horizon=30., settling=.8):
        self.margin=margin
        self.braking=braking
        self.reaction=reaction
        self.horizon=horizon
        # Recorded zero commands still leave approximately 0.8 s of velocity
        # settling in SimpleFlight. Configured command acceleration is not the
        # aircraft's instantaneous physical braking acceleration.
        self.settling=settling
        self.cloud_stamp=None
        self.index=None
        self.faces=()
        self.response_model=None
        self.envelope_times=None
        self.pose_stamp=None
        self.command_constraint=None
        self.dynamic_scene=None
        self.surface_seed_points=None

    def set_faces(self,faces):
        self.faces=faces
        if self.index is not None:self.index.faces=faces

    def _update(self, points, cloud_stamp, pose_stamp, position=None):
        self.pose_stamp=pose_stamp
        age=None if cloud_stamp is None else pose_stamp-cloud_stamp
        if points is None or age is None or not 0.<=age<.5:
            return None, 'LIDAR_STALE'
        if len(points)==0:return None, 'LIDAR_EMPTY'
        if self.index is None or self.cloud_stamp!=cloud_stamp:
            self.index=PointIndex(points,origin=position,faces=self.faces,road_height=5.,
                                  continuous_surfaces=self.response_model is not None,
                                  surface_seeds=self.surface_seed_points)
            self.cloud_stamp=cloud_stamp
        return age, None

    def command_clearance(self,samples,limit=None):
        distances=self.index.distance(samples,limit=limit)
        if limit is not None:distances=np.minimum(limit,distances)
        if self.dynamic_scene is not None and self.envelope_times is not None:
            distances=np.minimum(distances,self.dynamic_scene.distance(samples,self.envelope_times))
        return float(np.min(distances))

    def command_clearance_components(self,samples,limit=None):
        """Raw-return and moving-prediction minima separately.

        Callers may admit model lag against raw returns down to the base
        margin while moving predictions keep the full planning buffer.
        """
        distances=self.index.distance(samples,limit=limit)
        raw=float(np.min(distances))
        dynamic=float('inf')
        if self.dynamic_scene is not None and self.envelope_times is not None:
            dynamic=float(np.min(self.dynamic_scene.distance(samples,self.envelope_times)))
        return raw,dynamic

    def close_pass_ok(self,samples,floor):
        """Admit a tight raw minimum only when it is a side pass.

        The closest return must lie beside or behind the trajectory at its
        closest-approach sample and must not keep closing afterwards. A
        return ahead of the motion keeps the full buffer.
        """
        samples=np.asarray(samples,dtype=float)
        if len(samples)<3:return False
        distances=self.index.distance(samples,limit=2.)
        k=int(np.argmin(distances))
        if distances[k]<floor:return False
        _,ids=self.index.neighbors(samples[k:k+1],1)
        index=int(ids[0,0])
        if index>=len(self.index.points):return False
        nearest=np.asarray(self.index.points[index],dtype=float)
        direction=samples[min(k+1,len(samples)-1)]-samples[max(k-1,0)]
        if float(np.dot(nearest-samples[k],direction))>0.:return False
        later=np.linalg.norm(samples[k+1:]-nearest,axis=1)
        if len(later) and float(np.min(later))<distances[k]-1e-6:return False
        return True

    def _cap(self, distance, latency):
        accelerated=max(0.,math.sqrt((self.braking*latency)**2+2.*self.braking*distance)-self.braking*latency)
        settling=distance/max(1e-6,latency+self.settling)
        return min(accelerated,settling)

    @staticmethod
    def route_cap(verified, planned):
        cap=verified['cap']
        # A recovery ray describes a different trajectory. Its zero forward
        # speed cannot freeze a separately certified executing detour.
        if (planned.get('source')=='PREDICTIVE' and planned.get('feasible') and
                planned.get('join_accepted') is True and planned.get('cap') is not None):
            cap=min(cap,planned['cap'])
        return cap

    @staticmethod
    def resume_after_retreat(verified,planned,margin=1.15):
        return (planned.get('source')=='PREDICTIVE' and planned.get('feasible') and
                planned.get('join_accepted') is True and verified.get('execution_verified') and
                verified.get('verified_distance',0.)>=12. and
                verified.get('execution_clearance',0.)>=margin+.1)

    def motion_envelope(self, position, velocity, latency):
        """Brake along measured 3D motion, including lateral/vertical drift.

        The route can bend away from a car while the aircraft still slides
        toward it. Its stopping ray must be checked independently of the route.
        """
        velocity=np.asarray(velocity,dtype=float)
        speed=float(np.linalg.norm(velocity))
        stop=speed*latency+max(speed*speed/(2.*self.braking),speed*self.settling)
        if speed<.05:return dict(cap=float('inf'),distance=None,clearance=None,stop=stop)
        length=min(self.horizon,max(2.,stop+2.))
        along=np.linspace(0.,length,max(2,int(math.ceil(length/.15))+1))
        distances=self.index.distance(position+along[:,None]*velocity/speed)
        blocked=np.flatnonzero(distances<self.margin+.1)
        free=max(0.,float(along[blocked[0]])-.15) if len(blocked) else length
        return dict(cap=self._cap(free,latency) if len(blocked) or stop>length else float('inf'),
                    distance=free,clearance=float(np.min(distances)),stop=stop)

    def command_envelope(self,position,velocity,command,age):
        """Shared actuator response, reaction, immediate target and full stop."""
        if self.response_model is not None:
            queries,extent,times=self.response_model.envelope(position,velocity,command,self.reaction+age)
            self.envelope_times=times
            return queries,extent
        position=np.asarray(position);velocity=np.asarray(velocity);command=np.asarray(command)
        latency=self.reaction+age
        speed=float(np.linalg.norm(command))
        tau=max(self.settling,float(np.linalg.norm(velocity))/(2.*self.braking),speed/(2.*self.braking),.01)
        hold=self.reaction
        response=position+velocity*latency
        times=np.linspace(0.,hold,max(3,int(hold/.05)+1))
        driven=response+times[:,None]*command+tau*(1.-np.exp(-times/tau))[:,None]*(velocity-command)
        last_velocity=command+(velocity-command)*math.exp(-hold/tau)
        fractions=np.linspace(0.,1.,max(3,int(np.linalg.norm(last_velocity)*tau/.15)+2))
        braking=driven[-1]+fractions[:,None]*last_velocity*tau
        path=np.concatenate(([position],driven,braking))
        # Acceleration can be faster than the measured stopping response,
        # especially on Z. Also cover reaching the target immediately.
        immediate=np.array([position,position+command*latency,
            position+command*(latency+max(self.settling,speed/(2.*self.braking)))])
        queries,_,_=swept_samples(path)
        other,_,_=swept_samples(immediate)
        queries=np.concatenate((queries,other))
        extent=float(np.max(np.linalg.norm(np.concatenate((path,immediate))-position,axis=1)))
        return queries,extent

    def filter_command(self, position, velocity, desired, points, cloud_stamp, pose_stamp):
        """Certify the final world-NED XYZ command, not just its reference.

        Include measured response, the commanded direction, and a full stop.
        A blocked horizontal path must not leave an unchecked Z controller on.
        """
        age,error=self._update(points,cloud_stamp,pose_stamp,position)
        if error:return np.zeros(3),dict(command_reason=error,command_scale=0.)
        position=np.asarray(position);velocity=np.asarray(velocity);desired=np.asarray(desired)
        if self.response_model is not None:
            brake_z=self.response_model.fallback_vertical(position,velocity)
            candidates=[desired,np.array([0.,0.,desired[2]]),np.zeros(3),
                        np.array([0.,0.,brake_z])]
            candidates.extend(np.array([0.,0.,z]) for z in (-4.,-2.,-1.,1.,2.,4.5))
            commands=[self.response_model.prepare(target,velocity,pose_stamp,position) for target in candidates]
            envelopes=self.response_model.envelopes(position,velocity,commands,self.reaction+age)
            clearances={};constraints=[]
            for i,(command,(samples,extent,times)) in enumerate(zip(commands,envelopes)):
                self.envelope_times=times
                constrained=bool(self.command_constraint is None or self.command_constraint(samples))
                constraints.append(constrained)
                # Rejected road/floor bounds cannot win or be certified.
                # Avoid expensive surface queries for those trajectories.
                if not constrained:continue
                clearance=self.command_clearance(samples)
                clearances[i]=clearance
                if clearance>=self.margin+.1 and extent<=self.horizon:
                    return command,dict(command_reason='COMMAND_BRAKING',command_scale=None,
                                        command_clearance=clearance)
            # No full-stop candidate passed. Clearance ranking is not a
            # certificate and must never select a rejected +/-4m/s height
            # probe by ranking clearance. Retain road-grade tracking while
            # braking XY, damp near-rest motion, and expose the failure.
            settling=bool(np.linalg.norm(velocity)<.8 and
                          self.index.distance([position])[0]<self.margin+.1)
            neutral=bool(np.linalg.norm(velocity)<.05 and
                         np.linalg.norm(commands[2][:2])<.05 and constraints[2])
            choice=2 if neutral or (not constraints[3] and constraints[2]) else 3
            # Already tighter than the base margin: per-return monotone
            # checks cannot certify any direction, but staying is not a
            # certificate either. Retreat along the local outward normal of
            # the nearest surface cluster at bounded speed. Uncertified and
            # explicitly reported; every ordinary candidate above still wins.
            near=float(self.index.distance([position])[0])
            if near<.6:
                relative=np.asarray(points,dtype=float)-position
                distance=np.linalg.norm(relative,axis=1)
                close=relative[distance<.5]
                if len(close)>=16:
                    unit=close/np.maximum(np.linalg.norm(close,axis=1),1e-6)[:,None]
                    outward=-np.sum(unit,axis=0)
                    norm=float(np.linalg.norm(outward))
                    if norm>1e-6:
                        retreat=self.response_model.prepare(.6*outward/norm,velocity,pose_stamp,position)
                        return retreat,dict(command_reason='EMERGENCY_RETREAT',command_scale=None,
                            command_clearance=near,emergency_clearance=near,
                            emergency_retreat_direction=(outward/norm).tolist())
            return commands[choice],dict(command_reason='COMMAND_BLOCKED',command_scale=None,
                command_clearance=clearances.get(choice),blocked_settle=settling,
                blocked_neutral_settle=neutral,blocked_brake=True,
                blocked_bounds_accepted=constraints[choice],
                blocked_grade_follow=bool(self.response_model.stop_profile is not None and np.linalg.norm(velocity[:2])>=.8))
        latency=self.reaction+age
        def check(command):
            queries,extent=self.command_envelope(position,velocity,command,age)
            clearance=float(np.min(self.index.distance(queries)))
            return clearance,clearance>=self.margin+.1 and extent<=self.horizon
        clearance,safe=check(desired)
        command=desired.copy()
        if safe:return command,dict(command_reason='COMMAND_CLEAR',command_scale=1.,command_clearance=clearance)
        # Preserve safe vertical correction while braking horizontal motion.
        base=np.array([0.,0.,desired[2]])
        clearance,safe=check(base)
        if safe:
            lo,hi=0.,1.;command=base.copy()
            for _ in range(5):
                mid=(lo+hi)/2.;candidate=np.array([*(desired[:2]*mid),desired[2]])
                margin,safe=check(candidate)
                if safe:lo=mid;command=candidate;clearance=margin
                else:hi=mid
            return command,dict(command_reason='COMMAND_BRAKING',command_scale=lo,command_clearance=clearance)
        # If coasting to zero is already unsafe, a verified opposite target
        # can arrest momentum sooner. It uses the same geometry check.
        norm=float(np.linalg.norm(velocity))
        counter=-velocity*min(1.,3./max(norm,1e-6))
        clearance,safe=check(counter)
        if safe and norm>.05:
            return counter,dict(command_reason='COUNTER_BRAKE',command_scale=None,command_clearance=clearance)
        clearance,safe=check(np.zeros(3))
        if not safe:return np.zeros(3),dict(command_reason='COMMAND_BLOCKED',command_scale=0.,command_clearance=clearance)
        lo,hi=0.,1.
        command=np.zeros(3)
        for _ in range(5):
            mid=(lo+hi)/2.
            candidate=desired*mid; margin,safe=check(candidate)
            if safe:lo=mid;command=candidate;clearance=margin
            else:hi=mid
        return command,dict(command_reason='COMMAND_BRAKING',command_scale=lo,command_clearance=clearance)

    def escape(self, position, velocity, target, points, cloud_stamp, pose_stamp):
        """Slow in-place shift after stopping, certified against EVERY return.

        If already inside a buffer, no affected surface may get closer. The
        target must restore the full buffer; this never authorizes entering it.
        """
        age,error=self._update(points,cloud_stamp,pose_stamp,position)
        if error or np.linalg.norm(np.asarray(velocity)[:2])>.8:return None
        delta=np.asarray(target)-position
        distance=float(np.linalg.norm(delta))
        if not .05<distance<5.:return None
        latency=self.reaction+age
        points=np.asarray(points);relative=points-position
        # Returns outside this ball cannot touch the shift or stopping prefix.
        radius=max(distance,np.linalg.norm(velocity)*latency+.6*(latency+max(self.settling,.6/(2.*self.braking))))+self.margin+.1
        relative=relative[np.sum(relative*relative,axis=1)<=radius*radius]
        initial2=np.sum(relative*relative,axis=1)
        buffer=self.margin+.1
        close=initial2<buffer*buffer
        fraction=np.clip(relative@delta/(distance*distance),0.,1.)
        separation=relative-fraction[:,None]*delta
        if np.any(np.sum(separation*separation,axis=1)<np.minimum(initial2,buffer*buffer)-1e-7):return None
        target_clearance=float(self.index.distance([target])[0])
        # Reserve tracking tolerance before committing the resulting offset.
        if target_clearance<buffer+.2:return None
        # Certify the actual response prefix too: waiting for momentum to die
        # out cannot be replaced with a nominal zero-velocity assumption.
        # The per-return separation check above already keeps every return at
        # or above min(its initial distance, buffer); no first-order veto.
        prefix=np.asarray(velocity)*latency
        speed=min(.6,distance)
        speed=min(speed,max(0.,distance-np.linalg.norm(prefix))/(latency+max(self.settling,.6/(2.*self.braking))))
        if speed<.01:return None
        command=speed*delta/distance
        end=prefix+command*(latency+max(self.settling,speed/(2.*self.braking)))
        path,_,_=swept_samples(np.array([position,position+prefix,position+end,target]))
        # Check both planes independently so approaching a close floor cannot
        # be hidden by increasing clearance to a still-closer ceiling.
        if self.index.roof is not None:
            origin,coeff,_,_=self.index.roof
            height=path[:,2]-origin[2]-(path[:,:2]-origin[:2])@coeff[:2]-coeff[2]
            norm=math.sqrt(1.+np.sum(coeff[:2]**2))
            measured=np.isfinite(self.index.surface_distance(path))
            for distances in (height/norm,(5.-height)/norm):
                minimum=min(distances[0],buffer) if measured[0] else buffer
                if np.any(distances[measured]<minimum-1e-7):return None
        # Treat every face independently: moving away from one car cannot
        # authorize approaching another face that is already inside a buffer.
        for face in self.faces:
            index=PointIndex([],faces=[face])
            initial=index.face_distance([position],signed=True)[0]
            if np.min(index.face_distance(path,signed=True))<min(initial,buffer)-1e-7:return None
        for start,finish in ((np.zeros(3),prefix),(prefix,end)):
            step=finish-start
            t=np.clip(np.sum((relative-start)*step,axis=1)/max(1e-12,float(step@step)),0.,1.)
            squared=np.sum((relative-start-t[:,None]*step)**2,axis=1)
            if np.any(squared<np.minimum(initial2,buffer*buffer)-1e-7):return None
        if self.response_model is not None:
            from lidar_navigation import LidarNavigator
            model=self.response_model;previous_profile=model.stop_profile
            model.stop_profile=LidarNavigator._stop_profile(np.array([position,target]),gain=model.height_gain)
            prepared=model.prepare(command,velocity,pose_stamp,position)
            if not self.recovery_command_ok(position,velocity,prepared,points,age):
                model.stop_profile=previous_profile
                return None
            return prepared
        return command

    def recovery_command_ok(self,position,velocity,command,points,age):
        if np.linalg.norm(np.asarray(velocity)[:2])>.8:return False
        samples,extent=self.command_envelope(position,velocity,command,age)
        if extent>self.horizon or (self.command_constraint is not None and not self.command_constraint(samples)):return False
        buffer=self.margin+.1;position=np.asarray(position);points=np.asarray(points)
        initial=self.index.distance([position])[0]
        if self.command_clearance(samples)<min(initial,buffer)-1e-7:return False
        # Every raw return initially closer than the full buffer must keep
        # its own distance, so a nearer wall cannot hide a worsening floor.
        close=points[np.linalg.norm(points-position,axis=1)<extent+buffer]
        if len(close):
            threshold=np.minimum(np.sum((close-position)**2,axis=1),buffer**2)
            for i in range(0,len(samples),64):
                squared=np.sum((samples[i:i+64,None,:]-close[None,:,:])**2,axis=2)
                if np.any(squared<threshold[None,:]-1e-7):return False
        if self.index.roof is not None:
            origin,coeff,_,_=self.index.roof
            height=samples[:,2]-origin[2]-(samples[:,:2]-origin[:2])@coeff[:2]-coeff[2]
            norm=math.sqrt(1.+np.sum(coeff[:2]**2));measured=np.isfinite(self.index.surface_distance(samples))
            for distances in (height/norm,(5.-height)/norm):
                minimum=min(distances[0],buffer) if measured[0] else buffer
                if np.any(distances[measured]<minimum-1e-7):return False
        if self.index.patches is not None:
            centers,normals,bases,boundaries,offsets,_=self.index.patches
            ids=np.flatnonzero(np.linalg.norm(centers-position,axis=1)<extent+buffer+1.5)
            for begin in range(0,len(ids),32):
                chosen=ids[begin:begin+32]
                delta=samples[:,None,:]-centers[chosen][None,:,:]
                local=np.einsum('nki,kij->nkj',delta,bases[chosen])
                inside=np.all(np.einsum('nki,kji->nkj',local,boundaries[chosen])>=offsets[chosen][None,:,:]-1e-7,axis=2)
                distances=abs(np.sum(delta*normals[chosen][None,:,:],axis=2));distances[~inside]=np.inf
                if np.any(distances<np.minimum(distances[0],buffer)[None,:]-1e-7):return False
        for face in self.faces:
            index=PointIndex([],faces=[face]);minimum=min(index.face_distance([position],signed=True)[0],buffer)
            if np.min(index.face_distance(samples,signed=True))<minimum-1e-7:return False
        times=self.envelope_times
        # Coast/driven branches can settle after different durations. Each
        # scenario restarts its clock, so retain the tail of every branch.
        starts=np.r_[0,np.flatnonzero(np.diff(times)<0.)+1,len(times)]
        tail_mask=np.zeros(len(times),dtype=bool)
        for first,last in zip(starts[:-1],starts[1:]):
            tail_mask[first:last]=times[first:last]>=times[first:last].max()-.15
        tail=samples[tail_mask]
        # A recovery must make measurable progress in every modeled tail.
        # It is not a normal full-buffer certificate while still inside.
        self.envelope_times=times[tail_mask]
        progress=self.command_clearance(tail)>=min(buffer,initial+.01)-1e-7
        self.envelope_times=times
        return progress

    def find_shift(self, position, velocity, base, side, points, cloud_stamp, pose_stamp, forward_path=None, max_z=None):
        """Find an in-place escape without requiring 12 m of forward travel.

        A nearby second car can block the long-route fallback even when a
        safe initial shift exists. Certify that independent action first.
        """
        age,error=self._update(points,cloud_stamp,pose_stamp,position)
        if error or np.linalg.norm(velocity)>.8:return None
        choices=np.array([(y,z) for y in np.arange(-3.,3.01,.25)
                          for z in np.arange(-1.5,1.51,.25)])
        targets=np.asarray(base)+choices[:,0,None]*np.asarray(side)
        targets[:,2]+=choices[:,1]
        if max_z is not None:targets[:,2]=np.minimum(targets[:,2],max_z-.01)
        delta=targets-position
        eligible=self.index.distance(targets)>=self.margin+.3
        if forward_path is not None:
            paths=np.asarray(forward_path)[None,:,:]+(targets-np.asarray(base))[:,None,:]
            eligible&=np.min(self.index.distance(paths.reshape(-1,3)).reshape(len(targets),-1),axis=1)>=self.margin+.1
        cost=np.sum(delta*delta,axis=1)
        for i in np.argsort(cost):
            if not eligible[i] or cost[i]<.01:continue
            command=self.escape(position,velocity,targets[i],points,cloud_stamp,pose_stamp)
            if command is not None:return targets[i],command
        # Two adjacent cars can prevent any side shift at this station. A
        # separately certified short retreat can restore room to turn; it
        # does not need to authorize the still-blocked forward corridor.
        if forward_path is not None:
            backward=np.asarray(forward_path)[0]-np.asarray(forward_path)[-1]
            backward[2]=0.
            norm=np.linalg.norm(backward)
            if norm>.1:
                for distance in (.5,1.,1.5,2.):
                    target=np.asarray(position)+distance*backward/norm
                    if max_z is not None and target[2]>max_z:continue
                    command=self.escape(position,velocity,target,points,cloud_stamp,pose_stamp)
                    if command is not None:return target,command
        return None

    def evaluate(self, s, position, velocity, xy, center, plan, points,
                 cloud_stamp, pose_stamp):
        started=time.monotonic()
        age,error=self._update(points,cloud_stamp,pose_stamp,position)
        if error:
            return dict(cap=0., execution_verified=False,
                        guard_reason=error, verified_stamp=None)
        stations=s+np.arange(0., self.horizon+.1, .2)
        path=PredictiveAvoidance.positions(stations,xy,center,plan)
        # Certify the actual position-to-reference connector as well. Starting
        # at the nominal point would hide an obstacle beside a tracking error.
        path[0]=position
        queries,segments,fraction=swept_samples(path)
        progress=stations[segments]-s+.2*fraction
        distances=self.index.distance(queries)
        blocked=np.flatnonzero(distances<self.margin+.1)
        free=self.horizon if not len(blocked) else max(0.,progress[blocked[0]]-.2)
        if free<1e-6:free=0.
        # An actuator cannot remove the measured velocity instantaneously.
        # Validate the response segment before following the spatial path.
        latency=self.reaction+age
        inertia=position+np.linspace(0.,latency,8)[:,None]*np.asarray(velocity)
        response_clear=float(np.min(self.index.distance(inertia)))
        if response_clear<self.margin+.1:free=0.
        cap=self._cap(free,latency)
        motion=self.motion_envelope(position,velocity,latency)
        # Route speed is horizontal; the stopping calculation uses XYZ speed.
        speed=float(np.linalg.norm(velocity))
        ratio=float(np.linalg.norm(np.asarray(velocity)[:2]))/speed if speed>.05 else 1.
        motion_cap=motion['cap']*ratio if math.isfinite(motion['cap']) else float('inf')
        motion_limited=motion_cap<cap
        # The final XYZ filter certifies the measured response, steering and
        # full stop together. A separate straight coasting ray may be unsafe
        # even when that steering command is safe; do not impose its cap on a
        # different trajectory and trigger repeated stop/shift cycles.
        return dict(cap=cap, execution_verified=bool(free>0. and cap>1e-6),
                    guard_reason=('EXECUTION_BLOCKED' if free<=0. or cap<=1e-6 else
                                  'BRAKING_MOTION' if motion_limited else
                                  'BRAKING_OBSTACLE' if len(blocked) else 'EXECUTION_CLEAR'),
                    verified_stamp=pose_stamp, verified_cloud_stamp=cloud_stamp,
                    verified_distance=float(free),
                    execution_clearance=float(np.min(distances)) if np.all(np.isfinite(distances)) else None,
                    response_clearance=response_clear if math.isfinite(response_clear) else None,
                    motion_clearance=motion['clearance'],motion_free_distance=motion['distance'],
                    motion_speed_cap=motion_cap if math.isfinite(motion_cap) else None,
                    measured_stopping_distance=motion['stop'],
                    validation_ms=1000.*(time.monotonic()-started))
def certify_spacetime_command(snapshot,parameters,primitive,profile,config,hold,deadline=None):
    """Pure final-command certificate; uses the planner's collision definition.

    Certifies one actual compensated XYZ publication held until next tick, and
    its full feedback stop. Known drive AND coast uncertainty during reaction
    delay are checked separately. Does not mutate a live response model.
    """
    from dataclasses import replace
    import time
    from response_rollout import _commands,rollout_delay,rollout_stop
    from trajectory_types import join_traces,readonly
    from trajectory_collision import TrajectoryCollision
    from trajectory_executor import ExecutionDecision
    if not np.isfinite(hold) or hold<=0:raise ValueError('Invalid command hold')
    checker=TrajectoryCollision(snapshot,config)
    fresh=checker.freshness()
    def reject(reason,conflict=None):
        return ExecutionDecision(np.zeros(3),'FAILSAFE',reason,False,conflict=conflict)
    if not fresh.safe:return reject(fresh.reason,fresh)
    state=snapshot.response_state
    if state.model_key!=parameters.model_key:return reject('MODEL_MISMATCH')
    _,commands=_commands(parameters,state.previous,state.last_control,primitive.target,
                         state.velocity,state.position,state.stamp,profile,primitive.name=='WAIT')
    command=commands[len(commands)//2].copy()
    if np.linalg.norm(command[:2])>config.max_horizontal_speed+1e-8 or abs(command[2])>config.max_vertical_speed+1e-8:
        return reject('COMMAND_LIMIT')
    complete=None;backup_result=None
    # Account for old-drive uncertainty without duplicating future clock age.
    assumptions=(True,False) if snapshot.reaction_delay>0. and state.has_applied else (state.has_applied,)
    for driven in assumptions:
        if deadline is not None and time.monotonic()>=deadline:return reject('GUARD_TIMEOUT')
        initial=state;parts=[]
        if snapshot.reaction_delay>0.:
            delay=rollout_delay(parameters,replace(state,has_applied=driven),snapshot.reaction_delay)
            parts.append(delay);initial=delay.end_state
        applied=np.broadcast_to(command,initial.applied.shape).copy()
        previous=applied.copy()
        previous[:,2]-=parameters.lift_gain*np.sum((applied[:,:2]-initial.velocity[:,:2])**2,axis=1)
        issued=replace(initial,previous=previous,applied=applied,has_applied=True,
                       effective=applied,next_physics=initial.stamp,
                       last_control=np.full(len(applied),initial.stamp),
                       next_control=initial.stamp+np.asarray(parameters.periods))
        held=rollout_delay(parameters,issued,hold);parts.append(held)
        # Backup begins at next actual publication. Its controller is due now.
        stop_state=replace(held.end_state,next_control=np.full(len(applied),held.end_state.stamp))
        backup,stopped=rollout_stop(parameters,stop_state,profile,deadline=deadline)
        if not stopped:return reject('STOP_UNVERIFIED')
        trace=join_traces(parts+[backup]);checked=checker.check(trace)
        if not checked.safe:return reject(checked.reason,checked)
        if deadline is not None and time.monotonic()>=deadline:return reject('GUARD_TIMEOUT')
        complete=trace;backup_result=backup
    return ExecutionDecision(command,'NORMAL','PASS',True,trace=complete,backup=backup_result)
