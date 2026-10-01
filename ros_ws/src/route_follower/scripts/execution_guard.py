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

    def set_faces(self,faces):
        self.faces=faces
        if self.index is not None:self.index.faces=faces

    def _update(self, points, cloud_stamp, pose_stamp, position=None):
        age=None if cloud_stamp is None else pose_stamp-cloud_stamp
        if points is None or age is None or not 0.<=age<.5:
            return None, 'LIDAR_STALE'
        if len(points)==0:return None, 'LIDAR_EMPTY'
        if self.index is None or self.cloud_stamp!=cloud_stamp:
            self.index=PointIndex(points,origin=position,faces=self.faces,road_height=5.)
            self.cloud_stamp=cloud_stamp
        return age, None

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

    def filter_command(self, position, velocity, desired, points, cloud_stamp, pose_stamp):
        """Certify the final world-NED XYZ command, not just its reference.

        Include measured response, the commanded direction, and a full stop.
        A blocked horizontal path must not leave an unchecked Z controller on.
        """
        age,error=self._update(points,cloud_stamp,pose_stamp,position)
        if error:return np.zeros(3),dict(command_reason=error,command_scale=0.)
        position=np.asarray(position);velocity=np.asarray(velocity);desired=np.asarray(desired)
        latency=self.reaction+age
        # Model the actual velocity after a new command, then a full stop.
        # A zero target still travels velocity*tau after the reaction prefix.
        # Checking only the zero command's ray hid that entire displacement.
        hold=self.reaction
        def check(command):
            speed=float(np.linalg.norm(command))
            tau=max(self.settling,float(np.linalg.norm(velocity))/(2.*self.braking),speed/(2.*self.braking),.01)
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
            clearance=float(np.min(self.index.distance(queries)))
            extent=float(np.max(np.linalg.norm(np.concatenate((path,immediate))-position,axis=1)))
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
        if error or np.linalg.norm(velocity)>.8:return None
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
        if np.any(relative[close]@delta>1e-7):return None
        fraction=np.clip(relative@delta/(distance*distance),0.,1.)
        separation=relative-fraction[:,None]*delta
        if np.any(np.sum(separation*separation,axis=1)<np.minimum(initial2,buffer*buffer)-1e-7):return None
        target_clearance=float(self.index.distance([target])[0])
        # Reserve tracking tolerance before committing the resulting offset.
        if target_clearance<buffer+.2:return None
        # Certify the actual response prefix too: waiting for momentum to die
        # out cannot be replaced with a nominal zero-velocity assumption.
        prefix=np.asarray(velocity)*latency
        if np.any(relative[close]@prefix>1e-7):return None
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
        return command

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
