#!/usr/bin/env python3
"""Calibrated command slew, horizontal/Z coupling and timed stopping policy.

Gain comes from motion_probe_20261002_01--03. Scenario ranges retain response
uncertainty; this is an empirical model, not a formal flight guarantee.
"""
import numpy as np
from path_sampling import swept_samples
from response_native import NativeResponse


class VelocityResponse:
    def __init__(self, acceleration=4., deceleration=8., lift_gain=.086,brake_feedback=.6,native=True,vertical_command_max=4.5,coupling_gains=None,height_gain=1.,coupling_limited=False,xy_error_max=np.inf,control_periods=None):
        self.acceleration=acceleration
        self.deceleration=deceleration
        self.lift_gain=lift_gain
        self.brake_feedback=brake_feedback
        self.vertical_command_max=vertical_command_max
        self.height_gain=height_gain
        self.coupling_limited=coupling_limited
        self.xy_error_max=float(xy_error_max)
        periods=None if control_periods is None else tuple(map(float,control_periods))
        if periods is not None and (not periods or any(not np.isfinite(p) or p<.08 for p in periods)):
            raise ValueError("Feedback periods must be finite and at least the integration step0.08s")
        self.control_periods=periods
        self.previous=np.zeros(3)
        self.applied_command=None
        self.last_stamp=None
        self.last_source_stamp=None
        self.stop_profile=None
        gains=(.075,.086,.10) if coupling_gains is None else coupling_gains
        self.scenarios=tuple((xy,z,gain) for (xy,z),gain in zip(((.8,.10),(.95,.16),(1.1,.22)),gains))
        if periods is not None:
            self.control_periods=tuple(period for scenario in self.scenarios for period in periods)
            self.scenarios=tuple(scenario for scenario in self.scenarios for period in periods)
        self.native=NativeResponse() if native else None

    def reset(self):
        self.previous=np.zeros(3);self.last_stamp=None;self.last_source_stamp=None;self.applied_command=None

    def freeze_parameters(self):
        from response_rollout import ResponseParameters
        return ResponseParameters.from_model(self)

    def snapshot_state(self,position,velocity,stamp,parameters=None):
        from response_rollout import snapshot_state
        return snapshot_state(self,position,velocity,stamp,parameters)

    def rollout_primitive(self,state,target,duration,profile=None,parameters=None):
        from response_rollout import rollout_primitive
        from trajectory_types import Primitive
        parameters=self.freeze_parameters() if parameters is None else parameters
        return rollout_primitive(parameters,state,Primitive(target,duration),profile)

    def rollout_sequence(self,state,primitives,profiles=None,parameters=None):
        from response_rollout import rollout_sequence
        parameters=self.freeze_parameters() if parameters is None else parameters
        return rollout_sequence(parameters,state,primitives,profiles)

    @staticmethod
    def slew(previous,desired,limit):
        delta=desired-previous;length=np.linalg.norm(delta)
        return previous+delta*min(1.,limit/max(1e-9,length))

    def prepare(self,desired,velocity,stamp,position=None):
        desired=np.asarray(desired,dtype=float).copy()
        braking=np.linalg.norm(desired[:2])<.05
        if braking:
            desired[:2]=self.braking_target(np.asarray(velocity)[:2],position)
        # Use the observed control interval through the certified command
        # hold horizon. Capping every delayed cycle at .15 s silently slowed
        # real braking while envelopes continued to integrate 8 m/s².
        dt=.05 if self.last_stamp is None else np.clip(stamp-self.last_stamp,.001,.35)
        slowing=np.dot(desired[:2]-self.previous[:2],self.previous[:2])<0.
        rate=self.deceleration if braking or slowing else self.acceleration
        desired[:2]=self.slew(self.previous[:2],desired[:2],rate*dt)
        return self.compensate(desired,velocity)

    def compensate(self,command,velocity):
        """Retain vertical authority when XY error would saturate lift correction.

        Cap the XY error relative to measured motion, after command slew.
        This can lengthen braking; the complete forecast uses the same cap.
        """
        command=np.asarray(command,dtype=float).copy()
        nominal=np.clip(command[...,2],-4.,4.5)
        if self.coupling_limited and self.lift_gain>0.:
            budget=np.sqrt(np.maximum(0.,self.vertical_command_max-nominal)/self.lift_gain)
            budget=np.minimum(budget,self.xy_error_max)
            delta=command[...,:2]-np.asarray(velocity)[...,:2]
            factor=np.minimum(1.,budget/np.maximum(1e-9,np.linalg.norm(delta,axis=-1)))
            command[...,:2]=np.asarray(velocity)[...,:2]+delta*factor[...,None]
        correction=self.lift_gain*np.sum((command[...,:2]-np.asarray(velocity)[...,:2])**2,axis=-1)
        command[...,2]=np.clip(nominal+correction,-4.,self.vertical_command_max)
        return command

    def braking_target(self,velocity,position=None):
        """Counter forward inertia while steering toward the verified path.

        The perpendicular correction uses a prospective position and fades
        with speed. It cannot keep a stationary aircraft moving along a path.
        Its separate limit preserves the longitudinal counter-brake budget.
        """
        velocity=np.asarray(velocity)
        target=-self.brake_feedback*velocity
        norm=np.linalg.norm(target,axis=-1,keepdims=True)
        target*=np.minimum(1.,3./np.maximum(1e-9,norm))
        path=None if self.stop_profile is None else getattr(self.stop_profile,'path',None)
        if path is not None and position is not None and len(path)>1:
            shape=velocity.shape;v=velocity.reshape(-1,2);p=np.asarray(position).reshape(-1,3)
            segments=np.diff(path[:,:2],axis=0);length=np.sum(segments**2,axis=1)
            guess=p[:,:2]+.8*v
            delta=guess[:,None,:]-path[None,:-1,:2]
            fraction=np.clip(np.sum(delta*segments,axis=2)/np.maximum(1e-8,length),0.,1.)
            residual=delta-fraction[:,:,None]*segments
            distance=np.sum(residual**2,axis=2);distance[:,length<1e-8]=np.inf
            if np.any(length>1e-8):
                nearest=np.argmin(distance,axis=1);error=residual[np.arange(len(p)),nearest]
                speed=np.linalg.norm(v,axis=1);direction=v/np.maximum(1e-8,speed[:,None])
                error-=np.sum(error*direction,axis=1)[:,None]*direction
                correction=-3.*error
                correction*=np.minimum(1.,8./np.maximum(1e-8,np.linalg.norm(correction,axis=1)))[:,None]
                correction*=np.minimum(1.,speed)[:,None]
                target+=correction.reshape(shape)
        return target

    def commit(self,command,velocity,stamp,control_stamp=None):
        # Keep the actual compensated command separately from the nominal
        # slew state. It is still driving the aircraft during response delay.
        self.applied_command=np.asarray(command,dtype=float).copy()
        self.previous=np.asarray(command,dtype=float).copy()
        self.previous[2]-=self.lift_gain*np.sum((command[:2]-velocity[:2])**2)
        # Fresh publication recertification can advance the source pose to
        # the end of computation. Slew intervals use consecutive planning
        # samples, preserving the real cycle interval when callbacks queue.
        self.last_source_stamp=stamp
        self.last_stamp=stamp if control_stamp is None else control_stamp

    def envelope(self,position,velocity,command,latency,hold=.35):
        """Hold the selected command, then slew XY to zero with Z compensation.

        Every sampled point carries elapsed time for moving-obstacle checks.
        The stopping controller must use the same prepare() policy in flight.
        """
        return self.envelopes(position,velocity,[command],latency,hold)[0]

    def envelopes(self,position,velocity,commands,latency,hold=.35):
        coasting=self._envelopes(position,velocity,commands,latency,hold,None)
        if self.applied_command is None or latency<=0.:return coasting
        driven=self._envelopes(position,velocity,commands,latency,hold,self.applied_command)
        return [(np.concatenate((a[0],b[0])),max(a[1],b[1]),np.concatenate((a[2],b[2])))
                for a,b in zip(coasting,driven)]

    def _envelopes(self,position,velocity,commands,latency,hold,applied):
        """Integrate all candidate/scenario combinations in one vector batch."""
        position=np.asarray(position,dtype=float);velocity=np.asarray(velocity,dtype=float)
        commands=np.asarray(commands,dtype=float)
        step=.08
        duration=hold+np.max(np.linalg.norm(commands[:,:2],axis=1))/self.deceleration+8.
        profile=None if self.stop_profile is None else getattr(self.stop_profile,'path',None)
        if self.native is not None and self.native.library is not None and (self.stop_profile is None or profile is not None):
            path,timing=self.native.integrate(position,velocity,commands,self.scenarios,profile,latency,hold,
                self.deceleration,self.brake_feedback,self.lift_gain,self.vertical_command_max,duration,applied,
                height_gain=getattr(self.stop_profile,'height_gain',1.),height_path=getattr(self.stop_profile,'height_path',None),coupling_limited=self.coupling_limited,xy_error_max=self.xy_error_max,control_periods=self.control_periods)
            return self.native.samples(path,timing,position)
        count=len(commands);scenario_count=len(self.scenarios)
        u=np.broadcast_to(commands[:,None,:],(count,scenario_count,3)).copy()
        v=np.broadcast_to(velocity,u.shape).copy()
        p=np.broadcast_to(position,u.shape).copy()
        initial=np.broadcast_to(position,u.shape).copy()
        path=[initial];times=[0.]
        scenarios=np.asarray(self.scenarios)
        taus=np.column_stack([scenarios[:,0],scenarios[:,0],scenarios[:,1]])[None,:,:]
        gains=scenarios[None,:,2]
        if applied is None:
            p+=latency*velocity;path.append(p.copy());times.append(latency)
        else:
            delay_elapsed=0.
            while delay_elapsed<latency-1e-12:
                interval=min(step,latency-delay_elapsed)
                delay_decay=np.exp(-interval/taus)
                effective=np.broadcast_to(applied,u.shape).copy()
                effective[:,:,2]-=gains*np.sum((effective[:,:,:2]-v[:,:,:2])**2,axis=2)
                p+=effective*interval+(v-effective)*taus*(1.-delay_decay)
                v=effective+(v-effective)*delay_decay
                delay_elapsed+=interval;path.append(p.copy());times.append(delay_elapsed)
        decay=np.exp(-step/taus)
        elapsed=0.
        periods=(np.zeros(scenario_count) if self.control_periods is None else np.asarray(self.control_periods))
        next_control=np.full(scenario_count,hold)
        last_control=np.zeros(scenario_count)
        while elapsed<duration:
            if elapsed>=hold:
                update=(periods==0.)|(elapsed+1e-9>=next_control)
                control_dt=np.where(periods==0.,step,np.minimum(.35,elapsed-last_control))
                target=self.braking_target(v[:,:,:2],p)
                delta=target-u[:,:,:2]
                length=np.linalg.norm(delta,axis=2)
                fraction=np.minimum(1.,self.deceleration*control_dt[None,:]/np.maximum(1e-9,length))
                candidate=u.copy()
                candidate[:,:,:2]+=delta*fraction[:,:,None]
                vertical=(np.zeros(u.shape[:2]) if self.stop_profile is None else
                          self.stop_profile(p.reshape(-1,3),v.reshape(-1,3)).reshape(u.shape[:2]))
                candidate[:,:,2]=vertical
                candidate=self.compensate(candidate,v)
                u[:,update,:]=candidate[:,update,:]
                last_control[update]=elapsed
                next_control[update]=elapsed+periods[update]
            effective=u.copy()
            effective[:,:,2]-=gains*np.sum((u[:,:,:2]-v[:,:,:2])**2,axis=2)
            p+=effective*step+(v-effective)*taus*(1.-decay)
            v=effective+(v-effective)*decay
            elapsed+=step;path.append(p.copy());times.append(latency+elapsed)
            if elapsed>hold+1. and np.max(np.linalg.norm(u[:,:,:2],axis=2))<.001 and np.max(np.linalg.norm(v,axis=2))<.01:break
        return self._sample_envelopes(np.array(path),np.array(times),position)

    @staticmethod
    def _sample_envelopes(path,timing,position):
        results=[]
        for candidate in range(path.shape[1]):
            queries=[];query_times=[]
            for scenario in range(path.shape[2]):
                samples,segments,fractions=swept_samples(path[:,candidate,scenario,:],spacing=.18)
                queries.append(samples)
                query_times.append(timing[segments]+fractions*(timing[segments+1]-timing[segments]))
            queries=np.concatenate(queries);query_times=np.concatenate(query_times)
            results.append((queries,float(np.max(np.linalg.norm(queries-position,axis=1))),query_times))
        return results

    def forecast(self,position,velocity,command,latency,duration=.8):
        position=np.asarray(position,dtype=float).copy();velocity=np.asarray(velocity,dtype=float).copy()
        tau=np.array([.95,.95,.16]);effective=np.asarray(command).copy()
        if self.applied_command is None:position+=velocity*latency
        else:
            elapsed=0.
            while elapsed<latency-1e-12:
                interval=min(.08,latency-elapsed);decay=np.exp(-interval/tau)
                old=self.applied_command.copy();old[2]-=self.lift_gain*np.sum((old[:2]-velocity[:2])**2)
                position+=old*interval+(velocity-old)*tau*(1.-decay)
                velocity=old+(velocity-old)*decay;elapsed+=interval
        effective[2]-=self.lift_gain*np.sum((command[:2]-velocity[:2])**2)
        return position+effective*duration+tau*(1.-np.exp(-duration/tau))*(velocity-effective)
