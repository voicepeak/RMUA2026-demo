#!/usr/bin/env python3
"""Calibrated command slew, horizontal/Z coupling and timed stopping policy.

Gain comes from motion_probe_20261002_01--03. Scenario ranges retain response
uncertainty; this is an empirical model, not a formal flight guarantee.
"""
import numpy as np
from path_sampling import swept_samples
from response_native import NativeResponse


class VelocityResponse:
    def __init__(self, acceleration=4., deceleration=8., lift_gain=.086,brake_feedback=.6,native=True,vertical_command_max=12.):
        self.acceleration=acceleration
        self.deceleration=deceleration
        self.lift_gain=lift_gain
        self.brake_feedback=brake_feedback
        self.vertical_command_max=vertical_command_max
        self.previous=np.zeros(3)
        self.last_stamp=None
        self.stop_profile=None
        self.scenarios=((.8,.10,.075),(.95,.16,.086),(1.1,.22,.10))
        self.native=NativeResponse() if native else None

    def reset(self):
        self.previous=np.zeros(3);self.last_stamp=None

    @staticmethod
    def slew(previous,desired,limit):
        delta=desired-previous;length=np.linalg.norm(delta)
        return previous+delta*min(1.,limit/max(1e-9,length))

    def prepare(self,desired,velocity,stamp):
        desired=np.asarray(desired,dtype=float).copy()
        braking=np.linalg.norm(desired[:2])<.05
        if braking:
            desired[:2]=self.braking_target(np.asarray(velocity)[:2])
        dt=.05 if self.last_stamp is None else np.clip(stamp-self.last_stamp,.001,.15)
        slowing=np.dot(desired[:2]-self.previous[:2],self.previous[:2])<0.
        rate=self.deceleration if braking or slowing else self.acceleration
        desired[:2]=self.slew(self.previous[:2],desired[:2],rate*dt)
        correction=self.lift_gain*np.sum((desired[:2]-velocity[:2])**2)
        desired[2]=np.clip(np.clip(desired[2],-4.,4.5)+correction,-4.,self.vertical_command_max)
        return desired

    def braking_target(self,velocity):
        velocity=np.asarray(velocity)
        target=-self.brake_feedback*velocity
        length=np.linalg.norm(target,axis=-1,keepdims=True)
        return target*np.minimum(1.,3./np.maximum(1e-9,length))

    def commit(self,command,velocity,stamp):
        self.previous=np.asarray(command,dtype=float).copy()
        self.previous[2]-=self.lift_gain*np.sum((command[:2]-velocity[:2])**2)
        self.last_stamp=stamp

    def envelope(self,position,velocity,command,latency,hold=.35):
        """Hold the selected command, then slew XY to zero with Z compensation.

        Every sampled point carries elapsed time for moving-obstacle checks.
        The stopping controller must use the same prepare() policy in flight.
        """
        return self.envelopes(position,velocity,[command],latency,hold)[0]

    def envelopes(self,position,velocity,commands,latency,hold=.35):
        """Integrate all candidate/scenario combinations in one vector batch."""
        position=np.asarray(position,dtype=float);velocity=np.asarray(velocity,dtype=float)
        commands=np.asarray(commands,dtype=float)
        step=.08
        duration=hold+np.max(np.linalg.norm(commands[:,:2],axis=1))/self.deceleration+5.
        profile=None if self.stop_profile is None else getattr(self.stop_profile,'path',None)
        if self.native is not None and self.native.library is not None and (self.stop_profile is None or profile is not None):
            path,timing=self.native.integrate(position,velocity,commands,self.scenarios,profile,latency,hold,
                self.deceleration,self.brake_feedback,self.lift_gain,self.vertical_command_max,duration)
            return self._sample_envelopes(path,timing,position)
        count=len(commands);scenario_count=len(self.scenarios)
        u=np.broadcast_to(commands[:,None,:],(count,scenario_count,3)).copy()
        v=np.broadcast_to(velocity,u.shape).copy()
        p=np.broadcast_to(position+latency*velocity,u.shape).copy()
        initial=np.broadcast_to(position,u.shape).copy()
        path=[initial,p.copy()];times=[0.,latency]
        scenarios=np.asarray(self.scenarios)
        taus=np.column_stack([scenarios[:,0],scenarios[:,0],scenarios[:,1]])[None,:,:]
        gains=scenarios[None,:,2]
        decay=np.exp(-step/taus)
        elapsed=0.
        while elapsed<duration:
            if elapsed>=hold:
                target=self.braking_target(v[:,:,:2])
                delta=target-u[:,:,:2]
                length=np.linalg.norm(delta,axis=2)
                fraction=np.minimum(1.,self.deceleration*step/np.maximum(1e-9,length))
                u[:,:,:2]+=delta*fraction[:,:,None]
                vertical=(np.zeros(u.shape[:2]) if self.stop_profile is None else
                          self.stop_profile(p.reshape(-1,3),v.reshape(-1,3)).reshape(u.shape[:2]))
                u[:,:,2]=np.clip(vertical+self.lift_gain*np.sum((u[:,:,:2]-v[:,:,:2])**2,axis=2),-4.,self.vertical_command_max)
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
        tau=np.array([.95,.95,.16]);effective=np.asarray(command).copy()
        effective[2]-=self.lift_gain*np.sum((command[:2]-velocity[:2])**2)
        return position+velocity*latency+effective*duration+tau*(1.-np.exp(-duration/tau))*(velocity-effective)
