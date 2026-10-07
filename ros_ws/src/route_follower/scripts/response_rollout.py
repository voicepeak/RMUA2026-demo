#!/usr/bin/env python3
"""Pure, scenario-preserving rollouts using the existing VelocityResponse law.

All vectors are world NED, including command Z. Reaction delay is integrated
once before search; first-order actuator lag applies throughout every segment.
Feedback schedules and command memory persist across primitive boundaries.
The legacy native envelope is unchanged; this sequence API is Python reference.
"""
from dataclasses import dataclass,asdict
import hashlib
import json
from types import SimpleNamespace
import numpy as np
from trajectory_types import ResponseState,ResponseTrace,Primitive,join_traces


@dataclass(frozen=True)
class ResponseParameters:
    scenarios: tuple = ((.8,.1,.075),(.95,.16,.086),(1.1,.22,.1))
    periods: tuple = (.08,.08,.08)
    acceleration: float = 4.
    deceleration: float = 8.
    lift_gain: float = .086
    brake_feedback: float = .6
    vertical_command_max: float = 4.5
    coupling_limited: bool = False
    xy_error_max: float = float('inf')
    integration_step: float = .08

    def __post_init__(self):
        object.__setattr__(self,'scenarios',tuple(tuple(map(float,s)) for s in self.scenarios))
        object.__setattr__(self,'periods',tuple(map(float,self.periods)))
        scenarios=np.asarray(self.scenarios)
        if (scenarios.ndim!=2 or scenarios.shape[1]!=3 or not len(scenarios) or
                not np.all(np.isfinite(scenarios)) or np.any(scenarios[:,:2]<=0) or
                np.any(scenarios[:,2]<0) or len(self.periods)!=len(scenarios) or
                any(not np.isfinite(p) or p<.001 for p in self.periods)):
            raise ValueError('Invalid rollout scenarios/feedback periods')
        for key in ('acceleration','deceleration','vertical_command_max','integration_step'):
            if not np.isfinite(getattr(self,key)) or getattr(self,key)<=0:raise ValueError('Invalid '+key)
        for key in ('lift_gain','brake_feedback'):
            if not np.isfinite(getattr(self,key)) or getattr(self,key)<0:raise ValueError('Invalid '+key)
        if np.isnan(self.xy_error_max) or self.xy_error_max<0 or self.integration_step>min(self.periods)+1e-9:
            raise ValueError('Invalid rollout integration/error bound')

    @property
    def model_key(self):
        # JSON Infinity is a deterministic local parameter token, not a log value.
        return hashlib.sha256(json.dumps(dict(law='sequence-v2-persistent-physics',parameters=asdict(self)),sort_keys=True).encode()).hexdigest()

    @classmethod
    def from_model(cls,model):
        return cls(scenarios=model.scenarios,periods=(tuple(model.control_periods) if model.control_periods is not None
                   else (.08,)*len(model.scenarios)),acceleration=model.acceleration,deceleration=model.deceleration,
                   lift_gain=model.lift_gain,brake_feedback=model.brake_feedback,
                   vertical_command_max=model.vertical_command_max,coupling_limited=model.coupling_limited,
                   xy_error_max=model.xy_error_max)


def snapshot_state(model,position,velocity,stamp,parameters=None):
    parameters=ResponseParameters.from_model(model) if parameters is None else parameters
    position=np.asarray(position,dtype=float);velocity=np.asarray(velocity,dtype=float)
    if position.shape!=(3,) or velocity.shape!=(3,):raise ValueError('Initial response vectors must be XYZ')
    shape=(len(parameters.scenarios),3)
    duplicate=lambda a:np.broadcast_to(a,shape).copy()
    previous=duplicate(model.previous)
    applied=duplicate(velocity if model.applied_command is None else model.applied_command)
    last=stamp-.05 if model.last_stamp is None else model.last_stamp
    return ResponseState(float(stamp),parameters.model_key,duplicate(position),duplicate(velocity),previous,applied,
                         np.full(shape[0],last),np.full(shape[0],stamp),model.applied_command is not None)


def _validate(parameters,state):
    if state.model_key!=parameters.model_key or len(state.position)!=len(parameters.scenarios):
        raise ValueError('Rollout state/model mismatch')


def _commands(parameters,previous,last_control,target,velocity,position,stamp,profile,wait=False):
    from velocity_response import VelocityResponse
    target=np.broadcast_to(target,velocity.shape).copy()
    if not wait and profile is not None and hasattr(profile,'xy_target'):
        target[:,:2]=profile.xy_target(target[0],position,velocity)
    if profile is not None:target[:,2]+=profile(position,velocity)
    holder=SimpleNamespace(brake_feedback=parameters.brake_feedback,stop_profile=profile)
    braking=np.linalg.norm(target[:,:2],axis=1)<.05
    if np.any(braking):target[braking,:2]=VelocityResponse.braking_target(holder,velocity[:,:2],position)[braking]
    intervals=np.clip(stamp-last_control,.001,.35)
    slowing=np.sum((target[:,:2]-previous[:,:2])*previous[:,:2],axis=1)<0.
    rate=np.where(slowing|braking,parameters.deceleration,parameters.acceleration)
    delta=target[:,:2]-previous[:,:2]
    target[:,:2]=previous[:,:2]+delta*np.minimum(1.,rate*intervals/np.maximum(1e-9,np.linalg.norm(delta,axis=1)))[:,None]
    applied=VelocityResponse.compensate(parameters,target,velocity)
    # Match commit(), including saturation's effect on stored nominal memory.
    remembered=applied.copy()
    remembered[:,2]-=parameters.lift_gain*np.sum((applied[:,:2]-velocity[:,:2])**2,axis=1)
    return remembered,applied


def rollout_primitive(parameters,state,primitive,profile=None,delay=False):
    _validate(parameters,state)
    if not isinstance(primitive,Primitive):raise ValueError('Expected Primitive')
    p=state.position.copy();v=state.velocity.copy();previous=state.previous.copy();applied=state.applied.copy()
    effective=state.effective.copy();next_physics=state.next_physics
    last=state.last_control.copy();next_control=state.next_control.copy()
    scenarios=np.asarray(parameters.scenarios);taus=np.column_stack([scenarios[:,0],scenarios[:,0],scenarios[:,1]])
    periods=np.asarray(parameters.periods);t=state.stamp;end=t+primitive.duration
    times=[];positions=[];velocities=[];nominals=[];commands=[];bounds=[]
    while t<end-1e-9:
        due=np.zeros(len(p),dtype=bool)
        if not delay:
            due=t>=next_control-1e-9
            if np.any(due):
                nominal,command=_commands(parameters,previous,last,primitive.target,v,p,t,profile,primitive.name=='WAIT')
                previous[due]=nominal[due];applied[due]=command[due]
                last[due]=t;next_control[due]=t+periods[due]
        physics_due=t>=next_physics-1e-9
        if physics_due:next_physics=t+parameters.integration_step
        refresh=due|physics_due
        if np.any(refresh):
            candidate=applied.copy()
            if delay and not state.has_applied:candidate=v.copy()
            else:candidate[:,2]-=scenarios[:,2]*np.sum((applied[:,:2]-v[:,:2])**2,axis=1)
            effective[refresh]=candidate[refresh]
        times.append(t);positions.append(p.copy());velocities.append(v.copy())
        nominals.append(previous.copy());commands.append(applied.copy())
        dt=min(next_physics-t,end-t)
        if not delay:dt=min(dt,float(np.min(next_control-t)))
        if dt<=1e-10:raise RuntimeError('Rollout controller failed to advance')
        bounds.append(abs(effective-v)/taus)
        decay=np.exp(-dt/taus)
        p+=effective*dt+(v-effective)*taus*(1.-decay)
        v=effective+(v-effective)*decay;t+=dt
    # A boundary event belongs to the following primitive, never the old one.
    t=end;next_control=np.maximum(next_control,t)
    times.append(t);positions.append(p.copy());velocities.append(v.copy())
    nominals.append(previous.copy());commands.append(applied.copy())
    new_state=ResponseState(t,state.model_key,p,v,previous,applied,last,next_control,state.has_applied or not delay,effective,max(t,next_physics))
    return ResponseTrace(times,positions,velocities,nominals,commands,new_state,bounds)


def rollout_delay(parameters,state,duration):
    return rollout_primitive(parameters,state,Primitive([0.,0.,0.],duration,'WAIT'),delay=True)


def rollout_sequence(parameters,state,primitives,profiles=None):
    primitives=tuple(primitives);profiles=(None,)*len(primitives) if profiles is None else tuple(profiles)
    if len(profiles)!=len(primitives) or not primitives:raise ValueError('Invalid response sequence')
    traces=[]
    for primitive,profile in zip(primitives,profiles):
        trace=rollout_primitive(parameters,state,primitive,profile);traces.append(trace);state=trace.end_state
    return join_traces(traces)


def rollout_stop(parameters,state,profile=None,max_duration=12.,chunk=.25,deadline=None):
    """Feedback brake until every scenario settles; never truncate as stopped.

    Returns (trace, stopped). An incomplete trace must not be certified as a
    backup. Deadline uses a monotonic wall clock and only affects computation.
    """
    import time
    if not np.isfinite(max_duration) or max_duration<=0 or not np.isfinite(chunk) or chunk<=0:
        raise ValueError('Invalid stop duration')
    initial=state.stamp;traces=[];stopped=False
    while state.stamp-initial<max_duration-1e-9:
        if deadline is not None and time.monotonic()>=deadline:break
        duration=min(chunk,max_duration-(state.stamp-initial))
        trace=rollout_primitive(parameters,state,Primitive([0.,0.,0.],duration,'WAIT'),profile)
        traces.append(trace);state=trace.end_state
        if (state.stamp-initial>=1. and np.max(np.linalg.norm(state.velocity,axis=1))<.01 and
                np.max(np.linalg.norm(state.applied[:,:2],axis=1))<.01):stopped=True;break
    return (join_traces(traces) if traces else None),stopped
