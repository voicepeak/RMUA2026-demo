#!/usr/bin/env python3
"""Immutable numerical contracts for offline space-time planning (world NED)."""
from dataclasses import dataclass
from typing import Optional,Tuple
import numpy as np


def readonly(value,dtype=float):
    result=np.asarray(value,dtype=dtype).copy();result.setflags(write=False);return result


@dataclass(frozen=True)
class ResponseState:
    stamp: float
    model_key: str
    position: np.ndarray
    velocity: np.ndarray
    previous: np.ndarray
    applied: np.ndarray
    last_control: np.ndarray
    next_control: np.ndarray
    has_applied: bool = True
    effective: object = None
    next_physics: object = None

    def __post_init__(self):
        if not np.isfinite(self.stamp) or not self.model_key:raise ValueError('Invalid response clock/model')
        shape=np.shape(self.position)
        if len(shape)!=2 or shape[1]!=3 or shape[0]<1:raise ValueError('Response state must be scenarios x XYZ')
        for key in ('position','velocity','previous','applied','last_control','next_control'):
            value=readonly(getattr(self,key));expected=shape if key not in ('last_control','next_control') else (shape[0],)
            if value.shape!=expected or not np.all(np.isfinite(value)):raise ValueError('Invalid response state: '+key)
            object.__setattr__(self,key,value)
        if np.any(self.last_control>self.stamp+1e-8) or np.any(self.next_control<self.stamp-1e-8):
            raise ValueError('Response control clocks are inconsistent')
        effective=readonly(self.applied if self.effective is None else self.effective)
        next_physics=self.stamp if self.next_physics is None else float(self.next_physics)
        if effective.shape!=shape or not np.all(np.isfinite(effective)) or not np.isfinite(next_physics) or next_physics<self.stamp-1e-8:
            raise ValueError('Invalid persistent physics state')
        object.__setattr__(self,'effective',effective);object.__setattr__(self,'next_physics',next_physics)


@dataclass(frozen=True)
class Primitive:
    target: np.ndarray
    duration: float
    name: str = 'MOVE'

    def __post_init__(self):
        target=readonly(self.target)
        if target.shape!=(3,) or not np.all(np.isfinite(target)) or not np.isfinite(self.duration) or self.duration<=0:
            raise ValueError('Invalid response primitive')
        if self.name not in ('MOVE','WAIT'):raise ValueError('Unknown primitive mode')
        if self.name=='WAIT' and np.linalg.norm(target[:2])>=.05:raise ValueError('WAIT requires a braking target')
        object.__setattr__(self,'target',target)


@dataclass(frozen=True)
class ResponseTrace:
    times: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    nominal_commands: np.ndarray
    applied_commands: np.ndarray
    end_state: ResponseState
    acceleration_bounds: object = None

    def __post_init__(self):
        times=readonly(self.times)
        shape=(len(times),len(self.end_state.position),3)
        if times.ndim!=1 or len(times)<2 or not np.all(np.isfinite(times)) or np.any(np.diff(times)<=0):
            raise ValueError('Invalid trajectory times')
        if abs(times[-1]-self.end_state.stamp)>1e-8:raise ValueError('Trajectory/state clocks differ')
        object.__setattr__(self,'times',times)
        for key in ('positions','velocities','nominal_commands','applied_commands'):
            values=readonly(getattr(self,key))
            if values.shape!=shape or not np.all(np.isfinite(values)):raise ValueError('Invalid trace: '+key)
            object.__setattr__(self,key,values)
        if not np.allclose(self.positions[-1],self.end_state.position,rtol=0.,atol=1e-8) or not np.allclose(self.velocities[-1],self.end_state.velocity,rtol=0.,atol=1e-8):
            raise ValueError('Trajectory endpoint differs from response state')
        if self.acceleration_bounds is not None:
            bounds=readonly(self.acceleration_bounds)
            if bounds.shape!=(len(times)-1,shape[1],3) or not np.all(np.isfinite(bounds)) or np.any(bounds<0):
                raise ValueError('Invalid inter-sample acceleration bounds')
            object.__setattr__(self,'acceleration_bounds',bounds)


def join_traces(traces):
    traces=tuple(traces)
    if not traces:raise ValueError('No traces to join')
    for a,b in zip(traces,traces[1:]):
        if (a.end_state.model_key!=b.end_state.model_key or abs(a.times[-1]-b.times[0])>1e-8 or
                not np.allclose(a.positions[-1],b.positions[0],rtol=0.,atol=1e-8) or
                not np.allclose(a.velocities[-1],b.velocities[0],rtol=0.,atol=1e-8)):
            raise ValueError('Discontinuous response sequence')
    # At a seam, the next primitive owns the command for the next interval.
    combine=lambda key:np.concatenate([getattr(t,key)[:-1] for t in traces[:-1]]+[getattr(traces[-1],key)])
    bounds=None if any(t.acceleration_bounds is None for t in traces) else np.concatenate([t.acceleration_bounds for t in traces])
    return ResponseTrace(combine('times'),combine('positions'),combine('velocities'),
                         combine('nominal_commands'),combine('applied_commands'),traces[-1].end_state,bounds)


@dataclass(frozen=True)
class CollisionResult:
    safe: bool
    reason: str
    stamp: Optional[float] = None
    position: Optional[Tuple[float,float,float]] = None
    scenario: Optional[int] = None
    track_id: Optional[int] = None
    separation: Optional[float] = None


@dataclass(frozen=True)
class PlanningSnapshot:
    epoch: int
    scene_version: int
    pose_stamp: float
    response_state: ResponseState
    route: object
    occupancy: object
    obstacles: tuple = ()
    reaction_delay: float = 0.
    evaluation_stamp: Optional[float] = None
    sensor_origin: object = None

    def __post_init__(self):
        if (not isinstance(self.epoch,int) or self.epoch<0 or not isinstance(self.scene_version,int) or
                self.scene_version<0 or not np.isfinite(self.pose_stamp) or
                abs(self.pose_stamp-self.response_state.stamp)>1e-8 or
                not np.isfinite(self.reaction_delay) or self.reaction_delay<0.):raise ValueError('Invalid planning snapshot')
        if self.evaluation_stamp is not None and (not np.isfinite(self.evaluation_stamp) or self.evaluation_stamp<self.pose_stamp):
            raise ValueError('Invalid current scene evaluation clock')
        if self.sensor_origin is not None:
            origin=readonly(self.sensor_origin)
            if origin.shape!=(3,) or not np.all(np.isfinite(origin)):raise ValueError('Invalid exposure sensor origin')
            object.__setattr__(self,'sensor_origin',origin)
        object.__setattr__(self,'obstacles',tuple(self.obstacles))


@dataclass(frozen=True)
class TimedTrajectory:
    plan_id: int
    epoch: int
    scene_version: int
    start_stamp: float
    valid_until: float
    trace: ResponseTrace
    primitives: tuple
    profiles: tuple
    backup: ResponseTrace
    backup_profile: object
    cost: float
    validation: CollisionResult
    collision_key: str = ''

    def __post_init__(self):
        if (not np.isfinite(self.cost) or self.cost<0 or not self.validation.safe or
                abs(self.start_stamp-self.trace.times[0])>1e-8 or
                not self.start_stamp<self.valid_until<=self.trace.times[-1]+1e-8 or
                abs(self.valid_until-self.backup.times[0])>1e-8):raise ValueError('Invalid certified trajectory')
        object.__setattr__(self,'primitives',tuple(self.primitives));object.__setattr__(self,'profiles',tuple(self.profiles))
        if len(self.primitives)!=len(self.profiles):raise ValueError('Primitive/profile mismatch')


@dataclass(frozen=True)
class PlanResult:
    reason: str
    trajectory: Optional[TimedTrajectory]
    expanded: int
    candidates: int
    rejected: tuple
    elapsed_ms: float
    budget_exhausted: bool
    first_conflict: Optional[CollisionResult] = None
    blocking_track_id: Optional[int] = None

    def summary(self):
        trajectory=self.trajectory
        return dict(reason=self.reason,expanded=self.expanded,candidates=self.candidates,
                    rejected=dict(self.rejected),elapsed_ms=self.elapsed_ms,budget_exhausted=self.budget_exhausted,
                    plan_id=None if trajectory is None else trajectory.plan_id,
                    valid_until=None if trajectory is None else trajectory.valid_until,
                    model_key=None if trajectory is None else trajectory.trace.end_state.model_key,
                    blocking_track_id=self.blocking_track_id,
                    first_conflict=None if self.first_conflict is None else vars(self.first_conflict))
