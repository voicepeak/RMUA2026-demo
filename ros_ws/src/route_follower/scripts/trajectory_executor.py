#!/usr/bin/env python3
"""ROS-independent unique command authority with fail-closed plan deadlines.

The caller invokes tick at a measured fixed period and publishes its world-NED
command exactly once. Slow perception and search never run in this object.
FAILSAFE zero is explicitly uncertified; it is not a safe waiting trajectory.
"""
from dataclasses import dataclass,asdict
import hashlib
import json
import time
import numpy as np
from trajectory_types import Primitive,readonly
from route_coordinates import HeightProfile
from trajectory_tracker import TrajectoryTracker
from response_rollout import rollout_primitive,rollout_stop
from trajectory_collision import CollisionConfig,TrajectoryCollision


def collision_key(config):
    return hashlib.sha256(json.dumps(asdict(config),sort_keys=True).encode()).hexdigest()


@dataclass(frozen=True)
class ExecutionDecision:
    command: np.ndarray
    mode: str
    reason: str
    certified: bool
    plan_id: object = None
    conflict: object = None
    trace: object = None
    backup: object = None

    def __post_init__(self):object.__setattr__(self,'command',readonly(self.command))


class TrajectoryExecutor:
    def __init__(self,parameters,collision_config=None,period=.05,pose_timeout=.3,
                 max_tick_gap=.15,guard_budget=.04,tracker=None):
        self.parameters=parameters
        self.collision_config=CollisionConfig() if collision_config is None else collision_config
        if any(not np.isfinite(v) or v<=0 for v in (period,pose_timeout,max_tick_gap,guard_budget)):
            raise ValueError('Invalid executor timing')
        if abs(min(parameters.periods)-period)>1e-8:
            raise ValueError('Executor period must match fastest frozen feedback scenario')
        self.period=period;self.pose_timeout=pose_timeout;self.max_tick_gap=max_tick_gap;self.guard_budget=guard_budget
        self.tracker=TrajectoryTracker() if tracker is None else tracker
        self.plan=None;self.last_tick=None;self.last_stamp=None;self.last_profile=None;self.epoch=None
        self.last_decision=None;self.rejections={}
        # Warm imports before the first timed publication.
        from execution_guard import certify_spacetime_command
        self.certify=certify_spacetime_command

    def reset(self,epoch=None):
        self.plan=None;self.last_profile=None;self.epoch=epoch;self.last_tick=None;self.last_stamp=None

    def accept(self,trajectory,snapshot,config_key,submitted_monotonic,now_monotonic=None):
        now=time.monotonic() if now_monotonic is None else now_monotonic
        reason='PASS'
        if config_key!=collision_key(self.collision_config) or trajectory.collision_key!=config_key:
            reason='COLLISION_CONFIG_MISMATCH'
        elif now<submitted_monotonic or now-submitted_monotonic>=trajectory.valid_until-trajectory.start_stamp:
            reason='RESULT_EXPIRED'
        else:_,_,reason=self.tracker.select(trajectory,snapshot)
        if reason=='PASS':
            if self.epoch!=snapshot.epoch:self.reset(snapshot.epoch)
            self.plan=trajectory;return True
        self.rejections[reason]=self.rejections.get(reason,0)+1;return False

    def tick(self,snapshot,now_monotonic=None,pose_arrival=None,bridge_verified=False):
        now=time.monotonic() if now_monotonic is None else now_monotonic
        gap=None if self.last_tick is None else now-self.last_tick;self.last_tick=now
        def failsafe(reason,conflict=None):
            self.plan=None
            return ExecutionDecision(np.zeros(3),'FAILSAFE',reason,False,conflict=conflict)
        if not np.isfinite(now):return failsafe('INVALID_WALL_CLOCK')
        if not bridge_verified:return failsafe('BRIDGE_CONTRACT_UNVERIFIED')
        if snapshot is None:return failsafe('SNAPSHOT_UNAVAILABLE')
        if pose_arrival is None or not np.isfinite(pose_arrival) or not 0.<=now-pose_arrival<self.pose_timeout:
            return failsafe('POSE_STALE')
        stamp=snapshot.pose_stamp
        if self.epoch!=snapshot.epoch:
            self.reset(snapshot.epoch);self.last_tick=now
        if self.last_stamp is not None and stamp<self.last_stamp-1e-8:
            self.reset(snapshot.epoch);self.last_stamp=stamp;return failsafe('CLOCK_RESET')
        self.last_stamp=stamp
        if snapshot.response_state.model_key!=self.parameters.model_key:return failsafe('MODEL_MISMATCH')
        checker=TrajectoryCollision(snapshot,self.collision_config)
        fresh=checker.freshness()
        if not fresh.safe:return failsafe(fresh.reason,fresh)
        reason='NO_PLAN';primitive=None;profile=None;plan_id=None
        if self.plan is not None:
            primitive,profile,reason=self.tracker.select(self.plan,snapshot);plan_id=self.plan.plan_id
            if snapshot.pose_stamp+snapshot.reaction_delay+self.period>self.plan.valid_until+1e-8:
                primitive=None;reason='PLAN_EXPIRED'
        if gap is not None and (gap<0 or gap>self.max_tick_gap):primitive=None;reason='EXECUTOR_DELAY'
        if primitive is None:
            self.plan=None
            profile=self.last_profile
            if profile is None:
                coord=snapshot.route.project(snapshot.response_state.position)
                profile=HeightProfile(snapshot.route,float(np.median(coord[:,1])),float(np.median(coord[:,2])))
            primitive=Primitive([0.,0.,0.],self.period,'WAIT')
        # A stale route profile cannot certify a different route version.
        old=profile.route;new=snapshot.route
        same_route=(old is new or (old.lateral_limit==new.lateral_limit and old.vertical_limit==new.vertical_limit and
            np.array_equal(old.stations,new.stations) and np.array_equal(old.points,new.points) and
            np.array_equal(old.floor_limits,new.floor_limits)))
        if not same_route:
            self.plan=None;profile=HeightProfile(snapshot.route,*np.median(snapshot.route.project(snapshot.response_state.position),axis=0)[1:])
            primitive=Primitive([0.,0.,0.],self.period,'WAIT');reason='ROUTE_CHANGED'
        deadline=time.monotonic()+self.guard_budget
        decision=self.certify(snapshot,self.parameters,primitive,profile,self.collision_config,
                                           self.period,deadline)
        if decision.certified:
            self.last_profile=profile
            return ExecutionDecision(decision.command,'NORMAL' if self.plan is not None else 'EMERGENCY_STOP',
                                     reason,True,plan_id,trace=decision.trace,backup=decision.backup)
        self.plan=None
        brake=Primitive([0.,0.,0.],self.period,'WAIT')
        # Only this one braking policy is attempted. No escape/recovery search.
        stopped=self.certify(snapshot,self.parameters,brake,profile,self.collision_config,
                                          self.period,deadline)
        if stopped.certified:
            self.last_profile=profile
            return ExecutionDecision(stopped.command,'EMERGENCY_STOP',decision.reason,True,plan_id,
                                     decision.conflict,stopped.trace,stopped.backup)
        return failsafe('EMERGENCY_BLOCK:'+stopped.reason,stopped.conflict)
