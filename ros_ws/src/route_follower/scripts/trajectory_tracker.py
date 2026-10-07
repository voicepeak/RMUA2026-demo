#!/usr/bin/env python3
"""Select only the certified first primitive; never execute the search suffix."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class TrackingLimits:
    position_error: float = .3
    velocity_error: float = .75

    def __post_init__(self):
        if any(not np.isfinite(v) or v<0. for v in vars(self).values()):
            raise ValueError('Invalid tracking tolerance')


class TrajectoryTracker:
    def __init__(self,limits=None):
        self.limits=TrackingLimits() if limits is None else limits

    def select(self,trajectory,snapshot):
        now=snapshot.pose_stamp
        if trajectory.epoch!=snapshot.epoch:return None,None,'EPOCH_MISMATCH'
        if trajectory.trace.end_state.model_key!=snapshot.response_state.model_key:
            return None,None,'MODEL_MISMATCH'
        if now<trajectory.start_stamp-1e-8:return None,None,'PLAN_IN_FUTURE'
        execution_now=now if snapshot.evaluation_stamp is None else snapshot.evaluation_stamp
        if execution_now>=trajectory.valid_until-1e-8:return None,None,'PLAN_EXPIRED'
        if not trajectory.primitives:return None,None,'EMPTY_PLAN'
        # The uncertainty envelope is used for deviation detection only. Every
        # actual command is independently rolled out from the latest observation.
        times=trajectory.trace.times
        def envelope(a):
            values=np.array([[np.interp(now,times,a[:,j,k]) for k in range(3)] for j in range(a.shape[1])])
            return values.min(axis=0),values.max(axis=0)
        for key,tolerance in (('position',self.limits.position_error),('velocity',self.limits.velocity_error)):
            low,high=envelope(getattr(trajectory.trace,'positions' if key=='position' else 'velocities'))
            actual=getattr(snapshot.response_state,key)
            if np.any(actual<low-tolerance) or np.any(actual>high+tolerance):
                return None,None,'TRACKING_'+key.upper()+'_ERROR'
        return trajectory.primitives[0],trajectory.profiles[0],'PASS'
