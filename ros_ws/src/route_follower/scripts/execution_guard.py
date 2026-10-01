#!/usr/bin/env python3
"""Short-path recertification independent of the expensive route search."""
import math
import time
import numpy as np
from point_index import PointIndex
from predictive_avoidance import PredictiveAvoidance


class ExecutionGuard:
    def __init__(self, margin=1.15, braking=4., reaction=.35, horizon=30.):
        self.margin=margin
        self.braking=braking
        self.reaction=reaction
        self.horizon=horizon
        self.cloud_stamp=None
        self.index=None

    def evaluate(self, s, position, velocity, xy, center, plan, points,
                 cloud_stamp, pose_stamp):
        started=time.monotonic()
        age=None if cloud_stamp is None else pose_stamp-cloud_stamp
        if points is None or age is None or not 0.<=age<.5:
            return dict(cap=0., execution_verified=False,
                        guard_reason='LIDAR_STALE', verified_stamp=None)
        if len(points)==0:
            return dict(cap=0.,execution_verified=False,
                        guard_reason='LIDAR_EMPTY',verified_stamp=None)
        if self.index is None or self.cloud_stamp!=cloud_stamp:
            self.index=PointIndex(points)
            self.cloud_stamp=cloud_stamp
        stations=s+np.arange(0., self.horizon+.1, .2)
        path=PredictiveAvoidance.positions(stations,xy,center,plan)
        # Certify the actual position-to-reference connector as well. Starting
        # at the nominal point would hide an obstacle beside a tracking error.
        path[0]=position
        queries,segments,fraction=PredictiveAvoidance.swept_samples(path)
        progress=stations[segments]-s+.2*fraction
        distances=self.index.distance(queries)
        blocked=np.flatnonzero(distances<self.margin+.1)
        free=self.horizon if not len(blocked) else max(0.,progress[blocked[0]]-.2)
        # An actuator cannot remove the measured velocity instantaneously.
        # Validate the response segment before following the spatial path.
        inertia=position+np.linspace(0.,self.reaction,8)[:,None]*np.asarray(velocity)
        response_clear=float(np.min(self.index.distance(inertia)))
        if response_clear<self.margin+.1:free=0.
        latency=self.reaction+age
        cap=max(0.,math.sqrt((self.braking*latency)**2+2.*self.braking*free)-self.braking*latency)
        return dict(cap=cap, execution_verified=bool(free>0.),
                    guard_reason=('EXECUTION_BLOCKED' if free<=0. else
                                  'BRAKING_OBSTACLE' if len(blocked) else 'EXECUTION_CLEAR'),
                    verified_stamp=pose_stamp, verified_cloud_stamp=cloud_stamp,
                    verified_distance=float(free),
                    execution_clearance=float(np.min(distances)) if np.all(np.isfinite(distances)) else None,
                    response_clearance=response_clear if math.isfinite(response_clear) else None,
                    validation_ms=1000.*(time.monotonic()-started))
