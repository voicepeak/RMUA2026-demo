#!/usr/bin/env python3
"""Spatial vertical-rate envelope, with braking before a slope changes."""
import math
import numpy as np


class TerrainSpeedEnvelope:
    def __init__(self, braking=4., response=.4, horizon=80., step=1., eta=.95):
        self.braking=float(braking)
        self.response=float(response)
        self.horizon=float(horizon)
        self.step=float(step)
        self.eta=float(eta)

    def evaluate(self, s, center, capability, cruise):
        ds=np.arange(0., self.horizon+self.step/2., self.step)
        slope=np.array([(center(s+d+.5)-center(s+d-.5)) for d in ds])
        # A rate cap is a property of the path, independent of temporary
        # height tracking error. Include curvature of the vertical path.
        available=np.where(slope>0.,capability.down(cruise),capability.up(cruise))*self.eta
        rate=np.minimum(cruise, available/np.maximum(abs(slope),1e-4))
        second=np.gradient(slope,self.step)
        local=np.minimum(rate,np.sqrt(4./np.maximum(abs(second),1e-5)))
        # Reserve actuator/jerk response distance before applying the braking
        # envelope. A future restriction does not become today's speed.
        effective=np.maximum(0.,ds-float(cruise)*self.response)
        caps=np.sqrt(local*local+2.*self.braking*effective)
        i=int(np.argmin(caps))
        return dict(v_climb_preview=float(min(cruise,caps[i])),
                    climb_worst_s=float(s+ds[i]),climb_worst_dz=float(center(s)-center(s+ds[i])),
                    climb_horizon=self.horizon,terrain_local_cap=float(local[0]),
                    terrain_future_cap=float(local[i]))
