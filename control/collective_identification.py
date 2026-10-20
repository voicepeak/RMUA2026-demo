"""One-command collective excitation in a settled local simulator hover.

Never a hover fallback. Flight limits and the host-owned scene stop remain in
force. The normal allocator's PWM ceiling is unchanged.
"""
import math
import numpy as np
from estimation.geometry import rotation
from interfaces.contracts import Health


class CollectivePulse:
    def __init__(self,pwm):
        if not math.isfinite(pwm) or not 0<=pwm<=1:raise ValueError('invalid collective excitation')
        self.pwm=float(pwm)
        self.sent=False
        self.stamp=None

    def request(self,state,elapsed):
        if not math.isfinite(elapsed) or elapsed<0:raise ValueError('invalid excitation time')
        if self.sent or not 5<=elapsed<=8 or state.health!=Health.OK:return None
        if (rotation(state.orientation_wxyz)[2,2]<math.cos(.08)
                or np.linalg.norm(state.angular_velocity_body)>.15
                or np.linalg.norm(state.velocity_ned)>.25):return None
        return (self.pwm,)*4

    def published(self,stamp):
        if self.sent:raise RuntimeError('COLLECTIVE_EXCITATION_REPLAY')
        self.sent=True
        self.stamp=stamp
