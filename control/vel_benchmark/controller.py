"""NED position reference to body velocity, simulator z-up / yaw degrees adapter."""
from dataclasses import dataclass
import math
import numpy as np
from estimation.geometry import rotation


@dataclass(frozen=True)
class VelocityCommand:
    stamp: float
    epoch: int
    sequence: int
    values: tuple

    def __post_init__(self):
        if (not math.isfinite(self.stamp) or self.stamp <= 0 or self.epoch < 0
                or self.sequence < 0 or not isinstance(self.values, tuple) or len(self.values) != 4
                or not all(math.isfinite(v) for v in self.values)
                or max(abs(v) for v in self.values[:3]) > 1.
                or abs(self.values[3]) > 30.):
            raise ValueError('invalid bounded VEL command')


def body_velocity(state, position, velocity=(0., 0., 0.), yaw_reference=0., yaw_rate=0.):
    reference = np.r_[position, velocity, yaw_reference, yaw_rate]
    if reference.shape != (8,) or not np.all(np.isfinite(reference)):
        raise ValueError('invalid VEL reference')
    v = np.asarray(velocity)+1.2*(np.asarray(position)-state.position_ned)
    norm = np.linalg.norm(v)
    if norm > .6:
        v *= .6/norm
    R = rotation(state.orientation_wxyz)
    yaw = math.atan2(R[1, 0], R[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    yaw_error = (yaw_reference-yaw+math.pi) % (2*math.pi)-math.pi
    return (float(c*v[0]+s*v[1]), float(-s*v[0]+c*v[1]), float(-v[2]),
            float(np.clip(math.degrees(yaw_error+yaw_rate), -20., 20.)))
