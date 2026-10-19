"""Collective-priority bounded allocator with explicit achieved wrench feedback."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Allocation:
    pwm: tuple
    achieved_wrench: tuple
    saturated: bool
    moment_scale: float
    min_margin: float


class MotorAllocator:
    def __init__(self, config):
        arm = float(config['arm_radius_m']) / math.sqrt(2)
        rps_squared = (float(config['max_rpm'])/60)**2
        self.max_thrust = float(config['thrust_coefficient_rps2'])*rps_squared
        ratio = float(config['torque_coefficient_rps2'])/float(config['thrust_coefficient_rps2'])
        self.ceiling = float(config['local_pwm_ceiling'])
        signs = np.asarray(config['yaw_torque_signs'], dtype=float)
        if (not all(math.isfinite(v) and v > 0 for v in (arm, self.max_thrust, ratio))
                or not 0 < self.ceiling <= 1 or signs.shape != (4,)
                or not np.all(np.isin(signs, (-1, 1)))):
            raise ValueError('invalid motor model')
        # F_body=(0,0,-f). Torque is r cross F; yaw signs are from local pulses.
        self.B = np.array(((1., 1., 1., 1.), (-arm, arm, arm, -arm),
                           (arm, -arm, arm, -arm), tuple(ratio*signs)))
        self.inverse = np.linalg.inv(self.B)

    def allocate(self, thrust, moment):
        request = np.r_[thrust, moment]
        if request.shape != (4,) or not np.all(np.isfinite(request)) or thrust < 0:
            raise ValueError('invalid wrench')
        limit = self.max_thrust*self.ceiling
        collective = min(float(thrust)/4, limit)
        differential = self.inverse @ np.r_[0., moment]
        scale = 1.
        for value in differential:
            if value > 0:
                scale = min(scale, (limit-collective)/value)
            elif value < 0:
                scale = min(scale, collective/-value)
        forces = np.clip(collective+scale*differential, 0., limit)
        pwm = forces/self.max_thrust
        achieved = self.B @ forces
        saturated = not np.allclose(request, achieved, atol=1e-8)
        margin = float(np.minimum(pwm, self.ceiling-pwm).min())
        return Allocation(tuple(pwm), tuple(achieved), saturated, float(scale), margin)
