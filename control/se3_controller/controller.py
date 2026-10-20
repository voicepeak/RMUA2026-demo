"""Conservative development controller. Gains require closed-loop evidence."""
from dataclasses import dataclass
import math
import numpy as np
from estimation.geometry import rotation


@dataclass(frozen=True)
class Wrench:
    thrust: float
    moment: tuple
    acceleration_reference: tuple
    integral: tuple


class Se3Controller:
    def __init__(self, config):
        self.mass = float(config['mass_kg'])
        self.inertia = np.asarray(config['inertia_kg_m2'], dtype=float)
        self.integral = np.zeros(3)
        self.kp = np.array((1.5, 1.5, 2.5))
        self.kd = np.array((2., 2., 2.))
        self.ki = np.array((.1, .1, .2))
        self.kr = np.array((.12, .16, .08))
        self.kw = np.array((.035, .045, .035))

    def step(self, state, position, velocity=(0., 0., 0.), acceleration=(0., 0., 0.),
             yaw=0., dt=.0125, allow_integral=True, yaw_rate=0.,rate_gain_scale=1.):
        reference = np.r_[position, velocity, acceleration, yaw, dt, yaw_rate]
        if (reference.shape != (12,) or not np.all(np.isfinite(reference))
                or not 0 < dt <= .05 or not math.isfinite(rate_gain_scale)
                or not 1.<=rate_gain_scale<=2.):
            raise ValueError('invalid controller reference or dt')
        R = rotation(state.orientation_wxyz)
        omega = np.asarray(state.angular_velocity_body)
        error = np.asarray(position)-state.position_ned
        v_error = np.asarray(velocity)-state.velocity_ned
        if allow_integral:
            self.integral = np.clip(self.integral + error*dt, -1., 1.)
        a = np.asarray(acceleration)+self.kp*error+self.kd*v_error+self.ki*self.integral
        a[2] = np.clip(a[2], -2., 2.)
        horizontal = np.linalg.norm(a[:2])
        if horizontal > 1.5:
            a[:2] *= 1.5/horizontal
        force = self.mass*(a-(0., 0., 9.81))
        b3 = -force/np.linalg.norm(force)
        heading = np.array((math.cos(yaw), math.sin(yaw), 0.))
        b2 = np.cross(b3, heading)
        b2 /= np.linalg.norm(b2)
        Rd = np.column_stack((np.cross(b2, b3), b2, b3))
        error_matrix = .5*(Rd.T @ R - R.T @ Rd)
        attitude_error = np.array((error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]))
        desired_body_rate = R.T @ (0., 0., yaw_rate)
        moment = -self.kr*attitude_error-rate_gain_scale*self.kw*(omega-desired_body_rate)+np.cross(omega, self.inertia*omega)
        thrust = max(0., float(-force @ R[:, 2]))
        return Wrench(thrust, tuple(moment), tuple(a), tuple(self.integral))
