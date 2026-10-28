"""15-state error-state Kalman filter. Provisional noise, pending calibration.

Nominal state p_NED, v_NED, q_body_to_NED, accel bias, gyro bias.
Error order p,v,local rotation,ba,bg. No debug or prerecorded route input.
GPS outages retain inertial propagation and explicitly degrade health.
"""
import math
import numpy as np
from interfaces.contracts import Health, Metadata, StateEstimate
from estimation.geometry import delta_quaternion, multiply, rotation, skew, unit


def vector(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError("invalid vector")
    return result


class ImuGpsEstimator:
    def __init__(self, epoch=0):
        self.epoch = epoch
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array((1., 0., 0., 0.))
        self.ba = np.zeros(3)
        self.bg = np.zeros(3)
        self.P = np.diag([.1]*3 + [1.]*3 + [.01]*3 + [.01]*3 + [.001]*3)
        self.stamp = None
        self.last_gps = None
        self.omega = np.zeros(3)
        self.initialized = False
        self.gap = False
        self.rejections = {}

    def initialize(self, position, attitude):
        if self.initialized:
            return False
        self.p, self.q = vector(position).copy(), unit(attitude)
        self.initialized = True
        return True

    def propagate(self, stamp, acceleration_body, angular_velocity_body):
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError("invalid IMU stamp")
        acceleration, omega = vector(acceleration_body), vector(angular_velocity_body)
        if self.stamp is None:
            self.omega = omega - self.bg
            self.stamp = stamp
            return
        dt = stamp - self.stamp
        if dt <= 0:
            raise ValueError("IMU must increase within epoch")
        self.omega = omega - self.bg
        self.stamp = stamp
        if dt > .1:
            # Missing integration interval cannot be silently treated as fresh.
            self.P += np.eye(15) * dt
            self.gap = True
            return
        R = rotation(self.q)
        specific_force = acceleration - self.ba
        world_acceleration = R @ specific_force + (0., 0., 9.81)
        self.p += self.v * dt + .5 * world_acceleration * dt**2
        self.v += world_acceleration * dt
        self.q = unit(multiply(self.q, delta_quaternion(self.omega*dt)))
        F = np.zeros((15, 15))
        F[:3, 3:6] = np.eye(3)
        F[3:6, 6:9] = -R @ skew(specific_force)
        F[3:6, 9:12] = -R
        F[6:9, 6:9] = -skew(self.omega)
        F[6:9, 12:15] = -np.eye(3)
        A = np.eye(15) + F*dt
        Q = np.diag([1e-8]*3 + [.15**2]*3 + [.01**2]*3 + [1e-6]*3 + [1e-8]*3)*dt
        self.P = A @ self.P @ A.T + Q

    def reject(self, reason):
        self.rejections[reason] = self.rejections.get(reason, 0) + 1
        return False

    def update_gps(self, stamp, position, attitude):
        position = vector(position)
        # Factory packets can zero BOTH position and quaternion. Reject the
        # sentinel before trying to normalize an invalid all-zero quaternion.
        if np.all(position == 0):
            return self.reject("GPS_ZERO")
        if not self.initialized or self.stamp is None:
            return self.reject("NOT_INITIALIZED")
        if not math.isfinite(stamp) or not 0 <= self.stamp-stamp <= .15:
            return self.reject("GPS_TIME")
        if self.last_gps is not None and stamp <= self.last_gps:
            return self.reject("GPS_REORDERED")
        attitude = unit(attitude)
        conjugate = self.q * (1., -1., -1., -1.)
        error_q = multiply(conjugate, attitude)
        if error_q[0] < 0:
            error_q = -error_q
        residual = np.r_[position-self.p, 2*error_q[1:]]
        H = np.zeros((6, 15))
        H[:3, :3] = np.eye(3)
        H[3:, 6:9] = np.eye(3)
        # GPS documented position <= .1m / attitude <= .2rad, conservative baseline.
        noise = np.diag([.1**2]*3 + [.2**2]*3)
        S = H @ self.P @ H.T + noise
        if residual @ np.linalg.solve(S, residual) > 30.:
            return self.reject("GPS_INNOVATION")
        K = np.linalg.solve(S, H @ self.P).T
        error = K @ residual
        self.p += error[:3]
        self.v += error[3:6]
        self.q = unit(multiply(self.q, delta_quaternion(error[6:9])))
        self.ba += error[9:12]
        self.bg += error[12:15]
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ noise @ K.T
        # Reset covariance after injecting local attitude error.
        G = np.eye(15)
        G[6:9, 6:9] -= .5*skew(error[6:9])
        self.P = G @ self.P @ G.T
        self.P = (self.P+self.P.T)*.5
        self.last_gps = stamp
        self.gap = False
        return True

    def snapshot(self):
        if not self.initialized or self.stamp is None:
            raise ValueError("estimator not initialized")
        age = math.inf if self.last_gps is None else self.stamp-self.last_gps
        health = Health.OK
        if age > .3 or self.gap:
            health = Health.DEGRADED
        if age > 2. or np.max(np.diag(self.P)[:3]) > 4.:
            health = Health.LOST
        metadata = Metadata(self.stamp, self.stamp+.05, self.epoch, "imu_gps_eskf_v1",
                            covariance_or_bounds=tuple(self.P.flatten()))
        return StateEstimate(metadata, tuple(self.p), tuple(self.v), tuple(self.q),
                             tuple(self.omega), health)
