"""Small shared formal-input sensor adapter; callbacks never wait for perception."""
import threading
import time
from collections import deque
import numpy as np
from estimation.clock import SensorClock
from estimation.imu_gps import ImuGpsEstimator
from adapters.ros_noetic.state_reader import pose_values


class FlightSensors:
    def __init__(self, subscribe=True):
        self.lock = threading.RLock()
        self.clock = SensorClock()
        self.estimator = ImuGpsEstimator()
        self.initial = None
        self.raw = None
        self.fault = None
        self.local_injection = None
        self.pending_gps = None
        self.subscribers = []
        self.bootstrap_imu=deque(maxlen=180)
        self.bootstrap_gps=deque(maxlen=24)
        self.bootstrap_applied=False
        if subscribe:
            import rospy
            from geometry_msgs.msg import PoseStamped
            from sensor_msgs.msg import Imu
            self.subscribers = [
                rospy.Subscriber('/airsim_node/initial_pose', PoseStamped, self.on_initial, queue_size=1),
                rospy.Subscriber('/airsim_node/drone_1/imu/imu', Imu, self.on_imu, queue_size=1),
                rospy.Subscriber('/airsim_node/drone_1/gps', PoseStamped, self.on_gps, queue_size=1),
            ]

    def on_initial(self, msg):
        with self.lock:
            if self.initial is None:
                self.initial = pose_values(msg.pose)

    def on_imu(self, msg):
        with self.lock:
            try:
                stamp = msg.header.stamp.to_sec()
                if self.local_injection is not None:
                    if self.local_injection.affects('IMU_DROP', stamp):
                        return
                    if self.local_injection.affects('IMU_ROLLBACK', stamp):
                        stamp -= 1.
                previous = self.clock.stamp
                if self.clock.observe(stamp, time.monotonic()):
                    self.fault = 'IMU_CLOCK_RESET'
                    return
                if previous == stamp:
                    return
                if self.initial is not None and not self.estimator.initialized:
                    self.estimator.initialize(*self.initial)
                a, w = msg.linear_acceleration, msg.angular_velocity
                acceleration, omega = (a.x, a.y, a.z), (w.x, w.y, w.z)
                self.estimator.propagate(stamp, acceleration, omega)
                if self.pending_gps is not None and self.pending_gps[0] <= stamp:
                    try:
                        if self.estimator.update_gps(*self.pending_gps):
                            self.bootstrap_gps.append(self.pending_gps)
                    except (ValueError, ArithmeticError) as error:
                        self.estimator.reject('INVALID_GPS:'+str(error))
                    self.pending_gps = None
                self.raw = {'stamp': stamp, 'acceleration_body': acceleration,
                            'omega_body': omega}
                attitude = getattr(msg,'orientation',None)
                if attitude is not None:
                    # Record for calibration only; it does not enter the filter.
                    self.raw['imu_orientation_wxyz']=(attitude.w,attitude.x,attitude.y,attitude.z)
                if not self.bootstrap_applied:self.bootstrap_imu.append(dict(self.raw))
            except (ValueError, ArithmeticError) as error:
                self.fault = str(error)

    def on_gps(self, msg):
        with self.lock:
            try:
                stamp = msg.header.stamp.to_sec()
                if stamp > 0 and self.clock.fresh(time.monotonic()):
                    position, attitude = pose_values(msg.pose)
                    if self.local_injection is not None and self.local_injection.affects('GPS_ZERO',stamp):
                        position, attitude = (0.,0.,0.), (0.,0.,0.,0.)
                    if self.clock.stamp < stamp <= self.clock.stamp+.15:
                        # ROS callback arrival order is not sensor measurement order.
                        # Defer future GPS to the first IMU propagation covering it.
                        self.pending_gps = (stamp, position, attitude)
                    else:
                        if self.estimator.update_gps(stamp, position, attitude):
                            self.bootstrap_gps.append((stamp,position,attitude))
            except (ValueError, ArithmeticError) as error:
                self.estimator.reject('INVALID_GPS:'+str(error))

    def read(self):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            if not self.clock.fresh(time.monotonic()):
                raise RuntimeError('STALE_IMU')
            raw = dict(self.raw)
            raw['last_gps_stamp'] = self.estimator.last_gps
            raw['gyro_bias_body'] = tuple(self.estimator.bg)
            raw['accel_bias_body'] = tuple(self.estimator.ba)
            return self.estimator.snapshot(), raw

    def bootstrap_at_rest_before_control(self):
        """Caller must not have created a control publisher; never call in flight."""
        from estimation.stationary_bootstrap import resting_window
        with self.lock:
            if self.bootstrap_applied:raise RuntimeError('BOOTSTRAP_ALREADY_APPLIED')
            if self.fault or self.initial is None or not self.clock.fresh(time.monotonic()):return None
            result=resting_window(self.bootstrap_imu,self.bootstrap_gps,self.initial[1],self.clock.stamp)
            if result is None:return None
            e=self.estimator
            e.p=np.array(result['position']);e.v=np.zeros(3)
            e.q=np.array(self.initial[1],dtype=float);e.q/=np.linalg.norm(e.q)
            e.bg=np.array(result['gyro_bias']);e.ba=np.zeros(3)
            # Retain conservative GPS/attitude/bias uncertainty; averaging does
            # not establish independent measurement noise or identify accel bias.
            e.P=np.diag([.01]*3+[.01]*3+[.01]*3+[.01]*3+[.001]*3)
            e.omega=np.array(self.raw['omega_body'])-e.bg
            self.bootstrap_applied=True
            return result

    def close(self):
        for subscriber in self.subscribers:
            subscriber.unregister()
