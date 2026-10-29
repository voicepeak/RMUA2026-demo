"""Small shared formal-input sensor adapter; callbacks never wait for perception."""
import threading
import time
from collections import deque
import numpy as np
from estimation.clock import SensorClock
from estimation.imu_gps import ImuGpsEstimator
from adapters.ros_noetic.state_reader import pose_values


class FlightSensors:
    def __init__(self, subscribe=True, pose_history=None, track_startup_velocity=False,
                 allow_stationary_bootstrap=False):
        self.lock = threading.RLock()
        self.clock = SensorClock()
        self.estimator = ImuGpsEstimator()
        self.initial = None
        self.raw = None
        self.fault = None
        self.rate_guard = None
        self.rate_guard_event = None
        self.local_injection = None
        self.pending_gps = None
        self.subscribers = []
        self.bootstrap_imu=deque(maxlen=180)
        self.bootstrap_gps=deque(maxlen=24)
        self.bootstrap_applied=False
        self._bootstrap_permitted = allow_stationary_bootstrap
        self.pose_history=pose_history
        self.startup_diagnostics = {'first_imu_callback_wall_monotonic': None,
                                    'initial_pose_callback_wall_monotonic': None,
                                    'initial_pose_accepted_wall_monotonic': None,
                                    'initial_pose_stamp': None,
                                    'initial_pose_frame_id': None,
                                    'initial_position': None,
                                    'initial_orientation_wxyz': None,
                                    'imu_skipped_before_initial': 0,
                                    'first_initialized_imu_stamp': None,
                                    'first_initialized_imu_callback_wall_monotonic': None}
        self.startup_velocity = {'imu_delta': np.zeros(3), 'gps_delta': np.zeros(3),
                                 'bootstrap_delta': np.zeros(3), 'gps_updates': 0,
                                 'imu_updates': 0, 'sealed': not track_startup_velocity, 'velocity_at_seal': None,
                                 'stamp_at_seal': None, 'gps_events': [],
                                 'gps_events_dropped': 0}
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
        arrival = time.monotonic()
        with self.lock:
            if self.initial is None:
                self.initial = pose_values(msg.pose)
                self.startup_diagnostics['initial_position'] = list(self.initial[0])
                self.startup_diagnostics['initial_orientation_wxyz'] = list(self.initial[1])
                self.startup_diagnostics['initial_pose_stamp'] = msg.header.stamp.to_sec()
                self.startup_diagnostics['initial_pose_frame_id'] = getattr(msg.header, 'frame_id', '')
                self.startup_diagnostics['initial_pose_callback_wall_monotonic'] = arrival
                self.startup_diagnostics['initial_pose_accepted_wall_monotonic'] = time.monotonic()

    def on_imu(self, msg):
        callback_arrival=time.monotonic()
        with self.lock:
            if self.startup_diagnostics['first_imu_callback_wall_monotonic'] is None:
                self.startup_diagnostics['first_imu_callback_wall_monotonic'] = callback_arrival
            try:
                stamp = msg.header.stamp.to_sec()
                if self.local_injection is not None:
                    if self.local_injection.affects('IMU_DROP', stamp):
                        return
                    if self.local_injection.affects('IMU_ROLLBACK', stamp):
                        stamp -= 1.
                previous = self.clock.stamp
                if self.clock.observe(stamp, callback_arrival):
                    if self.pose_history is not None:self.pose_history.reset(self.clock.epoch)
                    self.fault = self.fault or 'IMU_CLOCK_RESET'
                    return
                if previous == stamp:
                    return
                if self.initial is None:
                    self.startup_diagnostics['imu_skipped_before_initial'] += 1
                    return
                if not self.estimator.initialized:
                    self.estimator.initialize(*self.initial)
                a, w = msg.linear_acceleration, msg.angular_velocity
                acceleration, omega = (a.x, a.y, a.z), (w.x, w.y, w.z)
                if (self.local_injection is not None and self.local_injection.injected_samples == 0
                        and self.local_injection.affects('IMU_RATE_SPIKE', stamp)):
                    # Local diagnostic sample only; do not alter the ROS message.
                    omega = (2., 0., 0.)
                before_v = self.estimator.v.copy() if not self.startup_velocity['sealed'] else None
                self.estimator.propagate(stamp, acceleration, omega)
                if before_v is not None:
                    self.startup_velocity['imu_delta'] += self.estimator.v - before_v
                    self.startup_velocity['imu_updates'] += 1
                if self.startup_diagnostics['first_initialized_imu_stamp'] is None:
                    self.startup_diagnostics['first_initialized_imu_stamp'] = stamp
                    self.startup_diagnostics['first_initialized_imu_callback_wall_monotonic'] = callback_arrival
                self._check_rate_guard(stamp, callback_arrival)
                if self.pending_gps is not None and self.pending_gps[0] <= stamp:
                    try:
                        if self._update_gps(*self.pending_gps):
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
                if self.pose_history is not None and self.estimator.initialized:
                    self.pose_history.append(self.estimator.snapshot(), callback_arrival)
            except (ValueError, ArithmeticError) as error:
                self.fault = self.fault or str(error)

    def _check_rate_guard(self, stamp, arrival):
        if self.rate_guard is None or self.fault is not None:
            return
        limit, reason = self.rate_guard
        omega = self.estimator.omega
        magnitude = float(np.linalg.norm(omega))
        if magnitude > limit:
            self.fault = reason
            self.rate_guard_event = {'stamp': stamp, 'arrival_wall_monotonic': arrival,
                                     'angular_velocity_body': tuple(float(x) for x in omega),
                                     'rate_norm_rad_s': magnitude, 'limit_rad_s': limit}

    def arm_rate_guard(self, recovery=False):
        """Latch existing experiment bounds on every accepted IMU sample."""
        with self.lock:
            if self.rate_guard is not None:
                raise RuntimeError('RATE_GUARD_ALREADY_ARMED')
            if self.fault:
                raise RuntimeError(self.fault)
            if not self.estimator.initialized or self.estimator.stamp is None:
                raise RuntimeError('RATE_GUARD_NO_STATE')
            self.rate_guard = ((2.5, 'ANGULAR_RECOVERY_HARD_BOUND') if recovery
                               else (1., 'ANGULAR_RATE_LIMIT'))
            self._check_rate_guard(self.estimator.stamp, time.monotonic())
            if self.fault:
                raise RuntimeError(self.fault)

    def _update_gps(self, stamp, position, attitude):
        # Called only while the sensor lock is held.
        before = self.estimator.v.copy() if not self.startup_velocity['sealed'] else None
        if before is not None:
            state_stamp = self.estimator.stamp
            predicted_position = self.estimator.p.copy()
            variance_before = np.diag(self.estimator.P)[:6].copy()
        accepted = self.estimator.update_gps(stamp, position, attitude)
        if accepted and before is not None:
            delta = self.estimator.v - before
            self.startup_velocity['gps_delta'] += delta
            self.startup_velocity['gps_updates'] += 1
            events = self.startup_velocity['gps_events']
            if len(events) < 32:
                events.append({'gps_stamp': stamp, 'state_stamp': state_stamp,
                               'gps_position': list(position),
                               'predicted_position': predicted_position.tolist(),
                               'position_innovation': (np.asarray(position) - predicted_position).tolist(),
                               'position_velocity_variance_before': variance_before.tolist(),
                               'velocity_delta': delta.tolist()})
            else:
                self.startup_velocity['gps_events_dropped'] += 1
        return accepted

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
                        if self._update_gps(stamp, position, attitude):
                            self.bootstrap_gps.append((stamp,position,attitude))
            except (ValueError, ArithmeticError) as error:
                self.estimator.reject('INVALID_GPS:'+str(error))

    def read(self):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            if not self.clock.fresh(time.monotonic()):
                raise RuntimeError('STALE_IMU')
            if not self.estimator.initialized or self.raw is None:
                raise RuntimeError('NO_INITIAL_STATE')
            raw = dict(self.raw)
            raw['last_gps_stamp'] = self.estimator.last_gps
            raw['gyro_bias_body'] = tuple(self.estimator.bg)
            raw['accel_bias_body'] = tuple(self.estimator.ba)
            state = self.estimator.snapshot()
            raw['imu_callback_arrival_wall_monotonic'] = self.clock.arrival
            raw['state_snapshot_wall_monotonic'] = time.monotonic()
            return state, raw

    def bootstrap_at_rest_before_control(self):
        """Caller must not have created a control publisher; never call in flight."""
        from estimation.stationary_bootstrap import resting_window
        with self.lock:
            if not self._bootstrap_permitted:raise RuntimeError('BOOTSTRAP_NOT_PERMITTED')
            if self.bootstrap_applied:raise RuntimeError('BOOTSTRAP_ALREADY_APPLIED')
            if self.fault or self.initial is None or not self.clock.fresh(time.monotonic()):return None
            result=resting_window(self.bootstrap_imu,self.bootstrap_gps,self.initial[1],self.clock.stamp)
            if result is None:return None
            e=self.estimator
            if not self.startup_velocity['sealed']:
                self.startup_velocity['bootstrap_delta'] -= e.v
            e.p=np.array(result['position']);e.v=np.zeros(3)
            e.q=np.array(self.initial[1],dtype=float);e.q/=np.linalg.norm(e.q)
            e.bg=np.array(result['gyro_bias']);e.ba=np.zeros(3)
            # Retain conservative GPS/attitude/bias uncertainty; averaging does
            # not establish independent measurement noise or identify accel bias.
            e.P=np.diag([.01]*3+[.01]*3+[.01]*3+[.01]*3+[.001]*3)
            e.omega=np.array(self.raw['omega_body'])-e.bg
            self.bootstrap_applied=True
            return result

    def seal_precontrol(self):
        """Permanently disable resting reset before this adapter's publisher is created."""
        with self.lock:
            self._bootstrap_permitted = False

    def seal_startup_velocity(self):
        with self.lock:
            if self.startup_velocity['sealed']:
                raise RuntimeError('STARTUP_ALREADY_SEALED')
            self.startup_velocity['velocity_at_seal'] = self.estimator.v.copy()
            self.startup_velocity['stamp_at_seal'] = self.estimator.stamp
            self.startup_velocity['sealed'] = True

    def startup_velocity_snapshot(self):
        with self.lock:
            return {key: value.tolist() if isinstance(value, np.ndarray) else value
                    for key, value in self.startup_velocity.items()}

    def startup_snapshot(self):
        with self.lock:
            return dict(self.startup_diagnostics)

    def close(self):
        for subscriber in self.subscribers:
            subscriber.unregister()
