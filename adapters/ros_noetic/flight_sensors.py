"""Small shared formal-input sensor adapter; callbacks never wait for perception."""
import threading
import time
from estimation.clock import SensorClock
from estimation.imu_gps import ImuGpsEstimator
from adapters.ros_noetic.state_reader import pose_values


class FlightSensors:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import Imu
        self.lock = threading.RLock()
        self.clock = SensorClock()
        self.estimator = ImuGpsEstimator()
        self.initial = None
        self.raw = None
        self.fault = None
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
                self.raw = {'stamp': stamp, 'acceleration_body': acceleration,
                            'omega_body': omega}
            except (ValueError, ArithmeticError) as error:
                self.fault = str(error)

    def on_gps(self, msg):
        with self.lock:
            try:
                stamp = msg.header.stamp.to_sec()
                if stamp > 0 and self.clock.fresh(time.monotonic()):
                    self.estimator.update_gps(stamp, *pose_values(msg.pose))
            except (ValueError, ArithmeticError) as error:
                self.fault = str(error)

    def read(self):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            if not self.clock.fresh(time.monotonic()):
                raise RuntimeError('STALE_IMU')
            return self.estimator.snapshot(), dict(self.raw)

    def close(self):
        for subscriber in self.subscribers:
            subscriber.unregister()
