from types import SimpleNamespace as Obj
import time
import unittest
from adapters.ros_noetic.flight_sensors import FlightSensors
from interfaces.contracts import Health
from estimation.history import PoseHistory, HistoryUnavailable


def message(stamp,position=(1.,2.,-1.),q=(1.,0.,0.,0.)):
    return Obj(header=Obj(stamp=Obj(to_sec=lambda:stamp)),
               pose=Obj(position=Obj(x=position[0],y=position[1],z=position[2]),
                        orientation=Obj(w=q[0],x=q[1],y=q[2],z=q[3])),
               linear_acceleration=Obj(x=0.,y=0.,z=-9.81),
               angular_velocity=Obj(x=0.,y=0.,z=0.))


class SensorAdapterTests(unittest.TestCase):
    def sensors(self):
        s=FlightSensors(subscribe=False)
        s.on_initial(message(0.))
        s.on_imu(message(100.))
        s.on_gps(message(100.))
        return s

    def test_future_gps_waits_for_imu_then_updates(self):
        s=self.sensors()
        s.on_gps(message(100.02))
        self.assertEqual(s.estimator.last_gps,100.)
        self.assertIsNotNone(s.pending_gps)
        s.on_imu(message(100.03))
        self.assertEqual(s.estimator.last_gps,100.02)
        self.assertIsNone(s.pending_gps)
        self.assertEqual(s.read()[0].health,Health.OK)

    def test_invalid_queued_gps_cannot_kill_imu_propagation(self):
        for position in ((0.,0.,0.),(1.,2.,-1.)):
            s=self.sensors()
            s.on_gps(message(100.02,position,(0.,0.,0.,0.)))
            s.on_imu(message(100.03))
            self.assertIsNone(s.fault)
            self.assertEqual(s.estimator.last_gps,100.)
            self.assertTrue(s.estimator.rejections)
            self.assertEqual(s.estimator.stamp,100.03)

    def test_optional_history_samples_imu_and_clears_on_clock_reset(self):
        history=PoseHistory()
        s=FlightSensors(subscribe=False,pose_history=history)
        s.on_initial(message(0.));s.on_imu(message(100.));s.on_gps(message(100.))
        s.on_imu(message(100.01));s.on_imu(message(100.02))
        pose=history.lookup(100.015,100.02,time.monotonic(),0)
        self.assertEqual(pose.epoch,0)
        before=history.snapshot_sources()
        s.on_imu(message(100.02))
        self.assertEqual(history.snapshot_sources(),before)
        s.on_imu(message(99.))
        self.assertEqual(s.fault,'IMU_CLOCK_RESET')
        self.assertEqual(history.epoch,1)
        self.assertEqual(history.snapshot_sources(),())
        with self.assertRaises(HistoryUnavailable):history.lookup(100.015,100.02,time.monotonic(),1)

    def test_transient_rate_over_limit_is_latched_between_control_reads(self):
        s = self.sensors()
        s.arm_rate_guard()
        spike = message(100.01); spike.angular_velocity.y = 1.1
        s.on_imu(spike)
        s.on_imu(message(100.02))
        with self.assertRaisesRegex(RuntimeError, 'ANGULAR_RATE_LIMIT'):
            s.read()
        self.assertEqual(s.rate_guard_event['stamp'], 100.01)
        self.assertAlmostEqual(s.rate_guard_event['rate_norm_rad_s'], 1.1)
        with self.assertRaisesRegex(RuntimeError, 'RATE_GUARD_ALREADY_ARMED'):
            s.arm_rate_guard()

    def test_recovery_rate_guard_uses_existing_hard_limit_and_bias(self):
        s = self.sensors(); s.estimator.bg[1] = .2
        s.arm_rate_guard(recovery=True)
        safe = message(100.01); safe.angular_velocity.y = 2.6
        s.on_imu(safe)
        self.assertIsNone(s.fault)
        spike = message(100.02); spike.angular_velocity.y = 2.8
        s.on_imu(spike)
        with self.assertRaisesRegex(RuntimeError, 'ANGULAR_RECOVERY_HARD_BOUND'):
            s.read()
        self.assertAlmostEqual(s.rate_guard_event['rate_norm_rad_s'], 2.6)

    def test_unarmed_adapter_keeps_existing_observer_behavior(self):
        s = self.sensors()
        spike = message(100.01); spike.angular_velocity.y = 3.
        s.on_imu(spike)
        self.assertIsNone(s.fault)


if __name__ == '__main__':
    unittest.main()
