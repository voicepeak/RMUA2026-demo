from types import SimpleNamespace as Obj
import time
import unittest
from unittest.mock import patch
from adapters.ros_noetic.flight_sensors import FlightSensors
from interfaces.contracts import Health
from estimation.history import PoseHistory, HistoryUnavailable
from adapters.ros_noetic.sensor_fault import LocalSensorFault


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

    def test_imu_before_initial_pose_cannot_integrate_unknown_frame(self):
        s = FlightSensors(subscribe=False)
        for stamp in (100., 100.05, 100.10):
            msg = message(stamp)
            msg.linear_acceleration.x = 4.
            s.on_imu(msg)
        with self.assertRaisesRegex(RuntimeError, 'NO_INITIAL_STATE'):
            s.read()
        self.assertEqual(s.startup_snapshot()['imu_skipped_before_initial'], 3)
        s.on_initial(message(0.))
        s.on_imu(message(100.15))
        self.assertEqual(s.startup_snapshot()['first_initialized_imu_stamp'], 100.15)
        s.on_gps(message(100.15))
        state, _ = s.read()
        self.assertEqual(state.velocity_ned, (0., 0., 0.))
        self.assertEqual(state.position_ned, (1., 2., -1.))
        self.assertEqual(state.health, Health.OK)

    def test_future_gps_waits_for_imu_then_updates(self):
        s=self.sensors()
        s.on_gps(message(100.02))
        self.assertEqual(s.estimator.last_gps,100.)
        self.assertIsNotNone(s.pending_gps)
        s.on_imu(message(100.03))
        self.assertEqual(s.estimator.last_gps,100.02)
        self.assertIsNone(s.pending_gps)
        self.assertEqual(s.read()[0].health,Health.OK)

    def test_read_reports_same_accepted_imu_arrival_and_snapshot_time(self):
        s = self.sensors()
        state, raw = s.read()
        self.assertEqual(raw['stamp'], state.metadata.stamp)
        self.assertEqual(raw['imu_callback_arrival_wall_monotonic'], s.clock.arrival)
        self.assertGreaterEqual(raw['state_snapshot_wall_monotonic'], s.clock.arrival)

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

    def test_local_spike_injects_once_without_mutating_ros_message(self):
        s = self.sensors(); s.arm_rate_guard()
        s.local_injection = LocalSensorFault('IMU_RATE_SPIKE', 100.01, .1)
        msg = message(100.01); s.on_imu(msg)
        self.assertEqual(msg.angular_velocity.x, 0.)
        self.assertEqual(s.local_injection.injected_samples, 1)
        self.assertEqual(s.rate_guard_event['stamp'], 100.01)
        self.assertEqual(s.rate_guard_event['angular_velocity_body'], (2., 0., 0.))
        s.on_imu(message(100.02))
        self.assertEqual(s.local_injection.injected_samples, 1)
        with self.assertRaisesRegex(RuntimeError, 'ANGULAR_RATE_LIMIT'):
            s.read()

    def test_callback_lock_wait_does_not_refresh_old_imu_arrival(self):
        s = self.sensors()
        # The callback entered at 10.0; if lock acquisition is delayed until
        # 10.15, that delay must count against the 100 ms freshness deadline.
        with patch('adapters.ros_noetic.flight_sensors.time.monotonic', side_effect=[10., 10.15, 10.15]):
            s.on_imu(message(100.01))
            with self.assertRaisesRegex(RuntimeError, 'STALE_IMU'):
                s.read()


if __name__ == '__main__':
    unittest.main()
