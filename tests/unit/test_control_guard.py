from pathlib import Path
import tempfile
import unittest
from control.control_authority.lease import ControlLease, check_graph
from control.pwm_publisher.guard import PwmCommand, CommandGuard
from estimation.clock import SensorClock
from adapters.ros_noetic.pwm_experiment import run_experiment


class GuardTests(unittest.TestCase):
    def test_control_stall_is_restricted_before_ros_start(self):
        for options in ({'control_stall_seconds':.3},
                        {'control_stall_stage':'before-publish'},
                        {'control_stall_seconds':.15, 'hover_seconds':0},
                        {'control_stall_seconds':.15, 'hover_seconds':14, 'sensor_fault':'IMU_DROP'},
                        {'control_stall_seconds':.15, 'hover_seconds':14, 'step':True}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                run_experiment(Path('/unused'), (.2,), .3, **options)
    def test_conflicting_publishers_and_mode_change_are_rejected(self):
        topic = '/airsim_node/drone_1/rotor_pwm_cmd'
        check_graph({topic: ['/self']}, '/self')
        with self.assertRaises(RuntimeError):
            check_graph({topic: ['/self', '/other']}, '/self')
        with self.assertRaises(RuntimeError):
            check_graph({topic: ['/self'], '/airsim_node/drone_1/vel_body_cmd': ['/vel']}, '/self')
        with tempfile.TemporaryDirectory() as d:
            with ControlLease('scene1', 'PWM', Path(d)):
                with self.assertRaises(BlockingIOError):
                    with ControlLease('scene1', 'PWM', Path(d)):
                        pass
            with self.assertRaises(RuntimeError):
                with ControlLease('scene1', 'VEL', Path(d)):
                    pass
            with ControlLease('scene2', 'VEL', Path(d)):
                pass

    def test_rate_limit_and_replay_latch(self):
        clock = SensorClock()
        clock.observe(10., 1.)
        guard = CommandGuard(0)
        self.assertTrue(guard.accept(PwmCommand(10., 0, 0, (.2,)*4), clock, 1., 1.))
        self.assertFalse(guard.accept(PwmCommand(10., 0, 1, (.2,)*4), clock, 1.005, 1.))
        self.assertTrue(guard.accept(PwmCommand(10., 0, 2, (.2,)*4), clock, 1.02, 1.))
        with self.assertRaisesRegex(RuntimeError, 'COMMAND_REPLAY'):
            guard.accept(PwmCommand(10., 0, 2, (.2,)*4), clock, 1.04, 1.)
        with self.assertRaisesRegex(RuntimeError, 'COMMAND_REPLAY'):
            guard.accept(PwmCommand(10., 0, 3, (.2,)*4), clock, 1.06, 1.)

    def test_missing_imu_and_epoch_change_fail_closed(self):
        for reset in (False, True):
            c = SensorClock()
            c.observe(10., 1.)
            g = CommandGuard(0)
            if reset:
                c.observe(2., 1.02)
            with self.assertRaises(RuntimeError):
                g.accept(PwmCommand(10., 0, 1, (.2,)*4), c, 1.2, 1.)

    def test_new_imu_does_not_refresh_an_old_command_source(self):
        clock=SensorClock(); clock.observe(10.,1.); clock.observe(10.01,1.2)
        guard=CommandGuard(0); old=PwmCommand(10.,0,0,(.2,)*4)
        with self.assertRaisesRegex(RuntimeError, 'STALE_IMU_OR_COMMAND'):
            guard.accept(old, clock, 1.2, 1.)
        with self.assertRaisesRegex(RuntimeError, 'STALE_IMU_OR_COMMAND'):
            guard.accept(PwmCommand(10.01,0,1,(.2,)*4), clock, 1.2, 1.2)

    def test_publish_recheck_rejects_stall_after_acceptance(self):
        clock=SensorClock(); clock.observe(10.,1.)
        guard=CommandGuard(0); command=PwmCommand(10.,0,0,(.2,)*4)
        self.assertTrue(guard.accept(command,clock,1.,1.))
        clock.observe(10.01,1.15)
        with self.assertRaisesRegex(RuntimeError, 'STALE_IMU_OR_COMMAND'):
            guard.before_publish(command,clock,1.15,1.)
        with self.assertRaisesRegex(RuntimeError, 'STALE_IMU_OR_COMMAND'):
            guard.accept(PwmCommand(10.01,0,1,(.2,)*4),clock,1.15,1.15)

    def test_publish_recheck_requires_the_exact_accepted_command(self):
        clock=SensorClock(); clock.observe(10.,1.)
        command=PwmCommand(10.,0,0,(.2,)*4); guard=CommandGuard(0)
        self.assertTrue(guard.accept(command,clock,1.,1.))
        guard.before_publish(command,clock,1.001,1.)
        with self.assertRaisesRegex(RuntimeError, 'COMMAND_NOT_ACCEPTED'):
            guard.before_publish(command,clock,1.002,1.)

    def test_fresh_source_and_invalid_wall_times(self):
        clock=SensorClock(); clock.observe(10.,1.); clock.observe(10.01,1.05)
        self.assertTrue(CommandGuard(0).accept(PwmCommand(10.,0,0,(.2,)*4), clock, 1.08, 1.))
        for source in (None, float('nan'), .9, 1.06, 1.2):
            with self.subTest(source=source), self.assertRaisesRegex(RuntimeError, 'STALE_IMU_OR_COMMAND'):
                CommandGuard(0).accept(PwmCommand(10.,0,0,(.2,)*4), clock, 1.08, source)

    def test_invalid_pwm_is_rejected(self):
        for values in ((-.1,)*4, (1.1,)*4, (float('nan'),)*4, (.1,)*3):
            with self.assertRaises(ValueError):
                PwmCommand(10., 0, 0, values)

    def test_blocked_publication_cannot_be_followed_by_a_burst(self):
        c=SensorClock();c.observe(10.,1.)
        g=CommandGuard(0)
        self.assertTrue(g.accept(PwmCommand(10.,0,0,(.2,)*4),c,1.,1.))
        g.published(1.02)
        self.assertFalse(g.accept(PwmCommand(10.,0,1,(.2,)*4),c,1.025,1.))
        self.assertTrue(g.accept(PwmCommand(10.,0,2,(.2,)*4),c,1.04,1.))


if __name__ == '__main__':
    unittest.main()
