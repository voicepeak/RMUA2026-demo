from pathlib import Path
import tempfile
import unittest
from control.control_authority.lease import ControlLease, check_graph
from control.pwm_publisher.guard import PwmCommand, CommandGuard
from estimation.clock import SensorClock


class GuardTests(unittest.TestCase):
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
        self.assertTrue(guard.accept(PwmCommand(10., 0, 0, (.2,)*4), clock, 1.))
        self.assertFalse(guard.accept(PwmCommand(10., 0, 1, (.2,)*4), clock, 1.005))
        self.assertTrue(guard.accept(PwmCommand(10., 0, 2, (.2,)*4), clock, 1.02))
        with self.assertRaisesRegex(RuntimeError, 'COMMAND_REPLAY'):
            guard.accept(PwmCommand(10., 0, 2, (.2,)*4), clock, 1.04)
        with self.assertRaisesRegex(RuntimeError, 'COMMAND_REPLAY'):
            guard.accept(PwmCommand(10., 0, 3, (.2,)*4), clock, 1.06)

    def test_missing_imu_and_epoch_change_fail_closed(self):
        for reset in (False, True):
            c = SensorClock()
            c.observe(10., 1.)
            g = CommandGuard(0)
            if reset:
                c.observe(2., 1.02)
            with self.assertRaises(RuntimeError):
                g.accept(PwmCommand(10., 0, 1, (.2,)*4), c, 1.2)

    def test_invalid_pwm_is_rejected(self):
        for values in ((-.1,)*4, (1.1,)*4, (float('nan'),)*4, (.1,)*3):
            with self.assertRaises(ValueError):
                PwmCommand(10., 0, 0, values)

    def test_blocked_publication_cannot_be_followed_by_a_burst(self):
        c=SensorClock();c.observe(10.,1.)
        g=CommandGuard(0)
        self.assertTrue(g.accept(PwmCommand(10.,0,0,(.2,)*4),c,1.))
        g.published(1.02)
        self.assertFalse(g.accept(PwmCommand(10.,0,1,(.2,)*4),c,1.025))
        self.assertTrue(g.accept(PwmCommand(10.,0,2,(.2,)*4),c,1.04))


if __name__ == '__main__':
    unittest.main()
