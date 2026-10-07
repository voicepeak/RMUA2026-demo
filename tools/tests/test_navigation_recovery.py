import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros_ws/src/route_follower/scripts'))
from execution_guard import ExecutionGuard
from lidar_navigation import LidarNavigator
from velocity_response import VelocityResponse


class NavigationRecoveryTests(unittest.TestCase):
    def guard(self, native=True, limited=True):
        guard = ExecutionGuard(reaction=.25)
        guard.response_model = VelocityResponse(
            native=native, lift_gain=.095, coupling_gains=(.075, .11, .13),
            height_gain=1.25, coupling_limited=limited, xy_error_max=4.5,
            control_periods=(.08, .16, .4))
        return guard

    def test_fine_recovery_finds_narrow_road_and_floor_gap_with_all_stop_models(self):
        position = np.array([0., -1.85, 0.])
        # The close return blocks motion toward the route. Fore/aft returns
        # block the coarse longitudinal escape; the road blocks a 0.5m side
        # move and the departure floor blocks a 0.25m descent.
        points = np.array([[0., -.75, -.5], [1.4, -1.85, 0.], [-1.4, -1.85, 0.]])
        for native in (False, True):
            with self.subTest(native=native):
                guard = self.guard(native)
                guard.response_model.commit(np.zeros(3), np.zeros(3), .9)
                nav = LidarNavigator(guard)
                with patch.object(nav, 'path', return_value=(None, {})):
                    command, info = nav.select(position, np.zeros(3), np.array([3., 0., 0.]),
                        0., lambda s: (s, 0.), lambda s: 0., points, 1., 1.02, .15)
                self.assertEqual(info['command_reason'], 'LIDAR_ESCAPE')
                self.assertTrue(info['recovery_search_refined'])
                self.assertTrue(info['conditional_recovery'])
                self.assertEqual(len(guard.response_model.scenarios), 9)
                self.assertTrue(guard.recovery_command_ok(position, np.zeros(3), command, points, .02))
                self.assertGreaterEqual(guard.index.distance([info['recovery_target']])[0], 1.45)
                # A fresh return directly in that movement invalidates it.
                obstruction = position + .1 * command / np.linalg.norm(command)
                fresh = np.vstack((points, obstruction))
                guard.index = None
                guard._update(fresh, 1.1, 1.12, position)
                self.assertFalse(guard.recovery_command_ok(position, np.zeros(3), command, fresh, .02))

    def test_stationary_blocked_vehicle_cancels_rejected_height_input(self):
        guard = self.guard()
        cloud = np.array([[0., .8, -.2]])
        command, info = guard.filter_command(np.zeros(3), np.zeros(3),
            np.array([0., 0., -.01]), cloud, 1., 1.02)
        np.testing.assert_allclose(command, np.zeros(3))
        self.assertEqual(info['command_reason'], 'COMMAND_BLOCKED')
        self.assertTrue(info['blocked_neutral_settle'])
        self.assertLess(info['command_clearance'], 1.25)

    def test_neutral_fallback_preserves_vertical_braking_and_outbound_policy(self):
        for limited, velocity in ((False, np.zeros(3)), (True, np.array([0., 0., -.06]))):
            with self.subTest(limited=limited, velocity=velocity):
                guard = self.guard(limited=limited)
                command, info = guard.filter_command(np.zeros(3), velocity,
                    np.array([0., 0., -.01]), np.array([[0., .8, -.2]]), 1., 1.02)
                self.assertEqual(info['command_reason'], 'COMMAND_BLOCKED')
                self.assertNotIn('blocked_neutral_settle', info)
                self.assertLess(command[2], 0.)

    def test_neutral_fallback_cannot_choose_rejected_road_constraint(self):
        guard = self.guard()
        guard.command_constraint = lambda samples: np.min(samples[:, 2]) < -.00001
        command, info = guard.filter_command(np.zeros(3), np.zeros(3),
            np.array([0., 0., -.01]), np.array([[0., .8, -.2]]), 1., 1.02)
        self.assertNotIn('blocked_neutral_settle', info)
        self.assertLess(command[2], 0.)


if __name__ == '__main__':
    unittest.main()
