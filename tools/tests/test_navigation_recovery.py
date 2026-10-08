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
    def test_residual_hover_drift_does_not_deadlock_reference_floor_recovery(self):
        for native in (False,True):
            with self.subTest(native=native):
                guard=self.guard(native);nav=LidarNavigator(guard)
                position=np.array([0.,0.,.165]);velocity=np.array([0.,0.,.00127154])
                guard.response_model.commit(np.zeros(3),velocity,.9)
                guard.response_model.stop_profile=nav._stop_profile(np.array([position,[0.,0.,-.2]]))
                nav.reference_stations=np.arange(-3.,28.,.5)
                nav.reference_heights=np.zeros(len(nav.reference_stations))
                nav.reference_coordinates=np.column_stack((nav.reference_stations,np.zeros(len(nav.reference_stations))))
                guard.command_constraint=lambda samples:nav._floor_ok(samples,
                    nav._route_stations(samples,0.,lambda s:(s,0.)),lambda s:0.,0.,timed=True)
                command,info=guard.filter_command(position,velocity,np.array([0.,0.,-.6]),
                    np.array([[20.,0.,0.]]),1.,1.02)
                self.assertEqual(info['command_reason'],'COMMAND_BRAKING')
                self.assertLess(command[2],0.)
                samples,_=guard.command_envelope(position,velocity,command,.02)
                self.assertTrue(guard.command_constraint(samples))
                self.assertLessEqual(samples[:,2].max(),position[2]+.001)
                self.assertLess(samples[-1,2],position[2]-.01)
                # Actual vehicle returns still require the complete buffer.
                command,info=guard.filter_command(position,velocity,np.array([0.,0.,-.6]),
                    np.array([[0.,0.,position[2]-.15]]),1.1,1.12)
                self.assertEqual(info['command_reason'],'COMMAND_BLOCKED')

    def test_recovery_tolerance_requires_progress_in_every_timed_branch(self):
        guard=self.guard();nav=LidarNavigator(guard)
        guard.envelope_times=np.array([0.,.3,1.,0.,.3,.6])
        self.assertTrue(nav._bound_ok(np.array([.2,.2004,.17,.2,.2004,.17]),timed=True))
        self.assertFalse(nav._bound_ok(np.array([.2,.2004,.17,.2,.2004,.2]),timed=True))
        self.assertFalse(nav._bound_ok(np.array([.2,.202,.17,.2,.2004,.17]),timed=True))
        self.assertFalse(nav._bound_ok(np.array([-.0002,.0002,-.02,-.0002,0.,-.02]),timed=True))
        self.assertFalse(nav._bound_ok(np.array([.2,.2004,.17]),timed=False))

    def test_inside_band_transient_requires_bounded_return(self):
        guard=self.guard();nav=LidarNavigator(guard)
        guard.envelope_times=np.array([0.,.3,1.,0.,.3,.6])
        overshoot=np.array([-.4,.28,-.05,-.4,.28,-.2])
        # Starts inside, bounded prefix overshoot, every scenario tail returns.
        self.assertTrue(nav._bound_ok(overshoot,timed=True,inside_transient=.35))
        # No allowance reproduces the previous strict behavior.
        self.assertFalse(nav._bound_ok(overshoot,timed=True))
        # One scenario tail must not return inside the bound.
        self.assertFalse(nav._bound_ok(np.array([-.4,.28,-.05,-.4,.28,.02]),
                                       timed=True,inside_transient=.35))
        # Overshoot beyond the allowance is rejected.
        self.assertFalse(nav._bound_ok(np.array([-.4,.5,-.05,-.4,.5,-.05]),
                                       timed=True,inside_transient=.35))
        # Untimed geometry checks keep the strict rule.
        self.assertFalse(nav._bound_ok(overshoot,timed=False,inside_transient=.35))

    def test_inside_transient_allowance_scales_with_measured_climb(self):
        guard=self.guard();nav=LidarNavigator(guard)
        self.assertAlmostEqual(nav.inside_transient_allowance(np.zeros(3),0.),.15)
        self.assertAlmostEqual(nav.inside_transient_allowance(np.array([0.,0.,-.6]),.25),.45)
        self.assertLessEqual(nav.inside_transient_allowance(np.array([0.,0.,-9.]),1.),.6)

    def guard(self, native=True, limited=True):
        guard = ExecutionGuard(reaction=.25)
        guard.response_model = VelocityResponse(
            native=native, lift_gain=.095, coupling_gains=(.075, .11, .13),
            height_gain=1.25, coupling_limited=limited, xy_error_max=4.5,
            control_periods=(.08, .16, .4))
        return guard

    def test_departure_recovery_escapes_close_floor_without_forward_tracking(self):
        guard=self.guard()
        guard.response_model.commit(np.zeros(3),np.zeros(3),.9)
        nav=LidarNavigator(guard)
        cloud=np.array([[0.,0.,.5]])
        with patch.object(nav,'path',side_effect=AssertionError('departure must not plan forward')):
            command,info=nav.select(np.zeros(3),np.zeros(3),np.zeros(3),0.,
                lambda s:(s,0.),lambda s:0.,cloud,1.,1.02,.25,recovery_only=True)
        self.assertEqual(info['command_reason'],'LIDAR_ESCAPE')
        self.assertLess(command[2],0.)
        np.testing.assert_allclose(command[:2],np.zeros(2))
        self.assertTrue(guard.recovery_command_ok(np.zeros(3),np.zeros(3),command,cloud,.02))
        obstruction=np.array([[0.,0.,-.1]])
        self.assertFalse(guard.recovery_command_ok(np.zeros(3),np.zeros(3),command,
            np.vstack((cloud,obstruction)),.02))

    def test_clear_departure_recovery_only_keeps_zero_horizontal_command(self):
        guard=self.guard();nav=LidarNavigator(guard)
        command,info=nav.select(np.zeros(3),np.zeros(3),np.array([0.,0.,-.2]),0.,
            lambda s:(s,0.),lambda s:0.,np.array([[20.,0.,0.]]),1.,1.02,
            .25,recovery_only=True)
        np.testing.assert_allclose(command[:2],np.zeros(2))
        self.assertNotEqual(info['command_reason'],'LIDAR_TRACK')

    def test_departure_from_low_spawn_restores_buffer_at_marker_height(self):
        grid=np.array([[x,y,z] for x in np.arange(-3.,3.1,.5)
                       for y in np.arange(-3.,3.1,.5) for z in (.2,-4.8)])
        for native in (False,True):
            with self.subTest(native=native):
                guard=self.guard(native);nav=LidarNavigator(guard)
                guard.response_model.commit(np.zeros(3),np.zeros(3),.9)
                command,info=nav.select(np.zeros(3),np.zeros(3),np.array([0.,0.,-.6]),0.,
                    lambda s:(s,0.),lambda s:-1.5,grid,1.,1.02,.25,recovery_only=True)
                self.assertEqual(info['command_reason'],'LIDAR_ESCAPE')
                self.assertLess(info['recovery_target'][2],-1.)
                self.assertTrue(guard.recovery_command_ok(np.zeros(3),np.zeros(3),command,grid,.02))
                # The same upward movement cannot pass a newly observed roof.
                self.assertFalse(guard.recovery_command_ok(np.zeros(3),np.zeros(3),command,
                    np.vstack((grid,[[0.,0.,-.15]])),.02))

    def test_rebound_near_ceiling_recovers_toward_road_from_large_height_error(self):
        position=np.array([0.,0.,-2.87])
        cloud=np.array([[x,y,z] for x in np.arange(-3.,3.1,.5)
                        for y in np.arange(-3.,3.1,.5) for z in (-3.32,1.68)])
        velocity=np.array([0.,0.,.00127])
        for native in (False,True):
            with self.subTest(native=native):
                guard=self.guard(native);nav=LidarNavigator(guard)
                guard.response_model.commit(np.zeros(3),velocity,.9)
                command,info=nav.select(position,velocity,np.array([3.,0.,1.2]),0.,
                    lambda s:(s,0.),lambda s:0.,cloud,1.,1.02)
                self.assertEqual(info['command_reason'],'LIDAR_ESCAPE')
                self.assertGreater(info['recovery_target'][2]-position[2],1.)
                self.assertTrue(guard.recovery_command_ok(position,velocity,command,cloud,.02))
                self.assertFalse(guard.recovery_command_ok(position,velocity,command,
                    np.vstack((cloud,position+[0.,0.,.15])),.02))

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

    def test_initial_buffer_prunes_forward_search_and_keeps_full_recovery_checks(self):
        for native in (False,True):
            with self.subTest(native=native):
                guard=self.guard(native);nav=LidarNavigator(guard)
                guard.response_model.commit(np.zeros(3),np.zeros(3),.9)
                cloud=np.array([[0.,0.,.5]])
                with patch.object(nav,'path',side_effect=AssertionError('full-buffer path is impossible')):
                    command,info=nav.select(np.zeros(3),np.zeros(3),np.array([3.,0.,0.]),0.,
                        lambda s:(s,0.),lambda s:0.,cloud,1.,1.02)
                self.assertTrue(info['initial_buffer_blocked'])
                self.assertEqual(info['candidate_count'],0)
                self.assertEqual(info['command_reason'],'LIDAR_ESCAPE')
                self.assertTrue(guard.recovery_command_ok(np.zeros(3),np.zeros(3),command,cloud,.02))
                self.assertFalse(guard.recovery_command_ok(np.zeros(3),np.zeros(3),command,
                    np.vstack((cloud,[[0.,0.,-.1]])),.02))

    def test_moving_inside_buffer_uses_braking_without_stationary_escape(self):
        guard=self.guard();nav=LidarNavigator(guard)
        velocity=np.array([2.,0.,0.])
        with patch.object(nav,'path',side_effect=AssertionError('cannot certify origin')):
            with patch.object(guard,'escape',side_effect=AssertionError('vehicle is moving')):
                command,info=nav.select(np.zeros(3),velocity,np.array([3.,0.,0.]),0.,
                    lambda s:(s,0.),lambda s:0.,np.array([[.8,0.,0.]]),1.,1.02)
        self.assertTrue(info['initial_buffer_blocked'])
        self.assertFalse(info['feasible'])
        self.assertTrue(np.all(np.isfinite(command)))
        self.assertNotEqual(info['command_reason'],'LIDAR_ESCAPE')

    def test_neutral_fallback_preserves_vertical_braking_for_both_policies(self):
        for limited, velocity in ((False, np.array([0., 0., -.06])), (True, np.array([0., 0., -.06]))):
            with self.subTest(limited=limited, velocity=velocity):
                guard = self.guard(limited=limited)
                command, info = guard.filter_command(np.zeros(3), velocity,
                    np.array([0., 0., -.01]), np.array([[0., .8, -.2]]), 1., 1.02)
                self.assertEqual(info['command_reason'], 'COMMAND_BLOCKED')
                self.assertFalse(info['blocked_neutral_settle'])
                self.assertGreater(command[2], 0.)
                self.assertLess(command[2], .8)
                self.assertTrue(info['blocked_brake'])

    def test_neutral_fallback_cannot_choose_rejected_road_constraint(self):
        guard = self.guard()
        guard.command_constraint = lambda samples: np.min(samples[:, 2]) < -.00001
        command, info = guard.filter_command(np.zeros(3), np.zeros(3),
            np.array([0., 0., -.01]), np.array([[0., .8, -.2]]), 1., 1.02)
        self.assertFalse(info['blocked_neutral_settle'])
        self.assertFalse(info['blocked_bounds_accepted'])
        np.testing.assert_allclose(command,np.zeros(3))
        self.assertEqual(info['command_reason'],'COMMAND_BLOCKED')


if __name__ == '__main__':
    unittest.main()
