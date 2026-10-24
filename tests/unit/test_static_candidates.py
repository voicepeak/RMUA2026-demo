from dataclasses import replace
from itertools import product
import math
import unittest

from interfaces.contracts import Health, Metadata, StateEstimate, TrajectorySample
from planning.spacetime_local.static_candidates import LocalPlannerLimits, QuinticReference, plan_static_local
from planning.trajectory_validator.static_sweep import SweepBounds
from world_model.occupancy.grid import Cell, Occupancy, OccupancySnapshot


def scene(wall=False, empty=False):
    meta = Metadata(10., 30., 0, 'synthetic_static_world')
    cells = tuple(Cell(key, Occupancy.OCCUPIED if wall and key == (1, 0, 0) else Occupancy.FREE, meta)
                  for key in product(range(-2, 6), range(-4, 6), range(-1, 3)))
    grid = OccupancySnapshot(meta, 1., () if empty else cells)
    state = StateEstimate(replace(meta, valid_until=10.2), (.5, .5, .5), (0., 0., 0.),
        (1., 0., 0., 0.), (0., 0., 0.), Health.OK)
    initial = TrajectorySample(10., state.position_ned, state.velocity_ned, (0., 0., 0.), 0.)
    limits = LocalPlannerLimits(SweepBounds((.1, .1, .1), .02, 2.), 1.2)
    return state, initial, grid, limits


def plan(wall=False, empty=False, changes=None):
    state, initial, grid, limits = scene(wall, empty)
    values = {'state': state, 'initial_reference': initial, 'snapshot': grid, 'limits': limits,
              'goal': (2.5, .5, .5), 'now': 10., 'epoch': 0}
    values.update(changes or {})
    return plan_static_local(**values)


class StaticCandidateTests(unittest.TestCase):
    def test_quintic_preserves_initial_pva_and_stops_at_target(self):
        initial = TrajectorySample(10., (.2, -.3, .5), (.3, .1, -.2), (.1, -.2, .3), .4)
        curve = QuinticReference.to_rest(initial, (2., 1., .7), 2.)
        start, end = curve.sample(10.), curve.sample(12.)
        for name in ('position_ned', 'velocity_ned', 'acceleration_ned'):
            for actual, expected in zip(getattr(start, name), getattr(initial, name)):
                self.assertAlmostEqual(actual, expected, places=13)
        self.assertEqual(end.position_ned, (2., 1., .7))
        self.assertEqual(end.velocity_ned, (0., 0., 0.))
        self.assertEqual(end.acceleration_ned, (0., 0., 0.))

    def test_large_sensor_timestamp_keeps_endpoint_constraints(self):
        for duration in (.1, .3):
            initial = TrajectorySample(1792824811.1234, (0., 0., 0.), (.2, 0., 0.), (.1, 0., 0.), 0.)
            curve = QuinticReference.to_rest(initial, (.01, 0., 0.), duration)
            self.assertAlmostEqual(curve.sample(curve.stamp).velocity_ned[0], .2)
            self.assertEqual(curve.sample(curve.end_stamp).position_ned, (.01, 0., 0.))
            self.assertEqual(curve.sample(curve.end_stamp).acceleration_ned, (0., 0., 0.))

    def test_continuous_derivative_bounds_enclose_dense_samples(self):
        initial = TrajectorySample(10., (0., 0., 0.), (.2, -.1, 0.), (.1, .1, 0.), 0.)
        curve = QuinticReference.to_rest(initial, (2., 1., -.5), 3.)
        speed, acceleration = curve.derivative_bounds()
        for i in range(501):
            value = curve.sample(curve.stamp + i * 3. / 500)
            self.assertLessEqual(math.hypot(*value.velocity_ned), speed + 1e-12)
            self.assertLessEqual(math.hypot(*value.acceleration_ned), acceleration + 1e-12)

    def test_direct_free_route_is_selected_and_output_lease_is_source_bounded(self):
        result = plan()
        self.assertEqual(result.reason, 'STATIC_CANDIDATE')
        self.assertEqual(len(result.pieces), 1)
        self.assertLessEqual(result.trajectory.metadata.valid_until, result.source_state.valid_until)
        self.assertEqual(result.source_state.stamp, 10.)
        self.assertGreater(result.reference_at(10.04, 0).position_ned[0], .5)
        for stamp, epoch in ((result.trajectory.metadata.valid_until, 0), (10.04, 1)):
            with self.assertRaises(ValueError): result.reference_at(stamp, epoch)

    def test_wall_forces_a_checked_detour_with_continuous_join(self):
        result = plan(wall=True)
        self.assertIsNotNone(result.trajectory)
        self.assertEqual(len(result.pieces), 2)
        self.assertGreater(result.pieces[0].controls[-1][1], 1.5)
        before = result.pieces[0].sample(result.pieces[0].end_stamp)
        after = result.pieces[1].sample(result.pieces[1].stamp)
        self.assertEqual(before, after)
        self.assertTrue(any(c.reason == 'OCCUPIED_SPACE' for c in result.checks))

    def test_nonzero_initial_velocity_and_acceleration_are_preserved_in_a_plan(self):
        state, initial, _, _ = scene()
        state = replace(state, velocity_ned=(.2, 0., 0.))
        initial = replace(initial, velocity_ned=state.velocity_ned, acceleration_ned=(.1, 0., 0.))
        result = plan(changes={'state': state, 'initial_reference': initial})
        self.assertIsNotNone(result.trajectory)
        start = result.reference_at(10., 0)
        self.assertAlmostEqual(start.velocity_ned[0], .2, places=13)
        self.assertAlmostEqual(start.acceleration_ned[0], .1, places=13)

    def test_explicit_stationary_goal_can_use_a_zero_acceleration_bound(self):
        _, initial, _, limits = scene()
        limits = replace(limits, sweep=replace(limits.sweep, acceleration_bound_m_s2=0.), segment_durations_s=(.1,))
        result = plan(changes={'goal': initial.position_ned, 'limits': limits})
        self.assertIsNotNone(result.trajectory)
        self.assertEqual(result.reference_at(10.04, 0).velocity_ned, (0., 0., 0.))

    def test_unknown_map_does_not_create_motion_or_a_fake_hold(self):
        result = plan(empty=True)
        self.assertIsNone(result.trajectory)
        self.assertEqual(result.reason, 'NO_STATIC_CANDIDATE')
        with self.assertRaises(ValueError): result.reference_at(10., 0)

    def test_expired_unhealthy_future_or_other_epoch_state_rejects(self):
        state, _, _, _ = scene()
        states = [replace(state, health=Health.LOST),
                  replace(state, metadata=replace(state.metadata, valid_until=10.)),
                  replace(state, metadata=replace(state.metadata, stamp=10.1)),
                  replace(state, metadata=replace(state.metadata, epoch=1))]
        for value in states:
            self.assertEqual(plan(changes={'state': value}).reason, 'STATE_UNAVAILABLE')

    def test_reference_position_velocity_or_time_mismatch_is_not_retimestamped(self):
        _, initial, _, _ = scene()
        for value in (replace(initial, stamp=9.99), replace(initial, stamp=10. + 1e-7),
                      replace(initial, stamp=10. - 1e-7), replace(initial, position_ned=(1., .5, .5)),
                      replace(initial, velocity_ned=(.3, 0., 0.))):
            self.assertEqual(plan(changes={'initial_reference': value}).reason, 'REFERENCE_START_MISMATCH')

    def test_map_must_cover_full_stopping_reference_not_just_short_lease(self):
        _, _, grid, _ = scene()
        short = replace(grid, metadata=replace(grid.metadata, valid_until=10.5))
        result = plan(changes={'snapshot': short})
        self.assertIsNone(result.trajectory)
        self.assertTrue(all(c.reason == 'MAP_STOP_HORIZON' for c in result.checks))

    def test_frame_and_map_epoch_do_not_cross(self):
        _, _, grid, _ = scene()
        for meta in (replace(grid.metadata, frame_id='lidar'), replace(grid.metadata, epoch=1)):
            self.assertIsNone(plan(changes={'snapshot': replace(grid, metadata=meta)}).trajectory)

    def test_kinematic_and_sample_budgets_are_enforced(self):
        _, _, _, limits = scene()
        for value, reason in ((replace(limits, speed_limit_m_s=.01), 'KINEMATIC_LIMIT'),
            (replace(limits, sweep=replace(limits.sweep, max_segments=1)), 'SAMPLE_BUDGET'),
            (replace(limits, sweep=replace(limits.sweep, max_voxel_tests=1)), 'VOXEL_BUDGET')):
            result = plan(changes={'limits': value})
            self.assertIsNone(result.trajectory)
            self.assertTrue(any(c.reason == reason for c in result.checks))

    def test_attempt_budget_never_returns_an_unchecked_route(self):
        _, _, _, limits = scene()
        result = plan(wall=True, changes={'limits': replace(limits, max_attempts=1)})
        self.assertIsNone(result.trajectory)
        self.assertEqual(len(result.checks), 1)

    def test_invalid_configuration_and_queries_are_rejected(self):
        _, initial, _, limits = scene()
        with self.assertRaises(ValueError): replace(limits, segment_durations_s=())
        with self.assertRaises(ValueError): replace(limits, lease_s=math.inf)
        curve = QuinticReference.to_rest(initial, (2., 0., 0.), 2.)
        with self.assertRaises(ValueError): curve.sample(9.99)


if __name__ == '__main__':
    unittest.main()
