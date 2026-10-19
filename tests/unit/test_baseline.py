from dataclasses import FrozenInstanceError
import math
import unittest

from interfaces.contracts import Health, Metadata, StateEstimate, Trajectory, TrajectorySample
from world_model.occupancy.grid import Cell, Occupancy, OccupancySnapshot
from planning.speed_envelope.braking import speed_limit, stopping_distance
from control.safety_supervisor.preflight import check_inputs


def metadata(stamp=1.0, until=2.0, epoch=1, frame="world_ned"):
    return Metadata(stamp, until, epoch, "test", frame, (0.01,))


def state(health=Health.OK, frame="world_ned"):
    return StateEstimate(metadata(frame=frame), (0., 0., -2.), (0., 0., 0.),
                         (1., 0., 0., 0.), (0., 0., 0.), health)


def trajectory():
    def point(stamp):
        return TrajectorySample(stamp, (0., 0., -2.), (0., 0., 0.), (0., 0., 0.), 0.)
    return Trajectory(metadata(), (point(1.), point(2.)))


class ContractTests(unittest.TestCase):
    def test_reset_and_clock_rollback_invalidate_old_data(self):
        m = metadata()
        self.assertTrue(m.current(1.5, 1))
        for now, epoch in ((0.9, 1), (2., 1), (1.5, 2), (math.nan, 1)):
            self.assertFalse(m.current(now, epoch))

    def test_nonfinite_and_mutable_metadata_rejected(self):
        with self.assertRaises(ValueError):
            Metadata(math.nan, 2., 0, "imu")
        with self.assertRaises(ValueError):
            Metadata(1., 2., 0, "imu", covariance_or_bounds=[1.])

    def test_snapshots_are_immutable(self):
        with self.assertRaises(FrozenInstanceError):
            state().position_ned = (5., 0., 0.)

    def test_trajectory_must_cover_validity_and_increase(self):
        points = trajectory().samples
        with self.assertRaises(ValueError):
            Trajectory(metadata(), tuple(reversed(points)))
        with self.assertRaises(ValueError):
            Trajectory(metadata(until=3.), points)

    def test_invalid_attitude_rejected(self):
        with self.assertRaises(ValueError):
            StateEstimate(metadata(), (0., 0., 0.), (0., 0., 0.),
                          (0., 0., 0., 0.), (0., 0., 0.), Health.OK)


class OccupancyTests(unittest.TestCase):
    def test_unobserved_space_is_unknown(self):
        grid = OccupancySnapshot(metadata(), 1., (Cell((0, 0, 0), Occupancy.FREE, metadata()),))
        self.assertEqual(grid.query((0.2, 0., 0.), 1.5, 1), Occupancy.FREE)
        self.assertEqual(grid.query((2., 0., 0.), 1.5, 1), Occupancy.UNKNOWN)
        self.assertEqual(grid.query((-0.1, 0., 0.), 1.5, 1), Occupancy.UNKNOWN)

    def test_expired_and_previous_epoch_cells_are_unknown(self):
        for cell_meta in (metadata(until=1.4), metadata(epoch=0)):
            grid = OccupancySnapshot(metadata(), 1., (Cell((0, 0, 0), Occupancy.FREE, cell_meta),))
            self.assertEqual(grid.query((0., 0., 0.), 1.5, 1), Occupancy.UNKNOWN)

    def test_occupied_is_not_free(self):
        grid = OccupancySnapshot(metadata(), 1., (Cell((0, 0, 0), Occupancy.OCCUPIED, metadata()),))
        self.assertEqual(grid.query((0., 0., 0.), 1.5, 1), Occupancy.OCCUPIED)


class BrakingTests(unittest.TestCase):
    def test_design_high_speed_example(self):
        self.assertAlmostEqual(stopping_distance(40., 0.35, 8., 0.), 114.)
        self.assertLess(speed_limit(30., 0.35, 8., 2.), 20.)

    def test_speed_bound_respects_stopping_distance(self):
        for distance in (0., 1., 5., 30., 120.):
            for delay in (0., 0.35, 2.):
                speed = speed_limit(distance, delay, 5., 2.)
                if distance > 2.:
                    self.assertAlmostEqual(stopping_distance(speed, delay, 5., 2.), distance)
                else:
                    self.assertEqual(speed, 0.)

    def test_invalid_deceleration_and_nonfinite_inputs(self):
        for deceleration in (0., -1., math.nan):
            with self.assertRaises(ValueError):
                speed_limit(30., 0.35, deceleration, 2.)


class SafetyPreconditionsTests(unittest.TestCase):
    def test_control_modes_cannot_mix(self):
        result = check_inputs(state(), trajectory(), 1.5, 1, 1, 1)
        self.assertIn("CONTROL_PUBLISHER_CONFLICT", result.reasons)
        self.assertFalse(result.prerequisites_met)

    def test_missing_or_duplicate_publishers(self):
        for count in (0, 2):
            self.assertFalse(check_inputs(state(), trajectory(), 1.5, 1, count, 0).prerequisites_met)

    def test_degraded_localization_rejected(self):
        self.assertIn("STATE_HEALTH", check_inputs(
            state(Health.DEGRADED), trajectory(), 1.5, 1, 1, 0).reasons)

    def test_frame_mismatch_rejected(self):
        self.assertIn("FRAME_MISMATCH", check_inputs(
            state(frame="camera_optical"), trajectory(), 1.5, 1, 1, 0).reasons)

    def test_reset_rejects_both_state_and_trajectory(self):
        reasons = check_inputs(state(), trajectory(), 1.5, 2, 1, 0).reasons
        self.assertIn("STATE_TIME_OR_EPOCH", reasons)
        self.assertIn("TRAJECTORY_TIME_OR_EPOCH", reasons)


if __name__ == "__main__":
    unittest.main()
