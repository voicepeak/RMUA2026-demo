from dataclasses import replace
from itertools import product
import math
import unittest

from interfaces.contracts import Metadata, Trajectory, TrajectorySample
from planning.trajectory_validator.static_sweep import SweepBounds, check_static_sweep
from world_model.occupancy.grid import Cell, Occupancy, OccupancySnapshot


def metadata(until=12., stamp=10., epoch=0, frame='world_ned'):
    return Metadata(stamp, until, epoch, 'synthetic_hand_checked_geometry', frame)


def trajectory(start=(.5, .5, .5), end=(2.5, .5, .5), acceleration=(0., 0., 0.)):
    return Trajectory(metadata(11.), tuple(TrajectorySample(t, p, (2., 0., 0.), acceleration, 0.)
        for t, p in ((10., start), (11., end))))


def grid(overrides=(), frame='world_ned'):
    cells = {key: Cell(key, Occupancy.FREE, metadata(frame=frame))
             for key in product(range(-2, 5), range(-2, 3), range(-2, 3))}
    cells.update({cell.key: cell for cell in overrides})
    return OccupancySnapshot(metadata(frame=frame), 1., tuple(cells.values()))


class StaticSweepTests(unittest.TestCase):
    bounds = SweepBounds((.1, .1, .1), 0., 0.)

    def check(self, path=None, snapshot=None, bounds=None, now=10., epoch=0):
        return check_static_sweep(path or trajectory(), snapshot or grid(), now, epoch, bounds or self.bounds)

    def test_complete_free_corridor_passes_static_geometry_only(self):
        self.assertTrue(self.check().clear)

    def test_wall_between_clear_endpoints_is_not_tunneled_through(self):
        wall = Cell((1, 0, 0), Occupancy.OCCUPIED, metadata())
        result = self.check(snapshot=grid((wall,)))
        self.assertEqual(result.reason, 'OCCUPIED_SPACE')
        self.assertEqual(result.voxel_key, (1, 0, 0))
        self.assertAlmostEqual(result.contact_time, 10.2)

    def test_unknown_missing_and_explicit_unknown_both_reject(self):
        unknown = Cell((1, 0, 0), Occupancy.UNKNOWN, metadata())
        self.assertEqual(self.check(snapshot=grid((unknown,))).reason, 'UNKNOWN_SPACE')
        empty = OccupancySnapshot(metadata(), 1., ())
        self.assertEqual(self.check(snapshot=empty).reason, 'UNKNOWN_SPACE')

    def test_box_grazing_neighbor_face_is_a_collision(self):
        wall = Cell((1, 1, 0), Occupancy.OCCUPIED, metadata())
        path = trajectory((.5, .8, .5), (2.5, .8, .5))
        self.assertFalse(self.check(path, grid((wall,)), SweepBounds((.2, .2, .2), 0., 0.)).clear)

    def test_tracking_and_between_sample_acceleration_expand_the_sweep(self):
        wall = Cell((1, 1, 0), Occupancy.OCCUPIED, metadata())
        self.assertTrue(self.check(snapshot=grid((wall,))).clear)
        self.assertFalse(self.check(snapshot=grid((wall,)), bounds=SweepBounds((.1, .1, .1), .5, 0.)).clear)
        # y=.5+2*t*(1-t) has |a|=4 and bows to y=1 at t=.5.
        self.assertFalse(self.check(snapshot=grid((wall,)), bounds=SweepBounds((.1, .1, .1), 0., 4.)).clear)

    def test_free_cell_expiry_is_checked_at_contact_not_only_at_now(self):
        expired_during_contact = Cell((1, 0, 0), Occupancy.FREE, metadata(10.5))
        self.assertEqual(self.check(snapshot=grid((expired_during_contact,))).reason, 'CELL_TIME_OR_EPOCH')
        expires_after_we_leave = Cell((0, 0, 0), Occupancy.FREE, metadata(10.31))
        self.assertTrue(self.check(snapshot=grid((expires_after_we_leave,))).clear)

    def test_cropped_suffix_keeps_the_original_chord_deviation_bound(self):
        wall = Cell((1, 1, 0), Occupancy.OCCUPIED, metadata())
        self.assertFalse(self.check(snapshot=grid((wall,)),
            bounds=SweepBounds((.1, .1, .1), 0., 4.), now=10.5).clear)

    def test_extreme_finite_coordinates_cannot_overflow_into_a_clear_result(self):
        path = trajectory((-1e308, .5, .5), (1e308, .5, .5))
        huge = replace(grid(), resolution_m=1e308)
        self.assertEqual(self.check(path, huge).reason, 'NUMERICAL_RANGE')

    def test_future_observation_wrong_epoch_and_wrong_cell_frame_reject(self):
        for meta in (metadata(stamp=10.1), metadata(epoch=1), metadata(frame='lidar')):
            result = self.check(snapshot=grid((Cell((1, 0, 0), Occupancy.FREE, meta),)))
            self.assertFalse(result.clear)

    def test_map_and_trajectory_time_and_frame_are_required(self):
        self.assertFalse(self.check(now=9.9).clear)
        self.assertFalse(self.check(epoch=1).clear)
        self.assertEqual(self.check(snapshot=grid(frame='lidar')).reason, 'WORLD_FRAME_REQUIRED')
        short_map = replace(grid(), metadata=metadata(11.))
        self.assertEqual(self.check(snapshot=short_map).reason, 'MAP_EXPIRES_BEFORE_HORIZON')

    def test_negative_coordinates_and_remaining_suffix(self):
        wall = Cell((-1, 0, 0), Occupancy.OCCUPIED, metadata())
        path = trajectory((-1.5, .5, .5), (1.5, .5, .5))
        self.assertFalse(self.check(path, grid((wall,))).clear)
        # At 10.8 the chord is past this wall; do not recheck old trajectory space.
        self.assertTrue(self.check(path, grid((wall,)), now=10.8).clear)

    def test_work_budget_and_contradictory_bounds_fail_closed(self):
        result = self.check(bounds=replace(self.bounds, max_voxel_tests=1))
        self.assertEqual(result.reason, 'VOXEL_BUDGET')
        self.assertEqual(result.voxel_tests, 0)
        self.assertEqual(self.check(trajectory(acceleration=(1., 0., 0.))).reason,
                         'ACCELERATION_BOUND_CONTRADICTION')
        with self.assertRaises(ValueError): SweepBounds((.1, .1, .1), math.nan, 0.)


if __name__ == '__main__':
    unittest.main()
