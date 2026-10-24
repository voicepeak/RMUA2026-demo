from dataclasses import replace
import math
import unittest

import numpy as np

from adapters.ros_noetic.lidar_cloud import LidarCloud
from estimation.history import HistoricalPose
from interfaces.contracts import Metadata
from interfaces.contracts import StateEstimate, Health, TrajectorySample
from planning.spacetime_local.static_candidates import LocalPlannerLimits, plan_static_local
from planning.trajectory_validator.static_sweep import SweepBounds
from perception.stereo_depth.geometry import SensorPose
from world_model.occupancy.grid import Occupancy
from world_model.occupancy.rolling_endpoints import RollingEndpointMap, world_endpoints


MOUNT = SensorPose((0., 0., 0.), ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)))


def observation(stamp=10., points=((1., .1, .1),), epoch=0, position=(0., 0., 0.), quaternion=(1., 0., 0., 0.)):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    cloud = LidarCloud(Metadata(stamp, stamp + .2, epoch, 'synthetic_lidar', 'lidar'), points, len(points), 0)
    pose = HistoricalPose(stamp, epoch, position, quaternion, (stamp, stamp),
        ((0.,) * 15, (0.,) * 15), 0., 0., 0.)
    return cloud, pose


def frame(*args, **kwargs):
    return world_endpoints(*observation(*args, **kwargs), MOUNT)


class RollingEndpointTests(unittest.TestCase):
    def test_mount_translation_then_body_rotation_matches_hand_computation(self):
        cloud, pose = observation(points=((2., 0., 0.),), position=(1., 2., 3.),
            quaternion=(math.sqrt(.5), 0., 0., math.sqrt(.5)))
        mount = replace(MOUNT, translation_body_frd=(.1, .2, .3))
        world = world_endpoints(cloud, pose, mount)
        np.testing.assert_allclose(world.world_points, [[.8, 4.1, 3.3]], atol=1e-12)
        np.testing.assert_allclose(world.sensor_origin_ned, [.8, 2.1, 3.3], atol=1e-12)
        self.assertEqual(world.metadata.valid_until, cloud.metadata.valid_until)
        self.assertFalse(world.current_control_eligible)
        with self.assertRaises(ValueError): world.world_points.setflags(write=True)

    def test_pose_stamp_and_epoch_must_match_scan(self):
        cloud, pose = observation()
        for changed in (replace(pose, epoch=1), replace(pose, stamp=10.01, bracket_stamps=(10.01, 10.01))):
            with self.assertRaises(ValueError): world_endpoints(cloud, changed, MOUNT)

    def test_no_return_and_out_of_range_do_not_create_hits_or_free(self):
        value = frame(points=((0., 0., 0.), (30., 0., 0.), (31., 0., 0.)))
        self.assertEqual(value.world_points.shape, (0, 3))
        mapping = RollingEndpointMap(); mapping.integrate(value, 10., 0)
        self.assertEqual(mapping.snapshot(10., 0).cells, ())

    def test_world_voxel_floor_and_unvisited_cells_remain_unknown(self):
        mapping = RollingEndpointMap(); mapping.integrate(frame(points=((-0.01, .1, .1),)), 10., 0)
        snapshot = mapping.snapshot(10., 0)
        self.assertEqual(snapshot.metadata.frame_id, 'world_ned')
        self.assertEqual(snapshot.query((-.01, .1, .1), 10., 0), Occupancy.OCCUPIED)
        self.assertEqual(snapshot.query((.1, .1, .1), 10., 0), Occupancy.UNKNOWN)

    def test_empty_new_scan_does_not_refresh_old_hit_lease(self):
        mapping = RollingEndpointMap(); mapping.integrate(frame(), 10., 0)
        mapping.integrate(frame(10.1, points=()), 10.1, 0)
        snapshot = mapping.snapshot(10.15, 0)
        self.assertAlmostEqual(snapshot.cells[0].metadata.valid_until, 10.2)
        self.assertEqual(mapping.snapshot(10.21, 0).cells, ())

    def test_duplicate_and_older_scan_cannot_replace_or_refresh(self):
        mapping = RollingEndpointMap(); mapping.integrate(frame(), 10., 0)
        self.assertFalse(mapping.integrate(frame(points=((2., .1, .1),)), 10.1, 0))
        snapshot = mapping.snapshot(10.1, 0)
        self.assertEqual(snapshot.metadata.stamp, 10.)
        self.assertEqual(snapshot.query((2., .1, .1), 10.1, 0), Occupancy.UNKNOWN)

    def test_capacity_failure_is_atomic(self):
        mapping = RollingEndpointMap(capacity=1); mapping.integrate(frame(), 10., 0)
        with self.assertRaises(ValueError): mapping.integrate(frame(10.1, points=((2., .1, .1),)), 10.1, 0)
        snapshot = mapping.snapshot(10.1, 0)
        self.assertEqual(snapshot.metadata.stamp, 10.)
        self.assertEqual(snapshot.query((1., .1, .1), 10.1, 0), Occupancy.OCCUPIED)

    def test_future_pose_information_stale_frame_and_clock_regression_reject(self):
        value = frame()
        mapping = RollingEndpointMap()
        with self.assertRaises(ValueError): mapping.integrate(replace(value, pose_bracket_stamps=(9.99, 10.01)), 10., 0)
        with self.assertRaises(ValueError): mapping.integrate(value, 10.2, 0)
        mapping.integrate(value, 10., 0); mapping.snapshot(10.1, 0)
        with self.assertRaises(ValueError): mapping.snapshot(10.05, 0)

    def test_reset_clears_occupancy_and_old_epoch(self):
        mapping = RollingEndpointMap(); mapping.integrate(frame(), 10., 0)
        old = mapping.snapshot(10., 0); mapping.reset(1)
        self.assertEqual(mapping.snapshot(1., 1).cells, ())
        self.assertEqual(old.query((1., .1, .1), 10., 1), Occupancy.UNKNOWN)
        with self.assertRaises(ValueError): mapping.integrate(frame(), 10., 0)
        mapping.integrate(frame(1., epoch=1), 1., 1)

    def test_spatial_window_drops_to_unknown_without_free(self):
        mapping = RollingEndpointMap(window_radius_m=2.)
        mapping.integrate(frame(), 10., 0)
        mapping.integrate(frame(10.1, position=(5., 0., 0.)), 10.1, 0)
        snapshot = mapping.snapshot(10.1, 0)
        self.assertEqual(snapshot.query((1., .1, .1), 10.1, 0), Occupancy.UNKNOWN)
        self.assertTrue(all(cell.status == Occupancy.OCCUPIED for cell in snapshot.cells))

    def test_world_map_connects_to_planner_without_inventing_free(self):
        mapping = RollingEndpointMap(); mapping.integrate(frame(), 10., 0)
        snapshot = mapping.snapshot(10., 0)
        state = StateEstimate(Metadata(10., 10.15, 0, 'synthetic_state'),
            (0., 0., 0.), (0., 0., 0.), (1., 0., 0., 0.), (0., 0., 0.), Health.OK)
        initial = TrajectorySample(10., state.position_ned, state.velocity_ned, (0., 0., 0.), 0.)
        limits = LocalPlannerLimits(SweepBounds((.1, .1, .1), .02, 2.), 1.,
            segment_durations_s=(.1,), lateral_offsets_m=())
        result = plan_static_local(state, initial, (.001, 0., 0.), snapshot, 10., 0, limits)
        self.assertIsNone(result.trajectory)
        self.assertEqual(result.checks[0].reason, 'UNKNOWN_SPACE')

    def test_nonrepresentable_voxel_coordinates_reject_without_partial_update(self):
        mapping = RollingEndpointMap()
        value = replace(frame(), world_points=np.array([[1e308, 0., 0.]]))
        with np.errstate(all='raise'):
            with self.assertRaises(ValueError): mapping.integrate(value, 10., 0)
        self.assertEqual(mapping.snapshot(10., 0).cells, ())


if __name__ == '__main__':
    unittest.main()
