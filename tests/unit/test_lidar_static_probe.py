from dataclasses import replace
import unittest

import numpy as np

from adapters.ros_noetic.lidar_cloud import LidarCloud
from estimation.history import PoseHistory
from evaluation.pipeline.lidar_static import LidarStaticProbe
from interfaces.contracts import Health, Metadata, StateEstimate, TrajectorySample
from perception.stereo_depth.geometry import SensorPose
from planning.spacetime_local.static_candidates import LocalPlannerLimits
from planning.trajectory_validator.static_sweep import SweepBounds


class LidarStaticProbeTests(unittest.TestCase):
    def setUp(self):
        self.history = PoseHistory()
        covariance = tuple(np.eye(15).flatten())
        self.state = StateEstimate(Metadata(10., 10.15, 0, 'state', covariance_or_bounds=covariance),
            (0., 0., 0.), (0., 0., 0.), (1., 0., 0., 0.), (0., 0., 0.), Health.OK)
        self.history.append(self.state, 20.)
        self.cloud = LidarCloud(Metadata(10., 10.2, 0, 'scan', 'lidar'),
            np.array([[1., .1, .1]]), 1, 0)
        self.ref = TrajectorySample(10., (0., 0., 0.), (0., 0., 0.), (0., 0., 0.), 0.)
        mount = SensorPose((0., 0., 0.), ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)))
        limits = LocalPlannerLimits(SweepBounds((.1, .1, .1), .02, 2.), 1.,
            segment_durations_s=(.1,), lateral_offsets_m=())
        self.probe = LidarStaticProbe(self.history, mount, limits)

    def run_probe(self, cloud=None, state=None, now=10., wall=20., epoch=0):
        return self.probe.evaluate(cloud or self.cloud, state or self.state, self.ref,
                                   (.001, 0., 0.), now, wall, epoch)

    def test_sensor_to_map_to_planner_fails_closed_on_unknown(self):
        result = self.run_probe()
        self.assertEqual(result.reason, 'NO_STATIC_CANDIDATE')
        self.assertEqual(result.map_cells, 1)
        self.assertEqual(result.candidate_reasons, ('UNKNOWN_SPACE',))
        self.assertFalse(result.control_eligible)
        self.assertFalse(hasattr(result, 'trajectory'))
        self.assertEqual(self.run_probe().reason, 'SCAN_DUPLICATE')

    def test_stale_scan_and_state_rejected_before_map_update(self):
        self.assertEqual(self.run_probe(now=10.2).reason, 'SCAN_UNAVAILABLE')
        other = replace(self.state, position_ned=(1., 0., 0.))
        self.assertEqual(self.run_probe(state=other).reason, 'STATE_UNAVAILABLE')
        self.assertEqual(self.probe.map.snapshot(10., 0).cells, ())

    def test_stale_arrival_and_missing_pose_do_not_map(self):
        self.assertEqual(self.run_probe(wall=20.11).reason, 'POSE_STALE_STATE_ARRIVAL')
        cloud = replace(self.cloud, metadata=replace(self.cloud.metadata, stamp=9.99))
        self.assertEqual(self.run_probe(cloud=cloud).reason, 'POSE_NO_BRACKET')
        self.assertEqual(self.probe.map.snapshot(10., 0).cells, ())

    def test_epoch_reset_requires_history_reset(self):
        with self.assertRaises(ValueError): self.probe.reset(1)
        self.history.reset(1)
        self.probe.reset(1)
        self.assertEqual(self.run_probe().reason, 'EPOCH_MISMATCH')
        self.assertEqual(self.probe.map.snapshot(1., 1).cells, ())

    def test_capacity_failure_does_not_commit_partial_scan(self):
        self.probe = LidarStaticProbe(self.history, self.probe.sensor_pose, self.probe.limits, capacity=1)
        cloud = LidarCloud(self.cloud.metadata, np.array([[1., .1, .1], [2., .1, .1]]), 2, 0)
        self.assertEqual(self.run_probe(cloud=cloud).reason, 'MAP_REJECTED')
        self.assertEqual(self.probe.map.snapshot(10., 0).cells, ())


if __name__ == '__main__':
    unittest.main()
