import unittest
import numpy as np

from adapters.ros_noetic.lidar_cloud import LidarCloud
from estimation.history import PoseHistory
from interfaces.contracts import Health, Metadata, StateEstimate
from perception.object_motion.online import OnlineDynamicLidar
from perception.stereo_depth.geometry import SensorPose


class OnlineDynamicLidarTests(unittest.TestCase):
    def setUp(self):
        self.history = PoseHistory()
        for stamp in (10., 10.01):
            self.state = StateEstimate(Metadata(stamp, stamp + .05, 0, 'test',
                covariance_or_bounds=tuple(np.eye(15).flatten())), (0., 0., 0.), (0., 0., 0.),
                (1., 0., 0., 0.), (0., 0., 0.), Health.OK)
            self.history.append(self.state, 20. + stamp - 10.)
        mount = SensorPose((0., 0., 0.), ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)))
        self.pipeline = OnlineDynamicLidar(self.history, mount)
        points = np.array([(5. + .01*i, .1, .1) for i in range(10)])
        self.cloud = LidarCloud(Metadata(10.005, 10.205, 0, 'test', 'lidar'), points, 10, 0)

    def test_live_candidate_is_diagnostic_only(self):
        row = self.pipeline.process(self.cloud, 20.01, lambda: (10.01, 0, True, self.state), lambda: 20.011)
        self.assertEqual(row['status'], 'PROCESSED', row)
        self.assertGreater(len(row['tracks']), 0)
        self.assertFalse(row['dynamic_check']['safety_certified'])
        self.assertFalse(row['control_eligible'])

    def test_expired_source_is_rejected_before_tracking(self):
        row = self.pipeline.process(self.cloud, 20.01, lambda: (10.21, 0, True, self.state), lambda: 20.02)
        self.assertEqual(row['reason'], 'INPUT_SOURCE_LEASE_NOT_CURRENT')
        self.assertEqual(self.pipeline.tracker.snapshot(10.21, 0), ())

    def test_expiry_during_clustering_rejects_without_committing(self):
        snapshots = iter([(10.01, 0, True, self.state), (10.21, 0, True, self.state)])
        row = self.pipeline.process(self.cloud, 20.01, lambda: next(snapshots), lambda: 20.02)
        self.assertEqual(row['reason'], 'AFTER_CLUSTER_SOURCE_LEASE_NOT_CURRENT')
        self.assertEqual(self.pipeline.tracker.snapshot(10.21, 0), ())


if __name__ == '__main__': unittest.main()
