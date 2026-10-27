import unittest
import numpy as np

from adapters.ros_noetic.lidar_cloud import LidarCloud
from adapters.ros_noetic.dynamic_lidar_observer import wait_for_imu
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

    def test_wait_for_imu_is_bounded_and_causal(self):
        wall = [20.01]
        def sleep(dt): wall[0] += dt
        wait_for_imu(self.cloud, 20.01, lambda: (10., 0, True, self.state),
                     lambda: wall[0], sleep, lambda: False)
        self.assertGreaterEqual(wall[0] - 20.01, .03)
        self.assertLess(wall[0] - 20.01, .031)
        wall[0] = 20.01
        def snapshot():
            return (10.01 if wall[0] >= 20.014 else 10., 0, True, self.state)
        wait_for_imu(self.cloud, 20.01, snapshot, lambda: wall[0], sleep, lambda: False)
        self.assertLess(wall[0] - 20.01, .01)
        row = self.pipeline.process(self.cloud, 20.01, snapshot, lambda: wall[0])
        self.assertEqual(row['status'], 'PROCESSED', row)

    def test_wait_stops_for_clock_fault_or_epoch_change(self):
        for snapshot in (lambda: (10., 1, True, self.state), lambda: (10., 0, False, self.state)):
            wait_for_imu(self.cloud, 20.01, snapshot, lambda: 20.01,
                         lambda _: self.fail('unexpected wait'), lambda: False)


if __name__ == '__main__': unittest.main()
