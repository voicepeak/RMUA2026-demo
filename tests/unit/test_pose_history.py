from dataclasses import replace
import math
import unittest

import numpy as np

from estimation.geometry import delta_quaternion
from estimation.history import HistoryUnavailable, PoseHistory
from interfaces.contracts import Health, Metadata, StateEstimate


def state(stamp, epoch=0, health=Health.OK, position=None, yaw=None, velocity=(1., 0., 0.), omega=(0., 0., .2)):
    return StateEstimate(Metadata(stamp, stamp + .05, epoch, 'synthetic_eskf',
                        covariance_or_bounds=tuple(np.eye(15).flatten())),
        position or (stamp - 10., 0., -1.), velocity,
        tuple(delta_quaternion((0., 0., .2 * (stamp - 10.) if yaw is None else yaw))), omega, health)


class PoseHistoryTests(unittest.TestCase):
    def history(self, **kwargs):
        history = PoseHistory(**kwargs)
        for stamp in (10., 10.01, 10.02):
            history.append(state(stamp), 20. + stamp - 10.)
        return history

    def test_interpolates_measurement_time_without_refreshing_current_state(self):
        history = self.history()
        pose = history.lookup(10.005, 10.02, 20.021, 0)
        self.assertAlmostEqual(pose.position_ned[0], .005)
        np.testing.assert_allclose(pose.orientation_wxyz, delta_quaternion((0., 0., .001)), atol=1e-12)
        self.assertEqual(pose.bracket_stamps, (10., 10.01))
        self.assertFalse(pose.current_control_eligible)
        self.assertFalse(pose.uncertainty_calibrated)
        self.assertFalse(hasattr(pose, 'valid_until'))
        with self.assertRaises(ValueError):replace(pose,position_ned=[0.,0.,0.])
        with self.assertRaises(ValueError):replace(pose,orientation_wxyz=(0.,0.,0.,0.))

    def test_antipodal_quaternions_follow_short_arc(self):
        history = PoseHistory()
        first, second = state(10.), state(10.01)
        second = replace(second, orientation_wxyz=tuple(-np.array(second.orientation_wxyz)))
        history.append(first, 20.)
        history.append(second, 20.01)
        pose = history.lookup(10.005, 10.01, 20.011, 0)
        np.testing.assert_allclose(pose.orientation_wxyz, delta_quaternion((0., 0., .001)), atol=1e-12)

    def test_latest_wall_sensor_time_and_epoch_must_be_valid(self):
        history = self.history()
        for now, wall, epoch in ((10.1, 20.021, 0), (10.02, 20.2, 0), (10.02, 19., 0),
                                 (10.01, 20.021, 0), (10.02, 20.021, 1)):
            with self.assertRaises(HistoryUnavailable): history.lookup(10.005, now, wall, epoch)
        self.assertFalse(history.append(state(10.02), 20.1))
        with self.assertRaises(HistoryUnavailable): history.lookup(10.005, 10.02, 20.13, 0)

    def test_no_extrapolation_gap_or_bad_health_interpolation(self):
        history = self.history()
        for stamp in (9.99, 10.025):
            with self.assertRaises(HistoryUnavailable): history.lookup(stamp, 10.03, 20.021, 0)
        history = PoseHistory()
        history.append(state(10.), 20.)
        history.append(state(10.1), 20.1)
        with self.assertRaises(HistoryUnavailable): history.lookup(10.05, 10.1, 20.101, 0)
        for health in (Health.DEGRADED, Health.LOST):
            history = PoseHistory()
            history.append(state(10., health=health), 20.)
            history.append(state(10.01), 20.01)
            with self.assertRaises(HistoryUnavailable): history.lookup(10.005, 10.01, 20.011, 0)
            history.append(state(10.02, health=health), 20.02)
            with self.assertRaises(HistoryUnavailable): history.lookup(10.01, 10.02, 20.021, 0)

    def test_pose_correction_jumps_are_not_interpolated(self):
        for second in (state(10.01, position=(1., 0., -1.)), state(10.01, yaw=.3)):
            history = PoseHistory()
            history.append(state(10.), 20.)
            history.append(second, 20.01)
            with self.assertRaisesRegex(HistoryUnavailable, 'ESTIMATE_DISCONTINUITY'):
                history.lookup(10.005, 10.01, 20.011, 0)

    def test_capacity_horizon_and_explicit_reset_do_not_reuse_old_data(self):
        history = PoseHistory(capacity=4, horizon_s=.2)
        for i in range(30): history.append(state(10. + i * .01), 20. + i * .01)
        with self.assertRaises(HistoryUnavailable): history.lookup(10.01, 10.29, 20.291, 0)
        with self.assertRaises(HistoryUnavailable): history.lookup(10.25, 10.29, 20.291, 0)
        history.reset(1)
        with self.assertRaises(HistoryUnavailable): history.lookup(10.28, 10.29, 20.291, 1)
        with self.assertRaises(ValueError): history.append(state(10.3), 20.3)
        history.append(state(1., epoch=1), 21.)
        self.assertEqual(history.lookup(1., 1., 21.001, 1).epoch, 1)

    def test_source_covariance_and_frame_are_required(self):
        history = PoseHistory()
        original = state(10.)
        for metadata in (replace(original.metadata, covariance_or_bounds=()),
                         replace(original.metadata, frame_id='camera'),
                         replace(original.metadata, covariance_or_bounds=tuple((-np.eye(15)).flatten()))):
            with self.assertRaises(ValueError): history.append(replace(original, metadata=metadata), 20.)


if __name__ == '__main__':
    unittest.main()
