import unittest
from evaluation.metrics.startup_gps_counterfactual import replay, gps_velocity_components
from estimation.imu_gps import ImuGpsEstimator
import numpy as np


class StartupReplayTest(unittest.TestCase):
    def test_decomposition_matches_joint_filter_update(self):
        estimator = ImuGpsEstimator()
        estimator.initialize((1., 1., 1.), (1., 0., 0., 0.))
        estimator.propagate(1., (0., 0., -9.81), (0., 0., 0.))
        estimator.P[3, 0] = estimator.P[0, 3] = .02
        estimator.P[3, 6] = estimator.P[6, 3] = .002
        sample = {'position': [1.02, 1., 1.], 'orientation': [0.99875026, 0.04997917, 0., 0.]}
        components = gps_velocity_components(estimator, sample)
        before = estimator.v.copy()
        self.assertTrue(estimator.update_gps(1., sample['position'], sample['orientation']))
        self.assertNotEqual(components['position_velocity_delta'][0], 0.)
        self.assertNotEqual(components['attitude_velocity_delta'][0], 0.)
        np.testing.assert_allclose(estimator.v - before,
                                   np.array(components['position_velocity_delta']) + components['attitude_velocity_delta'],
                                   atol=1e-12)

    def test_replay_refuses_missing_and_duplicate_gps(self):
        summary = {'startup_diagnostics': {'first_initialized_imu_stamp': 1., 'initial_position': [0., 0., 0.],
                                           'initial_orientation_wxyz': [1., 0., 0., 0.]},
                   'startup_velocity': {'sealed': True, 'gps_events_dropped': 0, 'bootstrap_delta': [0., 0., 0.],
                                        'stamp_at_seal': 1., 'imu_updates': 1, 'gps_updates': 1,
                                        'velocity_at_seal': [0., 0., 0.],
                                        'gps_events': [{'gps_stamp': 1., 'state_stamp': 1.,
                                                        'gps_position': [1., 0., .3], 'velocity_delta': [0., 0., 0.]}]}}
        imu = [{'stamp': 1., 'acceleration_body': [0., 0., -9.81], 'omega_body': [0., 0., 0.]}]
        gps = [{'stamp': 1., 'position': [1., 0., .3], 'orientation': [1., 0., 0., 0.]}]
        result = replay(summary, imu, gps)
        self.assertTrue(result['replay_agrees_with_recorded_velocity'])
        self.assertEqual(result['gps_velocity_decomposition'][0]['component_residual'], [0., 0., 0.])
        with self.assertRaisesRegex(ValueError, 'ambiguous or missing'):
            replay(summary, imu, gps * 2)
        with self.assertRaisesRegex(ValueError, 'ambiguous or missing'):
            replay(summary, imu, [])


if __name__ == '__main__':
    unittest.main()
