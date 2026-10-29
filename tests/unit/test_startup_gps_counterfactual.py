import unittest
from evaluation.metrics.startup_gps_counterfactual import replay


class StartupReplayTest(unittest.TestCase):
    def test_replay_refuses_missing_and_duplicate_gps(self):
        summary = {'startup_diagnostics': {'first_initialized_imu_stamp': 1., 'initial_position': [0., 0., 0.],
                                           'initial_orientation_wxyz': [1., 0., 0., 0.]},
                   'startup_velocity': {'sealed': True, 'gps_events_dropped': 0, 'bootstrap_delta': [0., 0., 0.],
                                        'stamp_at_seal': 1., 'imu_updates': 1, 'gps_updates': 1,
                                        'velocity_at_seal': [0., 0., 0.],
                                        'gps_events': [{'gps_stamp': 1., 'state_stamp': 1.,
                                                        'gps_position': [1., 0., .3]}]}}
        imu = [{'stamp': 1., 'acceleration_body': [0., 0., -9.81], 'omega_body': [0., 0., 0.]}]
        gps = [{'stamp': 1., 'position': [1., 0., .3], 'orientation': [1., 0., 0., 0.]}]
        result = replay(summary, imu, gps)
        self.assertTrue(result['replay_agrees_with_recorded_velocity'])
        with self.assertRaisesRegex(ValueError, 'ambiguous or missing'):
            replay(summary, imu, gps * 2)
        with self.assertRaisesRegex(ValueError, 'ambiguous or missing'):
            replay(summary, imu, [])


if __name__ == '__main__':
    unittest.main()
