import unittest
from evaluation.metrics.imu_receiver_comparison import evaluate


class ImuReceiverComparisonTests(unittest.TestCase):
    def test_signed_arrivals_and_ambiguous_duplicate_are_not_conflated(self):
        raw = {'stamp': 1., 'acceleration_body': [0, 0, -9.81], 'omega_body': [0, 0, 0],
               'imu_callback_arrival_wall_monotonic': 10.}
        row = {'raw': raw, 'publish_end_wall_monotonic': 10.01}
        observer = {'stamp': 1., 'acceleration_body': [0, 0, -9.81], 'omega_body': [0, 0, 0],
                    'arrival_wall_monotonic': 10.002}
        result = evaluate([row], [observer])
        self.assertEqual(result['unique_matches'], 1)
        self.assertAlmostEqual(result['controller_minus_observer_arrival_min_p50_p95_p99_max_s'][1], -.002)
        result = evaluate([row], [observer, observer])
        self.assertEqual(result['ambiguous_matches'], 1)
        self.assertIsNone(result['controller_minus_observer_arrival_min_p50_p95_p99_max_s'])

    def test_different_payload_and_reversed_time_are_not_matched(self):
        row = {'raw': {'stamp': 1., 'acceleration_body': [0, 0, 0], 'omega_body': [0, 0, 0],
                       'imu_callback_arrival_wall_monotonic': 10.}, 'publish_end_wall_monotonic': 11.}
        observer = {'stamp': 1., 'acceleration_body': [0, 0, 0], 'omega_body': [0, 0, 1],
                    'arrival_wall_monotonic': 10.}
        self.assertEqual(evaluate([row], [observer])['missing_matches'], 1)
        row['publish_end_wall_monotonic'] = 9.
        self.assertEqual(evaluate([row], [observer])['invalid_telemetry'], 1)
