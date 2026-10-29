import unittest
from evaluation.metrics.gps_truth_comparison import evaluate


class GpsTruthComparisonTests(unittest.TestCase):
    def test_nearest_time_and_quaternion_sign_invariance(self):
        truth = [{'stamp': 1., 'position': [0, 0, 0], 'orientation': [1, 0, 0, 0]}]
        gps = [{'stamp': 1.001, 'position': [0.1, 0, 0], 'orientation': [-1, 0, 0, 0],
                'arrival_wall_monotonic': 2.},
               {'stamp': 1.1, 'position': [0, 0, 0], 'orientation': [1, 0, 0, 0],
                'arrival_wall_monotonic': 2.1}]
        result = evaluate(gps, truth, failure_wall=2.05)
        self.assertEqual(result['matched_samples'], 1)
        self.assertEqual(result['unmatched_samples'], 1)
        self.assertAlmostEqual(result['position_error_median_max_m'][0], .1)
        self.assertAlmostEqual(result['orientation_error_median_max_rad'][0], 0.)
        self.assertEqual(result['gps_after_fault_arrival_count'], 1)
        self.assertFalse(result['gps_fallback_validated'])

    def test_zero_quaternion_cannot_be_counted(self):
        row = {'stamp': 1., 'position': [0, 0, 0], 'orientation': [0, 0, 0, 0],
               'arrival_wall_monotonic': 2.}
        self.assertEqual(evaluate([row], [row])['matched_samples'], 0)
