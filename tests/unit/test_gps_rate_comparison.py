import unittest
import numpy as np
from estimation.geometry import delta_quaternion
from evaluation.metrics.gps_rate_comparison import evaluate, relative_rate


class GpsRateComparisonTests(unittest.TestCase):
    def test_constant_body_rotation_and_quaternion_sign(self):
        q = delta_quaternion([0., 0., .1])
        np.testing.assert_allclose(relative_rate([1, 0, 0, 0], -q, .1), [0, 0, 1], atol=1e-12)
        gps = [{'stamp': 1., 'orientation': [1, 0, 0, 0], 'arrival_wall_monotonic': 5.},
               {'stamp': 1.1, 'orientation': q, 'arrival_wall_monotonic': 5.1}]
        imu = [{'stamp': 1.+i*.01, 'omega_body': [0, 0, 1]} for i in range(12)]
        result = evaluate(gps, imu)
        self.assertEqual(result['matched_pairs'], 1)
        self.assertLess(result['rate_error_norm_median_p95_max_rad_s'][0], 1e-10)
        self.assertFalse(result['gps_fallback_validated'])
        self.assertEqual(evaluate(gps, imu + [imu[5]])['matched_pairs'], 1)
        conflict = {'stamp': imu[5]['stamp'], 'omega_body': [0, 0, 2]}
        self.assertEqual(evaluate(gps, imu + [conflict])['matched_pairs'], 0)

    def test_missing_imu_coverage_does_not_create_a_comparison(self):
        gps = [{'stamp': 1., 'orientation': [1, 0, 0, 0]},
               {'stamp': 1.1, 'orientation': [1, 0, 0, 0]}]
        imu = [{'stamp': 1., 'omega_body': [0, 0, 0]}, {'stamp': 1.1, 'omega_body': [0, 0, 0]}]
        self.assertEqual(evaluate(gps, imu)['matched_pairs'], 0)
