import unittest
from evaluation.metrics.stationary_window_truth import evaluate


class StationaryWindowTruthTest(unittest.TestCase):
    def test_motion_and_coverage_are_reported_separately(self):
        bootstrap = {'imu_window_start_stamp': 1., 'imu_window_end_stamp': 2.}
        rows = [{'stamp': 1.+i*.01, 'position': [i*.001, 0., 0.], 'orientation': [1., 0., 0., 0.]}
                for i in range(101)]
        report = evaluate(bootstrap, rows)
        self.assertTrue(report['usable'])
        self.assertAlmostEqual(report['endpoint_displacement_m'], .1)
        self.assertFalse(report['ground_contact_verified'])
        missing = evaluate(bootstrap, rows[:40]+rows[60:])
        self.assertFalse(missing['usable'])
        self.assertEqual(missing['reason'], 'TRUTH_COVERAGE_GAP')
        with self.assertRaisesRegex(ValueError, 'conflicting duplicate'):
            evaluate(bootstrap, rows + [{'stamp': 1.5, 'position': [9., 0., 0.],
                                         'orientation': [1., 0., 0., 0.]}])


if __name__ == '__main__':
    unittest.main()
