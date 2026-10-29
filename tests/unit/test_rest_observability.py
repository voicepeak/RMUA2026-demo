import unittest
from evaluation.diagnostics.rest_observability import experiment


class RestObservabilityTest(unittest.TestCase):
    def test_bounded_noise_and_constant_motion_are_indistinguishable(self):
        report = experiment()
        self.assertTrue(report['same_imu_and_gps_for_both_worlds'])
        self.assertTrue(report['current_window_accepted'])
        self.assertLess(report['max_static_gps_error_m'], .1)
        self.assertAlmostEqual(report['moving_world_reset_error_m_s'], .18)


if __name__ == '__main__':
    unittest.main()
