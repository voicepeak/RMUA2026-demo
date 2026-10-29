import unittest
from evaluation.metrics.imu_control_latency import evaluate


class ImuControlLatencyTests(unittest.TestCase):
    def test_intervals_are_local_and_missing_or_reversed_timing_is_rejected(self):
        row = {'raw': {'imu_callback_arrival_wall_monotonic': 1., 'state_snapshot_wall_monotonic': 1.01},
               'publish_begin_wall_monotonic': 1.02, 'publish_end_wall_monotonic': 1.03}
        result = evaluate([row])
        self.assertTrue(result['timing_complete'])
        self.assertAlmostEqual(result['arrival_to_publish_end_p50_p95_p99_max_s'][0], .03)
        bad = dict(row, publish_end_wall_monotonic=.9)
        result = evaluate([row, bad, {}])
        self.assertEqual((result['valid_samples'], result['invalid_samples'], result['missing_samples']), (1, 1, 1))
        self.assertFalse(result['timing_complete'])
