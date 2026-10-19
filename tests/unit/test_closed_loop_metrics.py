import unittest
from evaluation.metrics.closed_loop import evaluate


class MetricsTests(unittest.TestCase):
    def test_temporal_interpolation_and_truth_error(self):
        truth = [{'stamp': 1+i*.01, 'position': (i*.01, 0., 0.),
                  'orientation': (1., 0., 0., 0.)} for i in range(201)]
        commands = [{'command': {'stamp': 1.005+i*.01}, 'wall_elapsed': 5+i*.01,
                     'state': {'position_ned': (.005+i*.01+.1, 0., 0.),
                               'orientation_wxyz': (1., 0., 0., 0.)},
                     'reference': (.005+i*.01, 0., 0.)} for i in range(200)]
        r = evaluate(commands, truth)
        self.assertTrue(r['evaluation_valid'])
        self.assertAlmostEqual(r['state_position_rmse_m'][0], .1)
        self.assertAlmostEqual(r['truth_tracking_rmse_m'][0], 0.)

    def test_missing_or_far_away_truth_cannot_pass(self):
        self.assertFalse(evaluate([], [])['evaluation_valid'])
        row = {'command': {'stamp': 10.}, 'wall_elapsed': 5.}
        self.assertFalse(evaluate([row], [])['evaluation_valid'])


if __name__ == '__main__':
    unittest.main()
