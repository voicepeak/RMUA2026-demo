import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from evaluation.pipeline.flight_stereo_motion import evaluate


class FlightStereoEvaluationTests(unittest.TestCase):
    def test_truth_comparison_interpolates_and_preserves_rejections(self):
        estimates = {'source': 'offline', 'results': [
            {'previous': 'a', 'current': 'b', 'previous_stamp': 1.01, 'current_stamp': 1.03,
             'status': 'ESTIMATED', 'motion': {'translation_previous_body_m': [.02, 0., 0.],
             'rotation_previous_from_current_body': np.eye(3).tolist()}},
            {'previous': 'b', 'current': 'c', 'status': 'REJECTED', 'reason': 'bad fit'}]}
        records = [{'stamp': t, 'position': [t - 1, 0., 0.], 'orientation': [1., 0., 0., 0.]}
                   for t in (1., 1.02, 1.04)]
        with tempfile.TemporaryDirectory() as tmp:
            truth = Path(tmp) / 'truth.jsonl'
            truth.write_text(''.join(json.dumps(row) + '\n' for row in records))
            output = evaluate(estimates, truth)
        self.assertEqual(output['status_counts'], {'COMPARED': 1, 'REJECTED': 1})
        self.assertLess(output['results'][0]['translation_error_m'], 1e-10)
        self.assertLess(output['results'][0]['rotation_error_rad'], 1e-10)
        self.assertEqual(output['results'][1]['reason'], 'bad fit')

    def test_no_truth_extrapolation_or_interpolation_across_large_gap(self):
        estimates = {'source': 'offline', 'results': [
            {'previous': 'a', 'current': 'b', 'previous_stamp': 1.01, 'current_stamp': 1.5,
             'status': 'ESTIMATED', 'motion': {'translation_previous_body_m': [0., 0., 0.],
             'rotation_previous_from_current_body': np.eye(3).tolist()}}]}
        with tempfile.TemporaryDirectory() as tmp:
            truth = Path(tmp) / 'truth.jsonl'
            truth.write_text(''.join(json.dumps({'stamp': t, 'position': [0., 0., 0.],
                'orientation': [1., 0., 0., 0.]}) + '\n' for t in (1., 1.02, 2.)))
            output = evaluate(estimates, truth)
        self.assertEqual(output['status_counts'], {'TRUTH_UNAVAILABLE': 1})


if __name__ == '__main__':
    unittest.main()
