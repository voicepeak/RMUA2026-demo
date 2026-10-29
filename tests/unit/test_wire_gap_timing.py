import unittest

from evaluation.metrics.wire_gap_timing import analyze


class WireGapTimingTest(unittest.TestCase):
    def test_missing_sequence_uses_audit_offset_and_other_observer(self):
        telemetry = [{'command': {'sequence': i}, 'publish_begin_wall_monotonic': 1 + i * .01,
                      'publish_end_wall_monotonic': 1 + i * .01 + .001} for i in range(3)]
        def row(seq, arrival):
            return {'data': {'header': {'seq': seq + 10}}, 'arrival_wall_monotonic': arrival}
        wires = [[row(0, 1.002), row(2, 1.022)],
                 [row(0, 1.002), row(1, 1.016), row(2, 1.022)]]
        def audit(missing, count):
            return {'ros_header_sequence_offset': 10, 'invalid_records': 0, 'duplicate_sequences': 0,
                    'mismatched_sequences': [], 'unexpected_sequences': [], 'sender_commands': 3,
                    'wire_records': count, 'missing_sequences': missing}
        report = analyze(telemetry, wires, [audit([1], 2), audit([], 3)])
        window = report['missing_windows'][0]
        self.assertEqual(window['logical_sequence'], 1)
        self.assertAlmostEqual(window['other_observer_arrival_ms'], 6)
        self.assertAlmostEqual(window['next_publish_begin_ms'], 10)
        bad = audit([], 2)
        with self.assertRaises(ValueError):
            analyze(telemetry, wires, [bad, audit([], 3)])


if __name__ == '__main__':
    unittest.main()
