import unittest

from evaluation.metrics.bridge_lifetime import evaluate


def row(event, start, end=None, span=1, parameter='0x123', command='0x456', thread=1):
    return {'event': event, 'mode': 'lifetime', 'start_monotonic_ns': start,
            'end_monotonic_ns': start if end is None else end, 'bridge_span': span,
            'parameter_address': parameter, 'command_address': command,
            'thread_id': thread, 'write_errors': 0}


class BridgeLifetimeAuditTests(unittest.TestCase):
    def records(self, free_start=25, free_end=26):
        return [row('bridge_begin', 10), row('parameter_borrow', 20, 22),
                row('command_constructed', 30, 32), row('command_handoff', 40, 42),
                row('bridge_end', 50), row('parameter_free_begin', free_start, span=0, thread=2),
                row('parameter_free_end', free_start, free_end, span=0, thread=2)]

    def test_freed_parameter_before_handoff_is_a_lifetime_violation_not_causal_proof(self):
        result = evaluate(self.records())
        self.assertEqual(result['freed_parameter_handoffs'], 1)
        self.assertEqual(result['linked_parameter_handoffs'], 1)
        self.assertTrue(result['hook_coverage_valid'])
        self.assertFalse(result['physical_impulse_cause_identified'])
        self.assertFalse(result['actual_native_consumer_read_observed'])

    def test_earlier_address_reuse_overlapping_get_and_later_free_cannot_prove_violation(self):
        for begin, end in ((1, 2), (21, 25), (45, 46)):
            with self.subTest(begin=begin):
                self.assertEqual(evaluate(self.records(begin, end))['freed_parameter_handoffs'], 0)
        records = self.records()
        records[-1]['parameter_address'] = '0x789'
        self.assertEqual(evaluate(records)['freed_parameter_handoffs'], 0)

    def test_incomplete_span_missing_hooks_and_bad_records_do_not_certify_coverage(self):
        records = [r for r in self.records() if r['event'] != 'bridge_end']
        self.assertFalse(evaluate(records)['hook_coverage_valid'])
        self.assertEqual(evaluate(records)['freed_parameter_handoffs'], 0)
        records = self.records()
        records[0]['end_monotonic_ns'] = 0
        with self.assertRaises(ValueError):
            evaluate(records)


if __name__ == '__main__':
    unittest.main()
