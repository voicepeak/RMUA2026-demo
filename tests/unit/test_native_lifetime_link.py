import unittest

from evaluation.metrics.native_lifetime_link import evaluate


def event(name, start, end, span=1, address='0x123'):
    return {'event': name, 'start_monotonic_ns': start, 'end_monotonic_ns': end,
            'bridge_span': span, 'parameter_address': address}


class NativeLifetimeLinkTests(unittest.TestCase):
    def test_strict_order_overlap_old_free_and_other_address(self):
        entry = event('native_pwm_entry', 30, 35)
        audit = {'unmatched': [{'entry': entry, 'snapshot_values': [0]*4, 'finite_unit_range': True}]}
        borrow = event('parameter_borrow', 10, 11)
        for free, expected in ((event('parameter_free_end', 20, 29, 0), 'free_completed_before_entry'),
                               (event('parameter_free_end', 20, 32, 0), 'free_overlaps_entry'),
                               (event('parameter_free_end', 1, 2, 0), 'no_linked_free'),
                               (event('parameter_free_end', 20, 29, 0, '0x999'), 'no_linked_free')):
            result = evaluate([entry, borrow, free], audit)
            self.assertEqual(result['findings'][0]['classification'], expected)
            self.assertFalse(result['allocation_identity_verified'])
            self.assertFalse(result['downstream_consumption_verified'])

    def test_missing_or_duplicate_borrow_is_ambiguous(self):
        entry = event('native_pwm_entry', 30, 35)
        audit = {'unmatched': [{'entry': entry, 'snapshot_values': [0]*4, 'finite_unit_range': True}]}
        borrow = event('parameter_borrow', 10, 11)
        for rows in ([entry], [entry, borrow, borrow]):
            self.assertEqual(evaluate(rows, audit)['findings'][0]['classification'], 'borrow_ambiguous')
