import struct
import unittest

from evaluation.metrics.native_input import evaluate


def records(values=(.1,.2,.3,.4),read_ok=True):
    base={'write_errors':0,'bridge_span':1,'parameter_address':'0x123'}
    return [dict(base,event='native_callback_bound'),
            dict(base,event='native_pwm_entry',start_monotonic_ns=10003000000,end_monotonic_ns=10003001000,
                 parameter_read_ok=read_ok,parameter_float32_hex=struct.pack('<4f',*values).hex()),
            dict(base,event='native_pwm_exit',start_monotonic_ns=10004000000,end_monotonic_ns=10004000000)]


def telemetry(begin=10.,end=10.001):
    return [{'publish_begin_wall_monotonic':begin,'publish_end_wall_monotonic':end,
             'command':{'values':(.1,.2,.3,.4),'sequence':7}}]


class NativeInputAuditTests(unittest.TestCase):
    def test_float32_conversion_and_monotonic_interval_bounds(self):
        result=evaluate(records(),telemetry(),10.004)
        self.assertEqual(result['matched_entries'],1)
        self.assertTrue(result['snapshot_coverage_valid'])
        self.assertEqual(result['matched'][0]['source_sequence'],7)
        self.assertAlmostEqual(result['publish_to_snapshot_lower_min_s'],.002)
        self.assertAlmostEqual(result['publish_to_snapshot_upper_max_s'],.003001)
        self.assertFalse(result['actual_actuator_consumption_verified'])

    def test_post_control_hold_is_excluded_and_free_is_linked_to_the_exact_original_call(self):
        rs=records()
        rs[-1].update(start_monotonic_ns=10003002000,end_monotonic_ns=10004000000,
                      native_call_interval_measured=True)
        rs.append(dict(rs[-1],event='parameter_free_end',start_monotonic_ns=10003500000,end_monotonic_ns=10003600000))
        rs.append(dict(rs[1],bridge_span=2,start_monotonic_ns=11003000000,end_monotonic_ns=11003001000))
        rs.append(dict(rs[2],bridge_span=2,start_monotonic_ns=11004000000,end_monotonic_ns=11004000000))
        result=evaluate(rs,telemetry(),10.004)
        self.assertEqual(result['parameters_freed_during_original_callback'],1)
        self.assertEqual(result['native_entries_after_control'],1)
        self.assertLess(result['publish_to_snapshot_upper_max_s'],.01)

    def test_unknown_payload_and_future_sender_are_not_matched(self):
        self.assertEqual(evaluate(records((1.,1.,1.,1.)),telemetry())['unmatched_entries'],1)
        self.assertEqual(evaluate(records(),telemetry(11.,11.001))['unmatched_entries'],1)
        result=evaluate(records((float('nan'),.2,.3,.4)),telemetry())
        self.assertIsNone(result['unmatched'][0]['snapshot_values'][0])
        self.assertFalse(result['unmatched'][0]['finite_unit_range'])

    def test_missing_read_or_exit_cannot_certify_snapshot_coverage(self):
        self.assertFalse(evaluate(records(read_ok=False),telemetry())['snapshot_coverage_valid'])
        result=evaluate(records()[:-1],telemetry())
        self.assertEqual(result['incomplete_native_calls'],1)
        self.assertFalse(result['snapshot_coverage_valid'])


if __name__=='__main__':unittest.main()
