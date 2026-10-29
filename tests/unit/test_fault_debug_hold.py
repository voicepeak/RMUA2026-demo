import unittest
from evaluation.metrics.command_trace import MD5
from evaluation.metrics.fault_debug_hold import evaluate


class FaultDebugHoldTest(unittest.TestCase):
    def test_only_final_matching_suffix_is_reported(self):
        telemetry = [{'command': {'values': [0.2]*4}}]
        def row(time, value):
            return {'arrival_wall_monotonic': time, 'md5sum': MD5,
                    'data': {'header': {'seq': 1, 'stamp': {'secs': 1, 'nsecs': 0}},
                             **{'rotorPWM'+str(i): value for i in range(4)}}}
        result = evaluate(telemetry, [row(2, .2), row(3, .1), row(4, .2), row(5, .2)], 1)
        self.assertEqual(result['matching_last_command_records'], 3)
        self.assertEqual(result['matching_suffix_records'], 2)
        self.assertEqual(result['matching_suffix_first_arrival_after_fault_s'], 3)
        self.assertEqual(result['last_record_arrival_after_fault_s'], 4)


if __name__ == '__main__':
    unittest.main()
