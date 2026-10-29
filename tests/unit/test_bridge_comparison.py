import json
from pathlib import Path
import tempfile
import unittest

from evaluation.seed_runner.bridge_comparison import case_order, collect_case, MODES


class BridgeComparisonTests(unittest.TestCase):
    def test_rotating_order_keeps_every_seed_mode_and_repeat(self):
        protocol = {'schema_version': 1, 'seeds': [2024, 2041], 'repeats': 3,
                    'hover_seconds': 14, 'stationary_bootstrap': True}
        cases = case_order(protocol)
        self.assertEqual(len(cases), 18)
        for seed in protocol['seeds']:
            for repeat in range(3):
                group = [c['mode'] for c in cases if c['seed'] == seed and c['repeat'] == repeat]
                self.assertEqual(set(group), set(MODES))
            first = [c['mode'] for i, c in enumerate(cases) if i % 3 == 0 and c['seed'] == seed]
            self.assertEqual(set(first), set(MODES))
        for change in ({'repeats': 0}, {'repeats': True}, {'seeds': [True]}, {'seeds': []},
                       {'hover_seconds': 300}, {'hover_seconds': float('nan')},
                       {'modes': ['lifetime']}, {'modes': ['none', 'none']},
                       {'stationary_bootstrap': 'false'}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                case_order(dict(protocol, **change))

    def test_lifetime_pair_rotates_each_seed_between_orders(self):
        cases = case_order({'schema_version': 1, 'seeds': [2024, 2041], 'repeats': 2,
                            'hover_seconds': 14, 'stationary_bootstrap': True,
                            'modes': ['none', 'lifetime']})
        self.assertEqual(len(cases), 8)
        for seed in (2024, 2041):
            first = [c['mode'] for i, c in enumerate(cases) if i % 2 == 0 and c['seed'] == seed]
            self.assertEqual(set(first), {'none', 'lifetime'})

    def test_native_pair_can_match_non_bootstrap_baseline(self):
        cases = case_order({'schema_version': 1, 'seeds': [1005], 'repeats': 2,
                            'hover_seconds': 14, 'stationary_bootstrap': False,
                            'modes': ['none', 'native-input']})
        self.assertEqual([c['mode'] for c in cases],
                         ['none', 'native-input', 'native-input', 'none'])

    def test_missing_trace_and_control_fault_are_reported_separately(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run / 'manifest.json').write_text(json.dumps({
                'run_state': 'FAILED', 'observer_exit_code': 0, 'command_trace_audit_exit_code': 2}))
            (run / 'pwm_summary.json').write_text(json.dumps({'commands': 100, 'fault': None}))
            result = collect_case(run)
            self.assertTrue(result['controller_completed'])
            self.assertEqual(result['command_trace_audit_exit_code'], 2)
            self.assertFalse(result['evaluation_valid'])
            (run / 'pwm_summary.json').write_text(json.dumps({'commands': 100, 'fault': 'ANGULAR_RATE_LIMIT'}))
            result = collect_case(run)
            self.assertFalse(result['controller_completed'])
            self.assertEqual(result['controller_fault'], 'ANGULAR_RATE_LIMIT')
            (run / 'pwm_summary.json').unlink()
            self.assertEqual(collect_case(run)['controller_fault'], 'MISSING_SUMMARY')


if __name__ == '__main__':
    unittest.main()
