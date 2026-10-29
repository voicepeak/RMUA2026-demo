import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from evaluation.seed_runner import p1_regression
from evaluation.seed_runner.p1_regression import load_protocol, case_succeeded


class ProtocolTests(unittest.TestCase):
    def test_case_requires_completed_valid_evidence(self):
        valid={'exit_code':0,'run_source_unchanged':True,'summary':{'fault':None},
               'run_state':'EVALUATED','evaluation_valid':True}
        self.assertTrue(case_succeeded(valid))
        for change in ({'exit_code':2}, {'summary':{'fault':'ANGULAR_RATE_LIMIT'}},
                       {'summary':{}}, {'run_source_unchanged':False},
                       {'run_state':'FAILED'}, {'evaluation_valid':False},
                       {'evaluation_valid':None}):
            self.assertFalse(case_succeeded(dict(valid,**change)))

    def test_batch_retains_failed_case_but_returns_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            protocol={'schema_version':1,'seeds':[1005]*3,'mode':'PWM','hover_seconds':14,
                      'braking':False,'yaw_motion':False,'snapshot':False}
            path=root/'protocol.json'; path.write_text(json.dumps(protocol))
            counter=iter([0,2,0])
            def run_case(command, cwd, stdout, stderr):
                code=next(counter); run=root/'runs'/('case'+str(run_case.index))
                run_case.index+=1; run.mkdir(parents=True)
                (run/'manifest.json').write_text(json.dumps({'source_unchanged':True,
                    'source_sha256_before':{'source':'hash'},'run_state':'FAILED' if code else 'EVALUATED'}))
                (run/'pwm_summary.json').write_text(json.dumps({'fault':'ANGULAR_RATE_LIMIT' if code else None}))
                (run/'closed_loop_metrics.json').write_text(json.dumps({'evaluation_valid':not bool(code)}))
                stdout.write(str(run)+'\n'); stdout.flush()
                return code
            run_case.index=0
            with patch.object(p1_regression, '__file__', str(root/'evaluation/seed_runner/p1_regression.py')), \
                 patch.object(p1_regression, 'source_hashes', return_value={'source':'hash'}), \
                 patch.object(p1_regression, 'archive_sources', return_value={}), \
                 patch.object(p1_regression.subprocess, 'call', side_effect=run_case), \
                 patch('sys.argv', ['p1_regression','--protocol',str(path)]):
                self.assertEqual(p1_regression.main(),1)
            report=json.loads(next((root/'runs/batches').glob('*/batch_report.json')).read_text())
            self.assertTrue(report['all_cases_completed'])
            self.assertFalse(report['batch_succeeded'])
            self.assertEqual(report['successful_cases'],2)
            self.assertEqual([c['exit_code'] for c in report['cases']],[0,2,0])

    def test_bad_seed_and_unbounded_duration_are_rejected_before_startup(self):
        valid={'schema_version':1,'seeds':[1016,1016],'mode':'PWM','hover_seconds':14,
               'braking':True,'yaw_motion':True,'snapshot':False}
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'protocol.json'
            p.write_text(json.dumps(valid));self.assertEqual(load_protocol(p)['seeds'],[1016,1016])
            for change in ({'seeds':[-1]},{'seeds':[True]},{'hover_seconds':300},{'mode':'MIXED'},
                           {'braking':'true'},{'seeds':[]}):
                p.write_text(json.dumps({**valid,**change}))
                with self.assertRaises(ValueError):load_protocol(p)

    def test_staged_pulses_cannot_skip_recovery_or_jump_over_the_local_limit(self):
        valid={'schema_version':1,'seeds':[2030,2041],'mode':'PWM','hover_seconds':14,
               'braking':False,'yaw_motion':False,'snapshot':False,'angular_recovery':True,
               'angular_pulse_moments':[.5,.8]}
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'protocol.json';p.write_text(json.dumps(valid))
            self.assertEqual(load_protocol(p)['angular_pulse_moments'],[.5,.8])
            for change in ({'angular_recovery':False},{'braking':True},{'mode':'VEL'},
                           {'angular_pulse_moments':[.8,.5]},{'angular_pulse_moments':[.9]},
                           {'angular_pulse_moments':[True]}):
                p.write_text(json.dumps(dict(valid,**change)))
                with self.assertRaises(ValueError):load_protocol(p)


if __name__ == '__main__':
    unittest.main()
