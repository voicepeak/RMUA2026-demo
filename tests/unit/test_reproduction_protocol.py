import json
from pathlib import Path
import tempfile
import unittest
from evaluation.seed_runner.p1_regression import load_protocol


class ProtocolTests(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()
