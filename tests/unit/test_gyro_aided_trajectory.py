import json
from pathlib import Path
import tempfile
import unittest

from evaluation.pipeline.gyro_aided_trajectory import build


class GyroAidedTrajectoryTests(unittest.TestCase):
    def test_imu_coverage_gaps_break_visual_chain(self):
        def row(a,b):
            return {'status':'ESTIMATED','previous_stamp':a,'current_stamp':b,'motion':{
                'previous_stamp':a,'current_stamp':b,'epoch':0,'translation_previous_body_m':[1,0,0]}}
        estimates={'source':'synthetic','results':[row(1.,1.02),row(1.02,1.2),row(1.2,1.22)]}
        stamps=(1.,1.01,1.02,1.2,1.21,1.22)
        telemetry=[{'raw':{'stamp':s,'omega_body':[0,0,0]},'command':{'epoch':0}} for s in stamps]
        with tempfile.TemporaryDirectory() as directory:
            a=Path(directory)/'estimate.json';b=Path(directory)/'imu.jsonl'
            a.write_text(json.dumps(estimates));b.write_text(''.join(json.dumps(r)+'\n' for r in telemetry))
            result=build(a,b)
        self.assertEqual([len(s['poses']) for s in result['segments']],[1,1])
        self.assertEqual(result['breaks'][0]['reason'],'gyro sample gap')
        self.assertFalse(result['truth_accessed']);self.assertFalse(result['control_eligible'])


if __name__=='__main__':unittest.main()
