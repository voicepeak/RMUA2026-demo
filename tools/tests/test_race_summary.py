import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from summarize_debug_race import summarize


class RaceSummaryTests(unittest.TestCase):
    def test_previous_finished_window_does_not_hide_current_valid_hud(self):
        with tempfile.TemporaryDirectory() as directory:
            run=Path(directory);(run/'hud').mkdir();(run/'flight').mkdir()
            (run/'settings.json').write_text(json.dumps(dict(Vehicles={'drone_1':{'Sensors':{'lidar':{'PointsPerSecond':75000}}}})))
            (run/'startup.json').write_text(json.dumps(dict(wall_time=10.)))
            captures=[dict(wall_time=9.,ocr='State:Finished'),
                      dict(wall_time=11.,ocr='State:State2'),
                      dict(wall_time=12.,ocr='State:Finished')]
            (run/'hud/captures.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in captures))
            telemetry=[dict(topic='telemetry',received=t,data=dict(s=s,speed=3.)) for t,s in ((10.5,5.),(11.5,8.))]
            (run/'flight/streams.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in telemetry))
            result=summarize(run)
            self.assertEqual(result['first_finished_hud']['wall_time'],12.)
            self.assertEqual(result['max_local_s_by_stage']['0'],5.)
            self.assertEqual(result['valid_telemetry_samples'],1)
