import json,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from race_seed import initial_course_inputs,cold_start_vision_arguments
from summarize_debug_race import summarize


class RaceSeedTests(unittest.TestCase):
    def test_seed123_keeps_its_measured_bootstrap(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);config=root/'config';out=root/'run'
            gates,guides=initial_course_inputs(config,out,123)
            self.assertEqual(gates,config/'gates_seed123_recorded.yaml')
            self.assertEqual(guides,config/'guides_seed123_recorded.yaml')
            self.assertFalse(out.exists())

    def test_other_seed_does_not_import_cached_gates_or_height_guides(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);config=root/'config';config.mkdir()
            (config/'gates_seed123_recorded.yaml').write_text('gates: [{x: 999}]')
            (config/'guides_seed123_recorded.yaml').write_text('altitude_guides: [{z: -999}]')
            gates,guides=initial_course_inputs(config,root/'run',456)
            self.assertEqual(json.loads(gates.read_text())['gates'],[])
            self.assertEqual(json.loads(guides.read_text())['altitude_guides'],[])
            self.assertEqual(json.loads(gates.read_text())['seed'],456)

    def test_summary_uses_actual_requested_seed_from_launch_manifest(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            (root/'settings.json').write_text(json.dumps(dict(Vehicles={'drone_1':{'Sensors':{'lidar':{'PointsPerSecond':75000}}}})))
            (root/'controller_versions.json').write_text(json.dumps(dict(seed=456)))
            self.assertEqual(summarize(root)['seed'],456)

    def test_wide_outbound_gate_bounds_do_not_expand_vehicle_gate_bounds(self):
        import numpy as np
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/rmua_gate_vision/scripts'))
        from gate_geometry import GateGeometry
        points=np.array([[-6.4,-3.8,30.],[6.4,-3.8,30.],[6.4,3.8,30.],[-6.4,3.8,30.]])
        args=cold_start_vision_arguments(456)
        dimensions={a.split(':=')[0][1:]:float(a.split(':=')[1]) for a in args if '_max_' in a}
        self.assertTrue(GateGeometry(**dimensions).compute(points)['geometry_valid'])
        self.assertFalse(GateGeometry().compute(points)['geometry_valid'])
        self.assertFalse(any('_max_' in a for a in cold_start_vision_arguments(456,vehicle=True)))
        self.assertEqual(cold_start_vision_arguments(123),[])


if __name__=='__main__':unittest.main()
