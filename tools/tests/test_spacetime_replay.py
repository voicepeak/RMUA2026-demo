from dataclasses import replace
from pathlib import Path
import json
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from spacetime_scenarios import known_snapshot,obstacle
from spacetime_snapshot import write_snapshot,read_snapshot
from replay_spacetime_navigation import replay
from trajectory_collision import TrajectoryCollision
from st_lattice import LatticeConfig,STLattice


class SpacetimeReplayTests(unittest.TestCase):
    def test_snapshot_round_trip_retains_response_history_and_full_track_covariance(self):
        snapshot,parameters=known_snapshot([obstacle([10.,0.,0.],[0.,1.,0.])],velocity=(2.,1.,0.))
        parameters=replace(parameters,xy_error_max=float('inf'))
        snapshot=replace(snapshot,response_state=replace(snapshot.response_state,model_key=parameters.model_key))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'snapshot.json';write_snapshot(snapshot,parameters,path,'test oracle')
            restored,model,provenance=read_snapshot(path)
            self.assertEqual(provenance,'test oracle');self.assertEqual(model.model_key,parameters.model_key)
            np.testing.assert_array_equal(restored.response_state.previous,snapshot.response_state.previous)
            np.testing.assert_array_equal(restored.response_state.next_control,snapshot.response_state.next_control)
            np.testing.assert_array_equal(restored.obstacles[0].covariance,snapshot.obstacles[0].covariance)
            self.assertTrue(TrajectoryCollision(restored).freshness().safe)
            with self.assertRaises(ValueError):restored.response_state.applied[0,0]=5.
            with self.assertRaises(ValueError):write_snapshot(snapshot,parameters,path,'overwrite')

    def test_complete_frozen_replay_writes_plan_and_preview(self):
        snapshot,parameters=known_snapshot(velocity=(2.,0.,0.))
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);path=root/'snapshot.json';write_snapshot(snapshot,parameters,path,'test oracle')
            result=replay(path,root/'out',budget=.2)
            self.assertIsNotNone(result['result']['plan_id'])
            self.assertTrue((root/'out/trajectory.npz').is_file());self.assertTrue((root/'out/preview.html').is_file())
            with np.load(root/'out/trajectory.npz') as data:
                self.assertLess(result['valid_until'],float(data['backup_times'][-1]))
            with self.assertRaises(ValueError):replay(path,root/'out')

    def test_old_cloud_archive_and_changed_model_hash_are_rejected_before_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);path=root/'snapshot.json';path.write_text(json.dumps(dict(s=0.,cloud_stamp=100.)))
            with self.assertRaisesRegex(ValueError,'response history'):replay(path,root/'out')
            self.assertFalse((root/'out').exists());path.unlink()
            snapshot,parameters=known_snapshot();write_snapshot(snapshot,parameters,path,'test')
            meta=json.loads(path.read_text());meta['response_parameters']['lift_gain']=.11;path.write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError,'hash'):read_snapshot(path)

    def test_nonfinite_scene_time_and_invalid_covariance_cannot_certify(self):
        snapshot,_=known_snapshot([obstacle([10.,0.,0.])])
        bad=replace(snapshot,occupancy=replace(snapshot.occupancy,stamp=float('nan')))
        self.assertEqual(TrajectoryCollision(bad).freshness().reason,'SCENE_INVALID')
        bad=replace(snapshot,obstacles=(replace(snapshot.obstacles[0],covariance=-np.eye(6)),))
        self.assertEqual(TrajectoryCollision(bad).freshness().reason,'TRACK_INVALID')


if __name__=='__main__':unittest.main()
