import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

TOOLS=Path(__file__).resolve().parents[1];sys.path.insert(0,str(TOOLS))
sys.path.insert(0,str(TOOLS.parent/'ros_ws/src/route_follower/scripts'))
from local_occupancy import LocalOccupancy,OccupancyConfig,FREE,OCCUPIED
from planning_visualization import occupancy_marker_specs
from replay_local_occupancy import read_frame,replay


class OccupancyReplayTests(unittest.TestCase):
    def write_cloud(self,path,origin=True,raw=True,stamp=1.,epoch=0):
        reference=np.array([[i+.1,.1,.1] for i in range(66)])
        values=dict(points=np.array([[3.1,.1,.1],[4.1,1.1,.1]]),position=np.array([.1,.1,.1]),reference=reference)
        if raw:values['frame_points']=values['points'][:1]
        if origin:values.update(sensor_origin=np.array([.1,.1,.05]),cloud_stamp=stamp,lidar_epoch=epoch)
        np.savez_compressed(path.with_suffix('.npz'),**values)
        path.write_text(json.dumps(dict(s=.1,cloud_stamp=stamp,lidar_epoch=epoch,
                                       sensor_origin=[.1,.1,.05] if origin else None)))

    def test_origins_are_required_and_unsmoothed_frame_is_selected(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'frame.json';self.write_cloud(path)
            meta,frame,position,reference=read_frame(path)
            self.assertEqual(len(frame.points),1)
            np.testing.assert_allclose(frame.sensor_origin,[.1,.1,.05])
            self.write_cloud(path,origin=False)
            with self.assertRaisesRegex(ValueError,'sensor_origin'):read_frame(path)

    def test_conflicting_origin_or_timestamp_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'frame.json'
            for changed in ({'sensor_origin':[0.,0.,0.]},{'cloud_stamp':2.},{'lidar_epoch':1}):
                self.write_cloud(path)
                meta=json.loads(path.read_text());meta.update(changed);path.write_text(json.dumps(meta))
                with self.assertRaisesRegex(ValueError,'Conflicting'):read_frame(path)

    def test_fractional_epoch_cannot_be_silently_truncated(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'frame.json';self.write_cloud(path,epoch=.5)
            with self.assertRaisesRegex(ValueError,'epoch'):read_frame(path)

    def test_complete_replay_preserves_reset_epoch_and_writes_three_state_preview(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);clouds=root/'clouds';clouds.mkdir()
            for i,(epoch,stamp) in enumerate(((0,10.),(0,10.1),(1,1.),(1,1.1))):
                self.write_cloud(clouds/('%d.json'%i),stamp=stamp,epoch=epoch)
            result=replay(clouds,root/'out')
            rows=[json.loads(line) for line in (root/'out/occupancy.jsonl').read_text().splitlines()]
            self.assertEqual(result['frames'],4)
            self.assertEqual([r['stamp'] for r in rows],[10.,10.1,1.,1.1])
            self.assertNotEqual(rows[1]['map']['epoch'],rows[2]['map']['epoch'])
            self.assertGreater(rows[-1]['map']['free'],0);self.assertGreater(rows[-1]['map']['unknown'],0)
            self.assertTrue((root/'out/preview.html').is_file())
            with self.assertRaises(ValueError):replay(clouds,root/'out')

    def test_old_archive_is_rejected_before_creating_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);clouds=root/'clouds';clouds.mkdir()
            self.write_cloud(clouds/'old.json',origin=False)
            with self.assertRaises(ValueError):replay(clouds,root/'out')
            self.assertFalse((root/'out').exists())

    def test_bounded_marker_specs_use_explicit_enu(self):
        mapping=LocalOccupancy(OccupancyConfig(resolution=1.,forward=8.))
        mapping.update([[3.1,.1,.1]],[.1,.1,.1],1.)
        specs=occupancy_marker_specs(mapping.snapshot(),max_count=2,show_unknown=True)
        self.assertEqual(len(specs),3)
        self.assertTrue(all(len(s['positions'])<=2 for s in specs))
        occupied=next(spec for spec in specs if spec['state']==OCCUPIED)
        np.testing.assert_allclose(occupied['positions'][0],[.5,3.5,-.5])
        self.assertEqual(occupied['kind'],'voxels')


if __name__=='__main__':unittest.main()
