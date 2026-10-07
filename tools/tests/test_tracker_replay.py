import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

TOOLS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(TOOLS.parent/'ros_ws/src/route_follower/scripts'))
from dynamic_tracker import DynamicTracker
from lidar_scene import LidarCluster
from planning_visualization import track_marker_specs
spec=importlib.util.spec_from_file_location('tracker_replay',TOOLS/'replay_dynamic_tracker.py')
replay_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(replay_module)


class TrackerReplayTests(unittest.TestCase):
    def test_marker_projection_is_explicit_enu_and_bounded(self):
        tracker=DynamicTracker()
        center=np.array([2.,3.,-4.]);size=np.array([1.,2.,3.])
        observation=LidarCluster(center,center-size/2.,center+size/2.,np.array([0]))
        tracks=tracker.update_clusters([observation],0.)
        markers=track_marker_specs(tracks)
        self.assertEqual(len(markers),4)
        np.testing.assert_allclose(markers[0]['position'],[3.,2.,4.])
        np.testing.assert_allclose(markers[0]['size'],[2.,1.,3.])
        self.assertEqual([m['dt'] for m in markers if m['kind']=='box'],[0.,1.,2.])
        self.assertGreater(markers[3]['size'][0],markers[2]['size'][0])
        self.assertEqual(track_marker_specs(tracks*2,max_tracks=1),markers)

    def test_continuous_recorded_cloud_pipeline_reports_prediction_residuals(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);clouds=root/'clouds';clouds.mkdir()
            shape=np.array([(x,y,z) for x in np.linspace(-.5,.5,6)
                            for y in np.linspace(-.5,.5,6) for z in (-.5,.5)])
            reference=np.array([[float(i),0.,0.] for i in range(66)])
            for i in range(45):
                t=i*.1
                # Filenames deliberately do not sort in observation order.
                name=clouds/('%03d'%(45-i))
                points=shape+[8.+.6*t,0.,0.]
                np.savez_compressed(str(name)+'.npz',points=points,position=np.zeros(3),reference=reference)
                Path(str(name)+'.json').write_text(json.dumps(dict(s=0.,cloud_stamp=10.+t,pose_stamp=10.+t)))
            output=root/'replay'
            summary=replay_module.replay(clouds,output,TOOLS.parent/'ros_ws/src/route_follower/config/dynamic_tracker.yaml')
            self.assertEqual(summary['frames'],45)
            self.assertEqual(summary['tracks_created'],1)
            self.assertGreater(summary['moving_frames'],30)
            self.assertEqual(summary['proxy_timestamp_frames'],0)
            error=summary['prediction_observation_residual_m']['2.0']
            self.assertGreater(error['samples'],10)
            self.assertLess(error['statistics']['p95'],.1)
            rows=[json.loads(line) for line in (output/'tracks.jsonl').read_text().splitlines()]
            self.assertEqual(rows[0]['stamp'],10.)
            self.assertTrue((output/'preview.html').is_file())
            self.assertEqual(rows[-1]['tracks'][0]['track_id'],1)
            with self.assertRaises(ValueError):
                replay_module.replay(clouds,output,TOOLS.parent/'ros_ws/src/route_follower/config/dynamic_tracker.yaml')

    def test_sparse_archive_does_not_extend_lifetimes_to_fabricate_stable_ids(self):
        self.assertEqual(replay_module.observation_stamp({'pose_stamp':1.}), (1.,'pose_stamp_proxy'))
        self.assertEqual(replay_module.observation_stamp({'pose_stamp':1.,'clearance':{'geometry_cloud_stamp':.9}}),
                         (.9,'geometry_cloud_stamp'))
        tracker=DynamicTracker()
        observation=LidarCluster(np.zeros(3),-np.ones(3),np.ones(3),np.array([0]))
        a=tracker.update_clusters([observation],1.)[0]
        b=tracker.update_clusters([observation],2.)[0]
        self.assertNotEqual(a.track_id,b.track_id)
        self.assertEqual(b.status,'TENTATIVE')

    def test_capture_epochs_are_not_sorted_together_across_clock_reset(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);clouds=root/'clouds';clouds.mkdir()
            shape=np.array([(x,y,z) for x in np.linspace(-.5,.5,6)
                            for y in np.linspace(-.5,.5,6) for z in (-.5,.5)])+[8.,0.,0.]
            reference=np.array([[float(i),0.,0.] for i in range(66)])
            for i,(epoch,stamp) in enumerate(((0,10.),(0,10.1),(1,1.),(1,1.1))):
                name=clouds/str(i)
                np.savez_compressed(str(name)+'.npz',points=shape,position=np.zeros(3),reference=reference)
                Path(str(name)+'.json').write_text(json.dumps(dict(s=0.,cloud_stamp=stamp,pose_stamp=stamp,epoch=epoch)))
            replay_module.replay(clouds,root/'out',TOOLS.parent/'ros_ws/src/route_follower/config/dynamic_tracker.yaml')
            rows=[json.loads(line) for line in (root/'out/tracks.jsonl').read_text().splitlines()]
            self.assertEqual([r['stamp'] for r in rows],[10.,10.1,1.,1.1])
            self.assertNotEqual(rows[1]['tracks'][0]['epoch'],rows[2]['tracks'][0]['epoch'])
            self.assertNotEqual(rows[1]['tracks'][0]['track_id'],rows[2]['tracks'][0]['track_id'])


if __name__=='__main__':unittest.main()
