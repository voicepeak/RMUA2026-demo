"""Behavioral tests with known motion; no ROS or simulated controller required."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from dynamic_tracker import DynamicTracker,TrackerConfig
from lidar_scene import LidarCluster,LidarScene,extract_clusters


def observation(center,size=(1.,1.,1.),indices=(0,),truncated=False):
    center=np.array(center,dtype=float);half=np.asarray(size)/2.
    return LidarCluster(center,center-half,center+half,np.array(indices),truncated)


def box_points(center):
    return np.array([(x,y,z) for x in np.linspace(-.5,.5,6)
                     for y in np.linspace(-.5,.5,6) for z in (-.5,.5)])+center


class DynamicTrackerTests(unittest.TestCase):
    def test_cv_tracks_actual_world_velocity_with_irregular_sensor_intervals(self):
        tracker=DynamicTracker();velocity=np.array([2.,-.7,.15])
        times=np.cumsum(np.resize([.06,.12,.09,.17],40))
        ids=[]
        for stamp in times:
            tracks=tracker.update_clusters([observation([8.,1.,0.]+velocity*stamp)],stamp)
            self.assertEqual(len(tracks),1);ids.append(tracks[0].track_id)
        track=tracks[0]
        self.assertEqual(len(set(ids)),1);self.assertEqual(track.status,'CONFIRMED')
        self.assertTrue(track.motion_confirmed)
        np.testing.assert_allclose(track.velocity,velocity,atol=.015)
        for dt in (1.,2.):
            np.testing.assert_allclose(track.predict(dt).position,[8.,1.,0.]+velocity*(times[-1]+dt),atol=.04)
        self.assertGreaterEqual(np.linalg.eigvalsh(track.covariance).min(),-1e-12)

    def test_static_world_returns_do_not_acquire_aircraft_motion(self):
        tracker=DynamicTracker();points=box_points([8.,0.,0.])
        for i in range(20):
            tracks=tracker.update(points,np.array([i*.2,0.,0.]),1.+i*.1,
                                  [1.,0.],lambda s:0.,i*.2)
        self.assertEqual(len(tracks),1)
        np.testing.assert_allclose(tracks[0].velocity,0.,atol=1e-10)
        self.assertEqual(tracker.summary()['moving_count'],0)

    def test_association_is_one_to_one_even_when_cluster_splits(self):
        tracker=DynamicTracker()
        first=tracker.update_clusters([observation([0.,0.,0.])],0.)[0]
        tracks=tracker.update_clusters([observation([.05,-.1,0.],indices=(1,)),
                                        observation([.05,.1,0.],indices=(2,))],.1)
        self.assertEqual(len(tracks),2)
        self.assertEqual(sum(t.track_id==first.track_id for t in tracks),1)
        self.assertEqual(sorted(t.observations for t in tracks),[1,2])
        self.assertEqual(tracker.last_info['matched'],1)

    def test_nearby_opposing_tracks_keep_ids_when_detection_order_changes(self):
        tracker=DynamicTracker();expected={}
        for i in range(51):
            t=i*.1
            a=observation([-3.+1.2*t,-.5,0.],indices=(1,))
            b=observation([3.-1.2*t,.5,0.],indices=(2,))
            observations=[a,b] if i%2 else [b,a]
            tracks=tracker.update_clusters(observations,t)
            ids={int(track.point_indices[0]):track.track_id for track in tracks}
            if i==0:expected=ids
            self.assertEqual(ids,expected)
        self.assertGreater(next(t for t in tracks if t.point_indices[0]==1).velocity[0],1.)
        self.assertLess(next(t for t in tracks if t.point_indices[0]==2).velocity[0],-1.)

    def test_short_occlusion_coasts_then_reacquires_same_id(self):
        tracker=DynamicTracker()
        for i in range(10):track=tracker.update_clusters([observation([i*.1,0.,0.])],i*.1)[0]
        id_before=track.track_id;cov_before=track.covariance.copy()
        coast=tracker.update_clusters([],1.3)[0]
        self.assertEqual(coast.status,'COASTING');self.assertAlmostEqual(coast.age,.4)
        self.assertEqual(len(coast.point_indices),0)
        self.assertGreater(np.trace(coast.position_covariance),np.trace(cov_before[:3,:3]))
        self.assertGreater(coast.position[0],track.position[0]);self.assertLess(coast.confidence,track.confidence)
        returned=tracker.update_clusters([observation([1.4,0.,0.])],1.4)[0]
        self.assertEqual(returned.track_id,id_before);self.assertEqual(returned.status,'CONFIRMED')

    def test_long_gap_expires_before_matching_and_does_not_reuse_id(self):
        tracker=DynamicTracker()
        for i in range(8):old=tracker.update_clusters([observation([i*.1,0.,0.])],i*.1)[0]
        fresh=tracker.update_clusters([observation([1.6,0.,0.])],1.6)[0]
        self.assertNotEqual(fresh.track_id,old.track_id);self.assertEqual(fresh.status,'TENTATIVE')
        self.assertEqual(tracker.last_info['expired'],1)

    def test_tentative_track_requires_consecutive_hits_and_expires(self):
        tracker=DynamicTracker()
        track=tracker.update_clusters([observation([0.,0.,0.])],0.)[0]
        tracker.update_clusters([],.1)
        tracker.update_clusters([observation([0.,0.,0.])],.2)
        track=tracker.update_clusters([observation([0.,0.,0.])],.3)[0]
        self.assertEqual(track.status,'TENTATIVE')
        self.assertEqual(tracker.update_clusters([],1.),())

    def test_far_detection_cannot_be_forced_into_a_track(self):
        tracker=DynamicTracker()
        old=tracker.update_clusters([observation([0.,0.,0.])],0.)[0]
        tracks=tracker.update_clusters([observation([10.,0.,0.])],.1)
        self.assertEqual(len(tracks),2)
        self.assertEqual(next(t for t in tracks if t.track_id==old.track_id).observations,1)

    def test_duplicate_stamp_is_idempotent_and_clock_reset_changes_epoch(self):
        tracker=DynamicTracker()
        old=tracker.update_clusters([observation([0.,0.,0.])],2.)[0]
        repeated=tracker.update_clusters([observation([10.,0.,0.])],2.)[0]
        np.testing.assert_array_equal(repeated.position,old.position)
        self.assertEqual(repeated.observations,1)
        fresh=tracker.update_clusters([observation([1.,0.,0.])],1.)[0]
        self.assertGreater(fresh.epoch,old.epoch);self.assertNotEqual(fresh.track_id,old.track_id)

    def test_prediction_uncertainty_grows_and_absolute_age_is_not_double_counted(self):
        tracker=DynamicTracker()
        for i in range(10):track=tracker.update_clusters([observation([i*.1,0.,0.])],i*.1)[0]
        coast=tracker.update_clusters([],1.2)[0]
        np.testing.assert_allclose(coast.predict(.5).position,coast.predict_at(1.7).position)
        predictions=[coast.predict(t) for t in (0.,1.,2.,4.)]
        for a,b in zip(predictions,predictions[1:]):
            self.assertTrue(np.all(b.uncertainty>a.uncertainty))
            self.assertTrue(np.all(b.high-b.low>a.high-a.low))
        for invalid in (-1.,np.nan,np.inf):
            with self.assertRaises(ValueError):coast.predict(invalid)
        with self.assertRaises(ValueError):coast.predict_at(.8)

    def test_snapshot_is_independent_of_future_updates(self):
        tracker=DynamicTracker();old=tracker.update_clusters([observation([0.,0.,0.])],0.)[0]
        before=old.predict(2.).position.copy()
        tracker.update_clusters([observation([.1,0.,0.])],.1)
        np.testing.assert_array_equal(old.predict(2.).position,before)
        with self.assertRaises(ValueError):old.position[0]=10.
        with self.assertRaises(ValueError):old.point_indices[0]=10

    def test_cropped_or_changing_box_withholds_motion_ownership(self):
        tracker=DynamicTracker()
        for i in range(5):track=tracker.update_clusters([observation([0.,0.,0.])],i*.1)[0]
        self.assertTrue(track.motion_confirmed)
        cropped=tracker.update_clusters([observation([.25,0.,0.],size=(.5,1.,1.))],.5)[0]
        self.assertFalse(cropped.motion_confirmed)
        self.assertEqual(cropped.bbox_size[0],1.)
        truncated=tracker.update_clusters([observation([.25,0.,0.],truncated=True)],.6)[0]
        self.assertFalse(truncated.motion_confirmed)
        self.assertTrue(truncated.truncated)

    def test_invalid_inputs_fail_before_modifying_filter(self):
        tracker=DynamicTracker();tracker.update_clusters([observation([0.,0.,0.])],0.)
        for stamp in (np.nan,np.inf):
            with self.assertRaises(ValueError):tracker.update_clusters([],stamp)
        with self.assertRaises(ValueError):tracker.update_clusters([observation([np.nan,0.,0.])],.1)
        self.assertEqual(tracker.stamp,0.)
        for params in ({'measurement_std':0.},{'max_coast':-1.},{'confirmation_hits':1.5},
                       {'acceleration_std':np.nan}):
            with self.assertRaises(ValueError):TrackerConfig(**params)

    def test_filter_adapts_to_velocity_reversal_without_reinitializing_identity(self):
        tracker=DynamicTracker();ids=[]
        for i in range(61):
            t=i*.1;x=t if t<=2. else 4.-t
            track=tracker.update_clusters([observation([x,0.,0.])],t)[0]
            ids.append(track.track_id)
        self.assertEqual(len(set(ids)),1)
        self.assertLess(track.velocity[0],-.9)
        self.assertLess(track.predict(1.).position[0],track.position[0])


class ClusterExtractionTests(unittest.TestCase):
    def test_clusters_preserve_original_return_ownership(self):
        a=box_points([5.,-2.,0.]);b=box_points([10.,2.,0.])
        points=np.vstack(([np.nan,0.,0.],a,[50.,0.,0.],b))
        clusters=extract_clusters(points,[0.,0.,0.],[1.,0.],lambda s:0.,0.)
        self.assertEqual(len(clusters),2)
        owned=np.concatenate([c.point_indices for c in clusters])
        self.assertEqual(len(set(owned)),len(a)+len(b))
        self.assertNotIn(0,owned);self.assertNotIn(len(a)+1,owned)
        for c in clusters:
            np.testing.assert_allclose(c.low,points[c.point_indices].min(axis=0))
            np.testing.assert_allclose(c.high,points[c.point_indices].max(axis=0))

    def test_measured_ground_removal_uses_ned_sign(self):
        body=box_points([8.,0.,0.]);ground=box_points([15.,0.,1.])
        clusters=extract_clusters(np.vstack((body,ground)),[0.,0.,0.],[1.,0.],lambda s:0.,0.,
                                  ground_height=lambda s:np.ones_like(s)*.5)
        self.assertEqual(len(clusters),1)
        self.assertLess(clusters[0].high[2],.5)

    def test_roi_clipped_box_is_marked_uncertain(self):
        clusters=extract_clusters(box_points([8.,3.6,0.]),[0.,0.,0.],[1.,0.],lambda s:0.,0.)
        self.assertEqual(len(clusters),1);self.assertTrue(clusters[0].truncated)

    def test_empty_and_invalid_geometry(self):
        self.assertEqual(extract_clusters(np.empty((0,3)),[0.,0.,0.],[1.,0.],lambda s:0.,0.),[])
        with self.assertRaises(ValueError):
            extract_clusters(box_points([8.,0.,0.]),[0.,0.,0.],[0.,0.],lambda s:0.,0.)

    def test_legacy_sparse_roi_keeps_existing_short_lifetime_behavior(self):
        scene=LidarScene();points=box_points([8.,0.,0.])
        scene.update(points,np.zeros(3),1.,1.,[1.,0.],lambda s:0.,0.)
        self.assertEqual(len(scene.tracks),1)
        scene.update(points+[100.,0.,0.],np.zeros(3),1.1,1.1,[1.,0.],lambda s:0.,0.)
        self.assertEqual(len(scene.tracks),1)
        scene.update(np.empty((0,3)),np.zeros(3),1.6,1.6,[1.,0.],lambda s:0.,0.)
        self.assertEqual(scene.tracks,[])


if __name__=='__main__':unittest.main()
