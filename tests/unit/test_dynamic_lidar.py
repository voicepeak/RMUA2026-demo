import unittest
import numpy as np

from interfaces.contracts import Metadata
from perception.object_motion.lidar_clusters import Cluster, ClusterFrame, cluster_endpoints
from world_model.dynamic_tracks.tracker import DynamicTracker, predict_candidate
from world_model.occupancy.rolling_endpoints import WorldEndpointObservation


def observation(stamp,points):
    meta=Metadata(stamp,stamp+.2,0,'lidar','world_ned')
    source=Metadata(stamp,stamp+.2,0,'raw','lidar')
    return WorldEndpointObservation(meta,source,np.array(points,dtype=float).reshape(-1,3),(0.,0.,0.),(stamp,stamp))


def frame(stamp,centers):
    return ClusterFrame(Metadata(stamp,stamp+.2,0,'cluster','world_ned'),
                        tuple(Cluster(tuple(c),(.5,.5,.5),10) for c in centers),len(centers),0)


class DynamicLidarTests(unittest.TestCase):
    def test_clusters_enclose_sampled_points_and_exclude_sparse_noise(self):
        cloud=[(1.+.01*i,2.,3.) for i in range(10)]+[(6.,0.,0.)]
        output=cluster_endpoints(observation(10.,cloud),10.,0)
        self.assertEqual(len(output.clusters),1);self.assertEqual(output.clusters[0].points,10)
        self.assertEqual(output.rejected_components,1);self.assertFalse(output.control_eligible)
        c=output.clusters[0]
        for point in cloud[:10]:
            self.assertTrue(all(abs(a-b)<=h+1e-12 for a,b,h in zip(point,c.center_ned,c.half_extents_m)))

    def test_track_velocity_lease_and_epoch(self):
        tracker=DynamicTracker(max_speed_m_s=10.)
        tracker.ingest(frame(10.,[(0.,0.,0.)]),10.,0)
        tracker.ingest(frame(10.1,[(.2,0.,0.)]),10.1,0)
        track=tracker.snapshot(10.1,0)[0]
        self.assertAlmostEqual(track.velocity_ned_m_s[0],2.);self.assertEqual(track.observations,2)
        self.assertFalse(track.control_eligible);self.assertFalse(track.safety_certified)
        self.assertEqual(tracker.snapshot(10.3,0),())
        tracker.reset(1);self.assertEqual(tracker.snapshot(1.,1),())

    def test_prediction_expands_bounds_and_rejects_stale_or_untracked(self):
        t=DynamicTracker(max_speed_m_s=10.);t.ingest(frame(10.,[(0.,0.,0.)]),10.,0)
        one=t.snapshot(10.,0)[0]
        with self.assertRaises(ValueError):predict_candidate(one,10.,10.1,0,position_error_m=.1,velocity_error_m_s=.2,acceleration_bound_m_s2=1.)
        t.ingest(frame(10.1,[(.2,0.,0.)]),10.1,0);track=t.snapshot(10.1,0)[0]
        value=predict_candidate(track,10.1,10.3,0,position_error_m=.1,velocity_error_m_s=.2,acceleration_bound_m_s2=1.)
        self.assertAlmostEqual(value.center_ned[0],.6);self.assertAlmostEqual(value.half_extents_m[0],.66)
        self.assertFalse(value.safety_certified);self.assertFalse(value.control_eligible)
        with self.assertRaises(ValueError):predict_candidate(track,10.3,10.4,0,position_error_m=.1,velocity_error_m_s=.2,acceleration_bound_m_s2=1.)

    def test_ambiguous_many_to_one_does_not_create_motion(self):
        t=DynamicTracker(max_speed_m_s=10.)
        t.ingest(frame(10.,[(0.,0.,0.)]),10.,0)
        stats=t.ingest(frame(10.1,[(-.2,0.,0.),(.2,0.,0.)]),10.1,0)
        self.assertEqual(stats['ambiguous'],2);self.assertEqual(t.snapshot(10.1,0)[0].observations,1)

    def test_stale_duplicate_and_capacity_failure(self):
        t=DynamicTracker(capacity=1);t.ingest(frame(10.,[(0.,0.,0.)]),10.,0)
        with self.assertRaises(ValueError):t.ingest(frame(10.,[(0.,0.,0.)]),10.1,0)
        with self.assertRaises(ValueError):t.ingest(frame(10.1,[(100.,0.,0.)]),10.1,0)
        self.assertEqual(t.snapshot(10.1,0)[0].observations,1)
        with self.assertRaises(ValueError):cluster_endpoints(observation(10.,[(1.,2.,3.)]),10.2,0)


if __name__ == '__main__':unittest.main()
