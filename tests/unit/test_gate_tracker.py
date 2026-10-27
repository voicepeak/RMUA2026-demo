import math
import unittest
from dataclasses import replace

from estimation.history import HistoricalPose
from interfaces.contracts import Metadata
from perception.gate_geometry.stereo_gate import GateCandidate
from world_model.gate_landmarks.tracker import GateTracker


def pose(stamp, position=(0.,0.,0.), quaternion=(1.,0.,0.,0.), epoch=0):
    return HistoricalPose(stamp,epoch,position,quaternion,(stamp,stamp),((0.,)*15,(0.,)*15),0.,0.,0.)


def candidate(stamp, center=(10.,0.,0.), epoch=0):
    return GateCandidate(Metadata(stamp,stamp+.2,epoch,'candidate','body_frd'),
                         ((0.,0.,1.),(1.,0.,1.)),center,5.,4.,(10.,10.),(20,20))


class GateTrackerTests(unittest.TestCase):
    def test_world_transform_association_and_expiration(self):
        t=GateTracker(); q=(math.sqrt(.5),0.,0.,math.sqrt(.5))
        t.ingest((candidate(10.),),pose(10.,(1.,2.,3.),q),10.,0)
        first=t.snapshot(10.,0)[0];self.assertAlmostEqual(first.center_world_ned[0],1.)
        self.assertAlmostEqual(first.center_world_ned[1],12.);self.assertEqual(first.observations,1)
        t.ingest((candidate(10.1),),pose(10.1,(1.,2.,3.),q),10.1,0)
        second=t.snapshot(10.1,0)[0];self.assertEqual(second.observations,2)
        self.assertFalse(second.control_eligible);self.assertFalse(second.aperture_verified)
        self.assertEqual(t.snapshot(10.3,0),())

    def test_ambiguity_no_duplicate_frame_or_false_confirmation(self):
        t=GateTracker();t.ingest((candidate(10.),candidate(10.,(13.,0.,0.))),pose(10.),10.,0)
        result=t.ingest((candidate(10.1,(11.5,0.,0.)),),pose(10.1),10.1,0)
        self.assertEqual(result['ambiguous'],1)
        self.assertEqual([x.observations for x in t.snapshot(10.1,0)],[1,1])
        with self.assertRaises(ValueError): t.ingest((candidate(10.1),),pose(10.1),10.1,0)

    def test_two_candidates_cannot_both_update_same_track(self):
        t=GateTracker();t.ingest((candidate(10.),),pose(10.),10.,0)
        result=t.ingest((candidate(10.1,(9.5,0.,0.)),candidate(10.1,(10.5,0.,0.))),pose(10.1),10.1,0)
        self.assertEqual(result['ambiguous'],2)
        self.assertEqual(t.snapshot(10.1,0)[0].observations,1)

    def test_reject_future_pose_bracket_and_wrong_frame(self):
        t=GateTracker()
        with self.assertRaises(ValueError):
            t.ingest((candidate(10.),),replace(pose(10.),bracket_stamps=(9.99,10.01)),10.,0)
        bad=replace(candidate(10.),metadata=replace(candidate(10.).metadata,frame_id='world_ned'))
        with self.assertRaises(ValueError):t.ingest((bad,),pose(10.),10.,0)
        self.assertEqual(t.snapshot(10.,0),())

    def test_empty_scan_does_not_refresh_expired_track(self):
        t=GateTracker();t.ingest((candidate(10.),),pose(10.),10.,0)
        t.ingest((),pose(10.1),10.1,0);self.assertEqual(t.snapshot(10.2,0),())
        t.ingest((candidate(10.3),),pose(10.3),10.3,0)
        self.assertEqual(t.snapshot(10.3,0)[0].observations,2)

    def test_cross_epoch_stale_source_and_capacity_failure_are_atomic(self):
        t=GateTracker(capacity=1);t.ingest((candidate(10.),),pose(10.),10.,0)
        with self.assertRaises(ValueError):t.ingest((candidate(10.1,(20.,0.,0.)),),pose(10.1),10.1,0)
        self.assertEqual(t.snapshot(10.1,0)[0].observations,1)
        with self.assertRaises(ValueError):t.ingest((candidate(10.1),),pose(10.1),10.3,0)
        t.reset(1);self.assertEqual(t.snapshot(1.,1),())
        with self.assertRaises(ValueError):t.ingest((candidate(10.1),),pose(10.1),10.1,0)


if __name__ == '__main__':unittest.main()
