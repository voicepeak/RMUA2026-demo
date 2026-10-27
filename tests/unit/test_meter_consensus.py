import unittest
from inspection.meter_reader.angular_scale import MeterCandidate
from inspection.meter_reader.consensus import ConsensusTracker
from interfaces.contracts import Metadata


class MeterConsensusTests(unittest.TestCase):
    def feed(self,t,stamp,device='a',lights=(True,True,True),value=12):
        meta=Metadata(stamp,stamp+.15,0,'test','image')
        return t.observe(meta,stamp,stamp,stamp,device,lights,MeterCandidate('CANDIDATE',value,11.8,12.2))

    def test_distinct_frames_are_needed_and_output_expires(self):
        tracker=ConsensusTracker();self.feed(tracker,1.);self.feed(tracker,1.1)
        with self.assertRaises(ValueError):self.feed(tracker,1.1)
        result=self.feed(tracker,1.2);self.assertEqual(result.status,'CONSENSUS')
        self.assertFalse(result.report_eligible);self.assertFalse(result.device_identity_verified)
        self.assertEqual(tracker.snapshot(1.35,1.35).status,'UNAVAILABLE')

    def test_identity_value_or_light_change_resets(self):
        for change in ({'device':'b'},{'value':13},{'lights':(True,None,True)}):
            tracker=ConsensusTracker();self.feed(tracker,1.);self.feed(tracker,1.1)
            result=self.feed(tracker,1.2,**change)
            self.assertNotEqual(result.status,'CONSENSUS')
            self.assertLess(result.observations,3)

    def test_reset_invalidates_prior_epoch(self):
        tracker=ConsensusTracker();self.feed(tracker,1.);tracker.reset(1)
        self.assertEqual(tracker.snapshot(1.,1.).status,'UNAVAILABLE')
        with self.assertRaises(ValueError):self.feed(tracker,1.1)


if __name__=='__main__':unittest.main()
