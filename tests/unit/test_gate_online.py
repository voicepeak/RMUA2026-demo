import unittest
from types import SimpleNamespace

from interfaces.contracts import Metadata
from perception.gate_geometry.online import GateWindow


class GateOnlineTests(unittest.TestCase):
    def pair(self):return SimpleNamespace(metadata=Metadata(10.,10.2,0,'pair','front_stereo'))

    def test_stale_after_depth_is_rejected_before_candidates(self):
        clocks=iter([(10.,0,True),(10.21,0,True)])
        window=GateWindow(None,depth_fn=lambda p:'depth',candidates_fn=lambda *args: self.fail('must not call'))
        row=window.process(self.pair(),20.,lambda:next(clocks),lambda:20.)
        self.assertEqual(row['reason'],'AFTER_DEPTH_SOURCE_LEASE_NOT_CURRENT')
        self.assertFalse(row['control_eligible'])

    def test_empty_timely_result_has_original_lease(self):
        window=GateWindow(None,depth_fn=lambda p:'depth',candidates_fn=lambda *args:())
        row=window.process(self.pair(),20.,lambda:(10.1,0,True),lambda:20.1)
        self.assertEqual(row['status'],'GATE_CANDIDATES')
        self.assertTrue(row['timing_current']);self.assertEqual(row['source_valid_until'],10.2)
        self.assertFalse(row['control_eligible'])

    def test_output_can_expire_during_candidate_extraction(self):
        clocks=iter([(10.,0,True),(10.1,0,True),(10.21,0,True)])
        window=GateWindow(None,depth_fn=lambda p:'depth',candidates_fn=lambda *args:())
        row=window.process(self.pair(),20.,lambda:next(clocks),lambda:20.)
        self.assertFalse(row['timing_current']);self.assertEqual(row['timing_rejection'],'SOURCE_LEASE_NOT_CURRENT')


if __name__ == '__main__':unittest.main()
