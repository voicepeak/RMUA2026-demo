import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from mission_state import MissionState


class ReconBudgetTests(unittest.TestCase):
    def test_faster_search_brakes_inside_the_original_distance_budget(self):
        mission=MissionState(recon_speed=4.,recon_max_dist=30.,map_brake_a=1.5)
        mode,speed,_=mission.update(True,None,100.,-2.)
        self.assertEqual((mode,speed),('RECON',4.))
        mode,speed,_=mission.update(True,None,129.,-2.)
        self.assertEqual(mode,'RECON')
        self.assertLessEqual(speed**2/(2.*mission.map_brake_a),1.+1e-8)
        self.assertEqual(mission.update(True,None,130.,-2.)[:2],('HOLD',0.))

    def test_unknown_gate_search_keeps_the_same_bound(self):
        mission=MissionState(recon_speed=4.,recon_max_dist=30.)
        mission.update(True,None,0.,-2.,require_gate=True)
        mode,speed,_=mission.update(True,None,29.,-2.,require_gate=True)
        self.assertEqual(mode,'RECON')
        self.assertLess(speed,4.)
        self.assertEqual(mission.update(True,None,30.,-2.,require_gate=True)[:2],('HOLD',0.))
