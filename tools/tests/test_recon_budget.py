import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from mission_state import MissionState


class ReconBudgetTests(unittest.TestCase):
    def test_delay_and_braking_share_the_known_visibility_budget(self):
        mission=MissionState(map_brake_a=4.)
        for distance in (.01,1.,5.,20.,100.):
            for delay in (.1,.35,.8):
                mode,speed,_=mission.update(True,distance-3.,0.,-2.,reaction=delay)
                if mode=='TRACK':
                    self.assertLessEqual(speed*delay+speed**2/8.,distance+1e-8)
        fast=mission.stopping_speed(20.,.35)
        self.assertGreater(fast,MissionState().stopping_speed(20.))
        self.assertLess(mission.stopping_speed(20.,.8),fast)

    def test_known_gate_search_brakes_at_its_limit_with_delay(self):
        mission=MissionState(map_brake_a=4.,gate_search_speed=10.)
        mission.update(True,1000.,0.,-2.,require_gate=True,reaction=.35)
        mode,speed,_=mission.update(True,1000.,249.,-2.,require_gate=True,reaction=.35)
        self.assertEqual(mode,'RECON')
        self.assertLessEqual(speed*.35+speed**2/8.,1.+1e-8)
        self.assertEqual(mission.update(True,1000.,250.,-2.,require_gate=True,reaction=.35)[:2],('HOLD',0.))

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
