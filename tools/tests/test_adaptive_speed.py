import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from speed_scheduler import SpeedScheduler


class AdaptiveSpeedTests(unittest.TestCase):
    def test_open_road_can_exceed_fifteen_with_stopping_room(self):
        s=SpeedScheduler(cruise_speed=40,max_speed=40,adaptive_speed=True)
        cruise=s.set_visibility(30.,.05)
        self.assertGreater(cruise,15.)
        self.assertLessEqual(cruise*(.35+.05)+cruise**2/16.,26.+1e-8)

    def test_delayed_or_shorter_visibility_reduces_cruise(self):
        s=SpeedScheduler(cruise_speed=40,max_speed=40,adaptive_speed=True)
        fast=s.set_visibility(30.,0.)
        delayed=s.set_visibility(30.,.4)
        short=s.set_visibility(10.,.4)
        self.assertLess(delayed,fast)
        self.assertLess(short,delayed)

    def test_fixed_speed_mode_remains_fixed(self):
        s=SpeedScheduler(cruise_speed=12,max_speed=12)
        self.assertEqual(s.set_visibility(30.),12.)

    def test_straight_road_and_sharp_bend_choose_different_speeds(self):
        s=SpeedScheduler(cruise_speed=40,max_speed=40,adaptive_speed=True)
        s.set_visibility(30.)
        common=dict(kz=0.,slope_trusted=True,miss_ratio=None,z_err=0.,
                    z_worsening=False,pred_worse=False,yaw_err_deg=0.)
        fast,_=s.target(kap=0.,**common)
        slow,_=s.target(kap=.15,**common)
        self.assertGreater(fast,15.)
        self.assertLess(slow,7.)


if __name__=='__main__':unittest.main()
