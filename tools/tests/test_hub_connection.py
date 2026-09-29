import sys
import unittest
import math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from splines_to_yaml import connect_routes,load_splines

class HubConnectionTests(unittest.TestCase):
    def test_approach_continues_through_door_before_turning(self):
        a=[(0.,0.,0.),(10.,0.,0.)];b=[(100.,100.,0.),(100.,110.,0.)]
        route=connect_routes(a,b)
        for p in route[2:]:
            if p[0]<=35.:self.assertAlmostEqual(p[1],0.)
            else:break
        self.assertEqual(route[-2:],b)

    def test_real_course_join_has_no_long_unplanned_chord(self):
        config=Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/config/splines.txt'
        splines=load_splines(config)
        route=connect_routes(splines[0],list(reversed(splines[2])))
        self.assertLess(max(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(route,route[1:])),10.)

if __name__=='__main__':unittest.main()
