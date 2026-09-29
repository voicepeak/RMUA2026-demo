import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from reference_planner import RouteGeometry,ReferencePlanner
from route_height_prior import RouteHeightPrior

class HeightPriorTests(unittest.TestCase):
    def setUp(self):
        pts=[(i*20.,.002*(i*20.)**2,.0005*(i*20.)**2) for i in range(31)]
        self.route=RouteGeometry(pts)
        self.prior=RouteHeightPrior(self.route)
        self.gates=[]
        for i in range(2,15):
            s=self.route.seg_s[i];x,y,_=self.route.point_at(s)
            z=-self.prior.height.center(s)-.05*x+.01*y+.8
            self.gates.append(dict(id=i,s=s,x=x,y=y,z=z,valid=True,trusted=True))

    def test_no_model_without_sufficient_geometry(self):
        self.assertFalse(self.prior.fit(self.gates[:4]))

    def test_calibrates_tilt_and_rejects_false_far_gate(self):
        self.assertTrue(self.prior.fit(self.gates))
        s=self.route.seg_s[17];x,y,_=self.route.point_at(s)
        true=-self.prior.height.center(s)-.05*x+.01*y+.8
        self.assertAlmostEqual(self.prior.center(s),true,delta=.1)
        self.assertFalse(self.prior.consistent(s,true-10.))

    def test_rejects_inconsistent_route_geometry(self):
        for i,g in enumerate(self.gates): g['z']+=5.*(-1)**i
        self.assertFalse(self.prior.fit(self.gates))

    def test_planner_does_not_follow_false_height_at_hilltop(self):
        planner=ReferencePlanner(self.route,snap_gate_to_route=False)
        s=self.gates[-1]['s']+50.
        self.prior.fit(self.gates)
        predicted=self.prior.center(s)
        _,profile=planner.build(self.gates,[],[(s,predicted-10.)],200.,0.)
        self.assertTrue(planner.height_prior.valid)
        self.assertAlmostEqual(profile.center(s),predicted,delta=.2)

    def test_reference_continues_beyond_speed_horizon(self):
        planner=ReferencePlanner(self.route,snap_gate_to_route=False)
        _,profile=planner.build(self.gates,[],[],200.,0.)
        h=planner.trend_horizon
        self.assertLess(profile.dz_ds(h+10.),-.1)

if __name__=='__main__':unittest.main()
