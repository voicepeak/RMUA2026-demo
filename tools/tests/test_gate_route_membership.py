import sys,unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from reference_planner import RouteGeometry
from gate_route_membership import gate_route_membership


class GateRouteMembershipTests(unittest.TestCase):
    def setUp(self):
        self.route=RouteGeometry([(0.,0.,100.),(100.,0.,120.)])
        self.prior=SimpleNamespace(valid=True,uncertainty=lambda s:.3,center=lambda s:-30.-.2*s)
        self.gate=dict(x=50.,y=0.,z=-40.,sigma_z=.3,geometry_valid=True,nx=1.,ny=0.,nz=-.2)

    def check(self,g):
        return gate_route_membership(g,self.route,self.prior,20.)

    def test_rejects_lower_crossing_road_before_either_height_cache(self):
        self.assertEqual(self.check(dict(self.gate,z=-32.)),(False,'OTHER_LAYER'))
        self.assertEqual(self.check(dict(self.gate,z=-40.)),(True,'ACCEPTED'))

    def test_uses_calibrated_ned_height_and_allows_sloped_road(self):
        self.assertTrue(self.check(self.gate)[0])
        self.assertTrue(self.check(dict(self.gate,z=-38.))[0])

    def test_unvalidated_raw_spline_z_cannot_reject_a_gate_layer(self):
        self.prior.valid=False
        self.assertTrue(self.check(dict(self.gate,z=-32.))[0])
        self.prior.valid=True;self.prior.uncertainty=lambda s:1.2
        self.assertTrue(self.check(dict(self.gate,z=-32.))[0])

    def test_measured_crossing_plane_is_rejected_but_bbox_normal_is_not_evidence(self):
        g=dict(self.gate,nx=0.,ny=1.,nz=0.)
        self.assertEqual(self.check(g),(False,'OTHER_DIRECTION'))
        self.assertTrue(self.check(dict(g,geometry_valid=False))[0])

    def test_separate_road_and_passed_gate_are_rejected(self):
        self.assertEqual(self.check(dict(self.gate,y=4.)),(False,'OTHER_ROAD'))
        self.assertEqual(self.check(dict(self.gate,x=10.,z=-32.)),(False,'BEHIND'))


if __name__=='__main__':unittest.main()
