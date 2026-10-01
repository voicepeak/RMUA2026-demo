import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from execution_guard import ExecutionGuard
from predictive_avoidance import Detour, JoinedDetour, PredictiveAvoidance
from lattice_detour import LatticeDetour, search
from point_index import PointIndex
from reference_planner import RouteGeometry


class ExecutionGuardTests(unittest.TestCase):
    def setUp(self):
        self.guard=ExecutionGuard()
        self.xy=lambda s:(s,0.,0.)

    def check(self,points=(),plan=None,cloud=10.,pose=10.1,velocity=(0.,0.,0.),position=(0.,0.,0.)):
        return self.guard.evaluate(0.,np.array(position),np.array(velocity),self.xy,lambda s:0.,
                                   plan,np.array(points).reshape(-1,3),cloud,pose)

    def test_old_plan_can_continue_only_after_latest_geometry_validation(self):
        plan=Detour(-20.,(0.,0.),(2.,0.),10.,20.)
        safe=self.check([(12.,0.,0.)],plan=plan,position=(0.,2.,0.))
        self.assertTrue(safe['execution_verified'])
        self.assertGreater(safe['cap'],10.)
        self.guard=ExecutionGuard()
        blocked=self.check([(1.,2.,0.)],plan=plan,position=(0.,2.,0.))
        self.assertEqual(blocked['cap'],0.)

    def test_stale_missing_or_future_cloud_does_not_authorize_motion(self):
        for cloud in (None,9.,11.):
            result=self.check(cloud=cloud)
            self.assertEqual(result['cap'],0.)
            self.assertEqual(result['guard_reason'],'LIDAR_STALE')

    def test_empty_cloud_is_not_a_measured_free_corridor(self):
        result=self.check()
        self.assertEqual(result['cap'],0.)
        self.assertEqual(result['guard_reason'],'LIDAR_EMPTY')

    def test_braking_begins_before_obstacle_without_unnecessary_zero(self):
        result=self.check([(15.,0.,0.)])
        self.assertEqual(result['guard_reason'],'BRAKING_OBSTACLE')
        self.assertGreater(result['cap'],0.)
        d=result['verified_distance'];v=result['cap'];latency=.45
        self.assertLessEqual(v*latency+v*v/(2.*self.guard.braking),d+1e-6)

    def test_actual_tracking_connector_is_certified(self):
        self.assertEqual(self.check([(0.,1.8,0.)],position=(0.,2.,0.))['cap'],0.)

    def test_measured_response_segment_cannot_cross_surface(self):
        result=self.check([(0.,2.,0.)],velocity=(0.,8.,0.))
        self.assertEqual(result['cap'],0.)

    def test_delayed_plan_join_preserves_position_slope_and_acceleration(self):
        old=Detour(0.,(0.,0.),(2.,-1.),20.,30.)
        new=Detour(2.,old.offset(2.),(-2.,0.),20.,40.,start_slope=old.slope(2.))
        s=8.;accel=(old.slope(s+.1)-old.slope(s-.1))/.2
        joined=JoinedDetour(s,old.offset(s),old.slope(s),accel,new,8.)
        np.testing.assert_allclose(joined.offset(s),old.offset(s),atol=1e-10)
        np.testing.assert_allclose(joined.slope(s),old.slope(s),atol=1e-10)
        np.testing.assert_allclose(joined.offset(16.),new.offset(16.),atol=1e-10)
        np.testing.assert_allclose(joined.slope(16.),new.slope(16.),atol=1e-10)
        self.assertLess(np.linalg.norm(joined.slope(16.001)-joined.slope(15.999)),.001)

    def test_lattice_recenters_when_reachable_and_preserves_endpoint_slopes(self):
        stations=np.arange(16.)
        nominal=np.column_stack([stations,np.zeros((16,2))]);sides=np.tile([0.,1.,0.],(16,1))
        options=search(stations,nominal,sides,np.array([2.,1.]),PointIndex([]),1.,[],start_slope=(.12,-.08))
        for plan in options:
            np.testing.assert_allclose(plan.offset(15.),[0.,0.],atol=1e-10)
            np.testing.assert_allclose(plan.slope(15.),[0.,0.],atol=1e-10)
            np.testing.assert_allclose(plan.slope(0.),[.12,-.08],atol=1e-10)

    def test_finished_nonzero_lattice_is_returned_continuously(self):
        planner=PredictiveAvoidance()
        planner.plan=LatticeDetour([0.,5.],np.array([[1.,1.],[1.,1.]]))
        route=RouteGeometry([(0.,0.,0.),(100.,0.,0.)])
        result=planner.evaluate(6.,np.array([6.,1.,1.]),np.array([8.,0.,0.]),self.xy,lambda s:0.,
                                route,np.empty((0,3)),[],10.,8.)
        self.assertTrue(result['feasible'])
        np.testing.assert_allclose(planner.plan.offset(6.),[1.,1.],atol=1e-10)
        np.testing.assert_allclose(planner.plan.offset(45.),[0.,0.],atol=1e-10)

    def test_nonzero_terminal_is_retained_when_nominal_return_is_blocked(self):
        planner=PredictiveAvoidance()
        previous=LatticeDetour([0.,5.],np.array([[2.,0.],[2.,0.]]));planner.plan=previous
        route=RouteGeometry([(0.,0.,0.),(100.,0.,0.)])
        result=planner.evaluate(6.,np.array([6.,2.,0.]),np.array([8.,0.,0.]),self.xy,lambda s:0.,
                                route,np.array([[16.,0.,0.]]),[],10.,8.)
        self.assertTrue(result['feasible']);self.assertIs(planner.plan,previous)


if __name__=='__main__':unittest.main()
