import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from predictive_avoidance import CarTracks,Detour,PredictiveAvoidance,departure_floor_limits
from reference_planner import RouteGeometry
from altitude_profile import AltitudeProfile
from terrain_speed import TerrainSpeedEnvelope
from pose_history import PoseHistory
from lidar_points import valid_points
from point_index import PointIndex
from async_planner import AsyncPlanner
import threading


class PredictiveRouteTests(unittest.TestCase):
    def test_departure_floor_shares_measured_bound_and_ends_at_20m(self):
        class Floor:
            def floor_limit(self,positions,margin):return np.array([np.nan,-1.,np.nan])
        limits=departure_floor_limits(np.array([0.,10.,20.]),np.zeros((3,3)),
                                     lambda s:-.1*s,.483,Floor(),1.25)
        np.testing.assert_allclose(limits[:2],[.483,-1.])
        self.assertTrue(np.isinf(limits[2]))

    def test_departure_detour_respects_floor_between_vertical_grid_steps(self):
        p=PredictiveAvoidance(margin=1.15,budget=3.)
        points=np.array([(x,y,z) for x in (9.,10.,11.)
                         for y in np.arange(-.8,.81,.2) for z in np.arange(-.8,.81,.2)])
        info=p.evaluate(0.,np.zeros(3),np.zeros(3),self.xy,lambda s:0.,self.route,
                        points,[],1.,1.,departure_floor_offset=.483)
        self.assertTrue(info['feasible'])
        path=p.positions(np.arange(0.,20.,.1),self.xy,lambda s:0.,p.plan)
        self.assertTrue(np.all(path[:,2]<=.483))
        self.assertGreaterEqual(info['selected_clearance'],1.25)

    def setUp(self):
        self.route=RouteGeometry([(0.,0.,0.),(100.,0.,0.)])
        self.xy=lambda s:(s,0.,0.)

    def evaluate(self,p,points=(),tracks=(),s=0.):
        return p.evaluate(s,np.array([s,0.,0.]),np.array([15.,0.,0.]),self.xy,
                          lambda s:0.,self.route,np.asarray(points).reshape(-1,3),tracks,1.,15.)

    def test_far_stereo_car_starts_detour_outside_lidar_range(self):
        p=PredictiveAvoidance()
        track=dict(world=np.array([42.,0.,0.]),stamp=1.,uncertainty=.5,half_width=.8,half_height=.8)
        result=self.evaluate(p,tracks=[track])
        self.assertTrue(result['feasible']);self.assertTrue(result['active'])
        self.assertEqual(result['cars'],1)
        self.assertGreater(result['cap'],10.)
        self.assertAlmostEqual(np.linalg.norm(p.plan.offset(0.)),0.)
        self.assertGreater(np.linalg.norm(p.plan.offset(20.)),1.)
        path=p.positions(np.arange(0.,66.,.5),self.xy,lambda s:0.,p.plan)
        nearby=path[abs(path[:,0]-42.)<4.5]
        self.assertTrue(np.all(abs(nearby[:,1])>=2.05))

    def test_radar_checks_moving_entry_not_just_displaced_parallel_ray(self):
        # An offset parallel ray at y=2 is clear, but a late ramp into it hits
        # this near wall. A predictive plan must reject that entry.
        p=PredictiveAvoidance()
        points=[(x,y,z) for x in (1.,2.,3.) for y in np.arange(-3.,3.1,.3) for z in np.arange(-2.,2.1,.3)]
        result=self.evaluate(p,points)
        self.assertFalse(result['feasible']);self.assertEqual(result['cap'],0.)

    def test_keep_spatial_ramp_across_replans(self):
        p=PredictiveAvoidance()
        points=[(x,y,z) for x in (19.,20.,21.) for y in np.arange(-.8,.81,.2) for z in np.arange(-.8,.81,.2)]
        self.assertTrue(self.evaluate(p,points)['feasible'])
        initial=p.plan
        self.assertTrue(self.evaluate(p,points,s=.5)['feasible'])
        self.assertIs(p.plan,initial)
        self.assertGreater(np.linalg.norm(p.plan.offset(10.)),np.linalg.norm(p.plan.offset(1.)))

    def test_detour_position_slope_and_acceleration_are_continuous(self):
        p=Detour(0.,(0.,0.),(2.,-1.),25.,40.)
        for s in (0.,25.,40.,62.):
            self.assertLess(np.linalg.norm(p.offset(s+.001)-p.offset(s-.001)),.001)
            self.assertLess(np.linalg.norm(p.slope(s+.001)-p.slope(s-.001)),.001)
        self.assertAlmostEqual(np.linalg.norm(p.offset(70.)),0.)

    def test_car_tracks_require_multiple_frames_and_expire(self):
        p=CarTracks();d=dict(geometry_valid=True,confidence=.8,world=[40.,0.,0.],depth=40.,depth_near=39.)
        p.update(.5,[d]);self.assertEqual(p.snapshot(.6),[])
        p.update(.7,[d]);self.assertEqual(len(p.snapshot(.8)),1)
        self.assertEqual(p.snapshot(3.),[])
        d['world']=[float('nan'),0.,0.];p.update(3.1,[d]);self.assertEqual(p.snapshot(3.2),[])

    def test_road_height_does_not_reverse_to_touch_each_gate_center(self):
        gates=[dict(s=19.,z=11.,valid=True)]
        guides=[dict(s=s,z=.5*s) for s in (0.,20.,40.)]
        old=AltitudeProfile(0.,0.,40.,20.,gates,guides)
        new=AltitudeProfile(0.,0.,40.,20.,gates,guides,corridor_guidance=True)
        self.assertLess(old.dz_ds(19.5),0.)
        self.assertTrue(all(new.dz_ds(s)>0. for s in np.arange(5.,35.,.1)))
        self.assertLessEqual(abs(new.center(19.)-11.),.851)

    def test_terrain_braking_precedes_slope_and_ignores_height_tracking_ripple(self):
        class Cap:
            def up(self,v):return 4.
            def down(self,v):return 4.5
        env=TerrainSpeedEnvelope()
        center=lambda s:max(0.,s-30.)*.5
        before=env.evaluate(0.,center,Cap(),17.)
        approach=env.evaluate(24.,center,Cap(),17.)
        on=env.evaluate(40.,center,Cap(),17.)
        self.assertGreater(before['v_climb_preview'],approach['v_climb_preview'])
        self.assertLess(approach['v_climb_preview'],17.)
        self.assertAlmostEqual(on['v_climb_preview'],.95*4.5/.5)

    def test_pose_interpolation_removes_high_speed_cloud_offset(self):
        p=PoseHistory();p.add(1.,[0.,0.,0.],[0.,0.,0.,1.]);p.add(1.1,[2.,0.,0.],[0.,0.,0.,1.])
        position,q=p.sample(1.025)
        self.assertAlmostEqual(position[0],.5)
        self.assertIsNone(p.sample(1.2))
        self.assertIsNotNone(p.sample(1.1))

    def test_zero_range_returns_are_removed_before_sensor_offset(self):
        cloud=np.array([[0.,0.,0.],[float('nan'),1.,0.],[.001,0.,0.],[.3,0.,0.],[20.,1.,2.]])
        filtered=valid_points(cloud)
        self.assertEqual(filtered.shape,(3,3))
        self.assertAlmostEqual(filtered[0,0],.001)
        self.assertAlmostEqual(filtered[1,0],.3)

    def test_exact_point_index_matches_brute_force(self):
        rng=np.random.RandomState(123);cloud=rng.normal(size=(200,3));query=rng.normal(size=(35,3))
        expected=np.sqrt(np.min(np.sum((cloud[None,:,:]-query[:,None,:])**2,axis=2),axis=1))
        np.testing.assert_allclose(PointIndex(cloud).distance(query),expected,atol=1e-6)

    def test_near_radar_refines_overlapping_stereo_volumes(self):
        p=PredictiveAvoidance()
        tracks=[dict(world=np.array([10.,y,0.]),stamp=1.,uncertainty=1.,half_width=1.1,half_height=.8) for y in (-2.2,2.2)]
        points=[(x,y,z) for x in (9.,10.,11.) for y in (-2.5,-2.2,2.2,2.5) for z in (-.5,0.,.5)]
        result=self.evaluate(p,points,tracks)
        self.assertTrue(result['feasible']);self.assertFalse(result['active'])
        self.assertEqual(result['cars'],2);self.assertIsNone(result['cap'])

    def test_lattice_can_pass_two_obstacles_on_opposite_sides(self):
        # Geometry feasibility must not depend on host/UE4 CPU contention.
        # Production deadline behavior is exercised separately below.
        p=PredictiveAvoidance(budget=5.)
        points=[(x,y,z) for xs,ys in [(np.arange(15.,21.1,.5),np.arange(-3.,.51,.5)),
                                      (np.arange(29.,35.1,.5),np.arange(-.5,3.1,.5))]
                for x in xs for y in ys for z in np.arange(-2.,2.1,.5)]
        result=self.evaluate(p,points)
        self.assertTrue(result['feasible'])
        self.assertGreater(p.plan.offset(18.)[0],1.5)
        self.assertLess(p.plan.offset(32.)[0],-1.5)

    def test_expired_search_budget_reports_timeout_and_stops_search(self):
        p=PredictiveAvoidance(budget=0.)
        result=self.evaluate(p,[(10.,0.,0.)])
        self.assertFalse(result['feasible'])
        self.assertTrue(result['search_timeout'])

    def test_planner_does_not_block_control_or_duplicate_pending_jobs(self):
        planner=AsyncPlanner();release=threading.Event()
        def job():
            release.wait(1.)
            return dict(feasible=True),None
        try:
            self.assertTrue(planner.submit(12.,job))
            self.assertIsNone(planner.poll())
            self.assertFalse(planner.submit(13.,job))
            release.set();planner.future.result(timeout=1.)
            info,plan=planner.poll();self.assertEqual(info['planned_stamp'],12.)
        finally:release.set();planner.close()

    def test_reset_discards_previous_course_planner_result(self):
        planner=AsyncPlanner();release=threading.Event()
        def job():
            release.wait(1.)
            return dict(feasible=True),None
        try:
            planner.submit(12.,job);planner.reset();release.set()
            planner.future.result(timeout=1.)
            self.assertIsNone(planner.poll())
        finally:release.set();planner.close()


if __name__=='__main__':unittest.main()
