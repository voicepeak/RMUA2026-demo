import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from route_coordinates import RouteCoordinates,HeightProfile
from lidar_navigation import LidarNavigator


class RouteCoordinatesTests(unittest.TestCase):
    def test_world_route_round_trip_with_grade_and_lateral_offset(self):
        route=RouteCoordinates([10.,15.,20.],[[0.,0.,0.],[5.,0.,1.],[10.,0.,2.]])
        s=np.array([11.,13.,17.]);world=route.world(s,.5,-.25)
        np.testing.assert_allclose(route.project(world),np.column_stack([s,np.full(3,.5),np.full(3,-.25)]))

    def test_projection_and_tangent_on_a_bend(self):
        route=RouteCoordinates([0.,5.,10.],[[0.,0.,0.],[5.,0.,0.],[5.,5.,0.]])
        np.testing.assert_allclose(route.project([[2.,.5,0.],[4.5,3.,0.]]),[[2.,.5,0.],[8.,.5,0.]])
        np.testing.assert_allclose(route.frame([2.,8.])[0],[[1.,0.],[0.,1.]])

    def test_projection_preserves_out_of_window_station(self):
        route=RouteCoordinates([0.,5.],[[0.,0.,0.],[5.,0.,0.]])
        np.testing.assert_allclose(route.project([[-2.,0.,0.],[8.,0.,0.]])[:,0],[-2.,8.])
        self.assertTrue(np.all(route.violations([[-2.,0.,0.],[8.,0.,0.]])))

    def test_ned_floor_and_road_priors(self):
        route=RouteCoordinates([0.,5.],[[0.,0.,0.],[5.,0.,0.]],1.,1.,[.5,.5])
        self.assertEqual(route.violations([[2.,0.,.4],[2.,0.,.6],[2.,1.1,0.],[2.,0.,-1.1]]).tolist(),[False,True,True,True])

    def test_frozen_route_and_height_path_own_their_arrays(self):
        points=np.array([[0.,0.,0.],[5.,0.,1.]])
        route=RouteCoordinates([0.,5.],points);points[1,2]=9.
        self.assertEqual(route.points[1,2],1.)
        profile=HeightProfile(route)
        with self.assertRaises(ValueError):profile.path[0,0]=5.

    def test_height_feedback_matches_existing_stop_profile(self):
        route=RouteCoordinates([0.,5.,10.],[[0.,0.,0.],[5.,0.,1.],[7.,4.,2.]])
        profile=HeightProfile(route,height_gain=2.)
        legacy=LidarNavigator._stop_profile(profile.path,gain=2.)
        p=np.array([[1.,.1,0.],[5.5,1.2,.8],[6.8,3.,2.1]])
        v=np.array([[2.,0.,0.],[1.,2.,0.],[0.,1.,0.]])
        np.testing.assert_allclose(profile(p,v),legacy(p,v),atol=1e-12)

    def test_invalid_samples_and_zero_length_segments_are_rejected(self):
        for s,p in (([0.,0.],[[0.,0.,0.],[1.,0.,0.]]),([0.,1.],[[0.,0.,0.],[0.,0.,1.]]),
                    ([0.,1.],[[0.,0.,0.],[np.nan,0.,0.]])):
            with self.assertRaises(ValueError):RouteCoordinates(s,p)

    def test_straight_projection_optimization_matches_general_at_all_stations(self):
        from dataclasses import replace
        stations=np.array([-3.,0.,1.,5.,9.])
        arc=np.array([0.,2.,3.,7.,10.]);axis=np.array([.6,.8])
        points=np.column_stack([arc[:,None]*axis,np.array([0.,1.,.3,-1.,2.])])
        optimized=RouteCoordinates(stations,points);general=replace(optimized)
        object.__setattr__(general,'_straight',False)
        queries=np.random.default_rng(4).uniform([-8.,-8.,-3.],[15.,15.,3.],size=(1000,3))
        np.testing.assert_allclose(optimized.project(queries),general.project(queries),atol=1e-12)


if __name__=='__main__':unittest.main()
