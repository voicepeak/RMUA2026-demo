import sys
import unittest
import numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from lidar_clearance import LidarClearance

class ClearanceTests(unittest.TestCase):
    def test_preview_includes_braking_distance_at_15(self):
        self.assertGreater(LidarClearance.preview_distance(15.),15.*15./8.+.5*15.)

    def test_aligned_displaced_path_restores_more_than_two_metres_per_second(self):
        cap,aligned=LidarClearance.progress_cap(dict(active=True,feasible=True,cap=0.,
                                                   obstacle_distance=.5,path_cap=8.),1.,0.,1.,0.)
        self.assertTrue(aligned)
        self.assertEqual(cap,8.)

    def test_alignment_inside_buffer_does_not_resume_forward_motion(self):
        cap,aligned=LidarClearance.progress_cap(dict(active=True,feasible=True,cap=0.,
                                                   recovery=True,path_cap=1.),1.,0.,1.,0.)
        self.assertFalse(aligned)
        self.assertEqual(cap,0.)

    def test_buffer_recovery_moves_away_before_following_road(self):
        # An overhead car is already 0.75 m away. Moving down restores 1 m;
        # a forward ray alone would remain inside the margin and stop forever.
        points=np.array([(x,y,-.75) for x in np.arange(-1.,2.1,.1)
                         for y in np.arange(-2.,2.1,.2)])
        reference=[(0.,0.,0.),(3.,0.,0.),(6.,0.,0.),(12.,0.,0.)]
        result=LidarClearance().evaluate_path(points,reference)
        self.assertTrue(result['feasible'])
        self.assertTrue(result['recovery'])
        self.assertGreater(result['vertical'],0.)
        self.assertGreaterEqual(result['selected_clearance'],.75-1e-6)
        self.assertGreaterEqual(result['shift_clearance'],1.)
        self.assertLessEqual(result['path_cap'],1.)

    def test_path_checker_does_not_escape_through_closed_wall(self):
        points=np.array([(3.,y,z) for y in np.arange(-10.,10.,.2)
                         for z in np.arange(-6.,6.,.2)])
        result=LidarClearance().evaluate_path(points,[(0.,0.,0.),(6.,0.,0.),(12.,0.,0.)])
        self.assertFalse(result['feasible'])
        self.assertEqual(result['cap'],0.)

    def test_path_checks_surfaces_behind_during_sideways_shift(self):
        front=[(4.,y,z) for y in np.arange(-3.,3.,.2) for z in np.arange(-3.,3.,.2)]
        rear=[(-.2,y,z) for y in np.arange(-4.,4.,.2) for z in np.arange(-4.,4.,.2)]
        result=LidarClearance().evaluate_path(front+rear,[(0.,0.,0.),(6.,0.,0.),(12.,0.,0.)])
        self.assertFalse(result['feasible'])

    def test_aligned_clear_ray_allows_progress_past_car(self):
        cap,aligned=LidarClearance.progress_cap(dict(active=True,feasible=True,cap=0.,
                                                   obstacle_distance=1.1),0.,.9,0.,1.)
        self.assertTrue(aligned)
        self.assertGreater(cap,0.)
        self.assertLessEqual(cap,2.)

    def test_unfinished_alignment_keeps_braking(self):
        cap,aligned=LidarClearance.progress_cap(dict(active=True,feasible=True,cap=0.,
                                                   obstacle_distance=1.1),0.,0.,0.,1.)
        self.assertFalse(aligned)
        self.assertEqual(cap,0.)

    def test_blocked_ray_does_not_authorize_forward_motion(self):
        cap,aligned=LidarClearance.progress_cap(dict(active=True,feasible=False,cap=0.,
                                                   obstacle_distance=1.1),0.,1.,0.,1.)
        self.assertFalse(aligned)
        self.assertEqual(cap,0.)

    def test_stale_clear_ray_does_not_authorize_forward_motion(self):
        cap,aligned=LidarClearance.progress_cap(dict(active=True,feasible=True,stale=True,
                                                   cap=0.,obstacle_distance=1.1),0.,1.,0.,1.)
        self.assertFalse(aligned)
        self.assertEqual(cap,0.)
    def test_door_floor_prompts_small_upward_offset(self):
        points=np.array([(8.,y,z) for y in np.arange(-4.,4.01,.2) for z in np.arange(.2,3.,.2)])
        result=LidarClearance().evaluate(points,(12.,0.,0.))
        self.assertTrue(result['active'])
        self.assertLess(result['vertical'],0.)
        self.assertAlmostEqual(result['lateral'],0.)
        self.assertGreater(result['cap'],0.)

    def test_safe_opening_does_not_force_center_alignment(self):
        points=np.array([(8.,y,z) for y in np.arange(-5.,5.,.2) for z in (-3.,3.)])
        result=LidarClearance().evaluate(points,(12.,.7,0.))
        self.assertFalse(result['active'])

    def test_obstacle_behind_is_not_a_reason_to_turn_back(self):
        points=np.array([(-1.,y,z) for y in np.arange(-2.,2.,.2) for z in (-.2,.2)])
        self.assertFalse(LidarClearance().evaluate(points,(12.,0.,0.))['active'])

    def test_closed_wall_requires_stop_even_if_best_gap_is_too_small(self):
        points=np.array([(3.,y,z) for y in np.arange(-12.,12.,.2)
                         for z in np.arange(-8.,8.,.2)])
        result=LidarClearance().evaluate(points,(12.,0.,0.))
        self.assertTrue(result['active'])
        self.assertEqual(result['cap'],0.)

    def test_imminent_wall_does_not_disappear_inside_half_metre(self):
        points=np.array([(.3,y,z) for y in np.arange(-3.,3.,.1)
                         for z in np.arange(-2.,2.,.1)])
        result=LidarClearance().evaluate(points,(8.,0.,0.))
        self.assertTrue(result['active'])
        self.assertEqual(result['cap'],0.)
