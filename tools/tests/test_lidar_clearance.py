import sys
import unittest
import numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from lidar_clearance import LidarClearance

class ClearanceTests(unittest.TestCase):
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
