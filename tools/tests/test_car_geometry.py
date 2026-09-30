from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/rmua_gate_vision/scripts'))
from car_geometry import locate_car,overlaps_car


class CarGeometryTests(unittest.TestCase):
    def test_uniform_face_disparity_has_conservative_near_depth(self):
        car=dict(bbox=[20.,20.,80.,80.],confidence=.9)
        result=locate_car(car,np.full((100,100),20.),[0.,0.,0.],[0.,0.,0.,1.])
        self.assertTrue(result['geometry_valid'])
        self.assertLess(result['depth_near'],result['depth'])

    def test_inconsistent_background_depth_is_not_an_obstacle_position(self):
        disparity=np.full((100,100),2.);disparity[:,50:]=20.
        result=locate_car(dict(bbox=[20.,20.,80.,80.]),disparity,[0,0,0],[0,0,0,1])
        self.assertFalse(result['geometry_valid'])

    def test_car_inside_gate_does_not_remove_real_portal(self):
        self.assertFalse(overlaps_car(dict(bbox=[0,0,400,400]),[dict(bbox=[100,100,160,150])]))

    def test_car_face_cannot_be_added_as_a_gate(self):
        self.assertTrue(overlaps_car(dict(bbox=[100,100,60,50]),[dict(bbox=[100,100,160,150])]))
