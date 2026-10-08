import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from yaw_controller import YawController,image_u_error


class YawControllerTests(unittest.TestCase):
    def test_offscreen_gate_cannot_force_heading_away_from_visible_gate(self):
        controller=YawController();position=np.zeros(3);rotation=np.eye(3)
        visible=(10.,1.,0.);offscreen=(10.,-60.,0.)
        expected=image_u_error(visible,position,rotation)
        self.assertAlmostEqual(controller.vision_fov_error([visible,offscreen],position,rotation),expected)

    def test_no_visible_gate_leaves_road_heading_authority(self):
        controller=YawController();position=np.zeros(3);rotation=np.eye(3)
        self.assertEqual(controller.vision_fov_error([(10.,60.,0.),(-10.,0.,0.)],position,rotation),0.)
        self.assertEqual(controller.rate((0.,0.),0.,(10.,0.)),(0.,0.,0.))

    def test_visible_gate_errors_keep_original_average(self):
        controller=YawController();position=np.zeros(3);rotation=np.eye(3)
        gates=[(10.,1.,0.),(12.,-2.,0.)]
        expected=sum(image_u_error(g,position,rotation) for g in gates)/2.
        self.assertAlmostEqual(controller.vision_fov_error(gates,position,rotation),expected)
