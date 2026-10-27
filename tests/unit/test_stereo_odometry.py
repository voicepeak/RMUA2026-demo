import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from interfaces.contracts import Metadata
from perception.stereo_depth.geometry import load_nominal_rig
from perception.stereo_depth.pairing import ImageFrame, StereoPair
from perception.visual_odometry.stereo import estimate_stereo_motion, solve_relative


class StereoOdometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rig = load_nominal_rig(Path(__file__).resolve().parents[2] / 'configs/simulator_settings.json')

    def scene(self):
        rng = np.random.RandomState(7)
        return np.column_stack((rng.uniform(7, 18, 100), rng.uniform(-4, 4, 100), rng.uniform(-3, 3, 100)))

    def test_known_body_motion_and_camera_lever_arm(self):
        points = self.scene()
        yaw = .05
        rotation = np.array(((np.cos(yaw), -np.sin(yaw), 0.), (np.sin(yaw), np.cos(yaw), 0.), (0., 0., 1.)))
        displacement = np.array((.25, -.12, .06))
        current = (points - displacement) @ rotation
        uv, _, valid = self.rig.left.project_body(current)
        result, found_rotation, count, p95 = solve_relative(points[valid], uv[valid], self.rig)
        np.testing.assert_allclose(result, displacement, atol=.005)
        np.testing.assert_allclose(found_rotation, rotation, atol=.001)
        self.assertGreater(count, 50)
        self.assertLess(p95, .1)

    def test_outliers_are_rejected_and_collinear_geometry_fails(self):
        points = self.scene()
        uv, _, valid = self.rig.left.project_body(points)
        points, uv = points[valid], uv[valid]
        uv[:20] = np.roll(uv[:20], 7, axis=0)
        _, _, count, p95 = solve_relative(points, uv, self.rig)
        self.assertLess(count, len(points))
        self.assertLess(p95, .1)
        with self.assertRaisesRegex(ValueError, 'degenerate'):
            solve_relative(np.tile((10., 0., 0.), (20, 1)), np.tile((480., 360.), (20, 1)), self.rig)

    def test_untextured_inputs_and_epoch_mismatch_rejected(self):
        image = np.zeros((720, 960, 1), dtype=np.uint8)
        def pair(stamp, epoch=0):
            left = ImageFrame(Metadata(stamp, stamp+.2, epoch, 'left', 'front_left_image'), 'mono8', image)
            right = ImageFrame(Metadata(stamp, stamp+.2, epoch, 'right', 'front_right_image'), 'mono8', image)
            return StereoPair(Metadata(stamp, stamp+.2, epoch, 'pair', 'front_stereo'), left, right, 0.)
        with self.assertRaises(ValueError): estimate_stereo_motion(pair(1.), pair(1.1), self.rig)
        with self.assertRaisesRegex(ValueError, 'epoch or time'): estimate_stereo_motion(pair(1.), pair(1.1, 1), self.rig)
        with self.assertRaisesRegex(ValueError, 'epoch or time'): estimate_stereo_motion(pair(1.), pair(1.), self.rig)


if __name__ == '__main__':
    unittest.main()
