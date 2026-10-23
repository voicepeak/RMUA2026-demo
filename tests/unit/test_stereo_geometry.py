from dataclasses import FrozenInstanceError, replace
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from perception.stereo_depth.geometry import load_nominal_rig, SensorPose
from perception.stereo_depth.epipolar import measure_sparse_epipolar


SETTINGS = Path(__file__).resolve().parents[2] / 'configs/simulator_settings.json'


class StereoGeometryTests(unittest.TestCase):
    def test_configuration_provenance_and_nominal_flags(self):
        rig = load_nominal_rig(SETTINGS)
        self.assertEqual(rig.settings_sha256, hashlib.sha256(SETTINGS.read_bytes()).hexdigest())
        self.assertAlmostEqual(rig.left.fx, 960 / (2 * np.tan(np.pi / 6)))
        self.assertAlmostEqual(rig.baseline_m, .3)
        self.assertFalse(rig.metric_calibration_verified)
        self.assertFalse(rig.current_control_eligible)
        with self.assertRaises(FrozenInstanceError): rig.metric_calibration_verified = True

    def test_front_projection_and_triangulation_recover_known_body_points(self):
        rig = load_nominal_rig(SETTINGS)
        body = np.array(((10., 0., 0.), (8., 1., .3), (5., -.5, -.3)))
        left, z, vl = rig.left.project_body(body)
        right, _, vr = rig.right.project_body(body)
        self.assertTrue(vl.all() and vr.all())
        np.testing.assert_allclose(z, body[:, 0] - .175)
        recovered, valid, sensitivity = rig.triangulate_body(left, right)
        self.assertTrue(valid.all())
        np.testing.assert_allclose(recovered, body, atol=1e-12)
        np.testing.assert_allclose(sensitivity, z ** 2 / (rig.left.fx * .3))

    def test_rear_camera_rotation_and_baseline_keep_left_right_sign(self):
        rig = load_nominal_rig(SETTINGS, 'back')
        body = np.array(((-10., 0., 0.), (-8., 1., .3)))
        left, _, vl = rig.left.project_body(body)
        right, _, vr = rig.right.project_body(body)
        self.assertTrue(vl.all() and vr.all())
        self.assertTrue((left[:, 0] > right[:, 0]).all())
        recovered, valid, _ = rig.triangulate_body(left, right)
        self.assertTrue(valid.all())
        np.testing.assert_allclose(recovered, body, atol=1e-12)

    def test_lidar_sensor_offset_and_optical_axes_are_explicit(self):
        rig = load_nominal_rig(SETTINGS)
        body = rig.lidar_pose.to_body(np.array(((2., 0., .3),)))
        np.testing.assert_allclose(body, ((2., 0., .25),))
        optical = rig.left.body_to_optical(body)
        np.testing.assert_allclose(optical, ((.15, .25, 1.825),))
        np.testing.assert_allclose(rig.left.optical_to_body(optical), body)
        np.testing.assert_allclose(rig.lidar_pose.from_body(body), ((2., 0., .3),))

    def test_bad_projection_disparity_vertical_error_and_distance_stay_unknown(self):
        rig = load_nominal_rig(SETTINGS)
        with np.errstate(all='raise'):
            uv, _, valid = rig.left.project_body(np.array(((-1., 0., 0.), (1., 100., 0.), (np.nan, 0., 0.), (np.inf, 0., 0.))))
        self.assertFalse(valid.any())
        self.assertTrue(np.isnan(uv).all())
        left = np.array(((480., 360.),) * 5)
        right = np.array(((480., 360.), (481., 360.), (470., 365.), (479., 360.), (np.nan, 360.)))
        with np.errstate(all='raise'):
            body, valid, sensitivity = rig.triangulate_body(left, right)
        self.assertFalse(valid.any())
        self.assertTrue(np.isnan(body).all() and np.isnan(sensitivity).all())

    def test_nonparallel_or_incompatible_configuration_is_rejected(self):
        original = json.loads(SETTINGS.read_text())
        for mutation in ('yaw', 'width', 'baseline', 'lidar_frame', 'duplicate_scene'):
            data = json.loads(json.dumps(original))
            vehicle = data['Vehicles']['drone_1']; camera = vehicle['Cameras']['front_right']
            if mutation == 'yaw': camera['Yaw'] = 1.
            if mutation == 'width': camera['CaptureSettings'][0]['Width'] = 640
            if mutation == 'baseline': camera['Y'] = -.15
            if mutation == 'lidar_frame': vehicle['Sensors']['lidar']['DataFrame'] = 'VehicleInertialFrame'
            if mutation == 'duplicate_scene': camera['CaptureSettings'].append(camera['CaptureSettings'][0])
            with tempfile.TemporaryDirectory() as root:
                path = Path(root) / 'settings.json';path.write_text(json.dumps(data))
                with self.assertRaises(ValueError): load_nominal_rig(path)

    def test_immutable_pose_rejects_reflections_and_nonunit_rotation(self):
        for rows in (((-1., 0., 0.), (0., 1., 0.), (0., 0., 1.)),
                     ((2., 0., 0.), (0., 1., 0.), (0., 0., 1.))):
            with self.assertRaises(ValueError): SensorPose((0., 0., 0.), rows)


class SparseEpipolarTests(unittest.TestCase):
    def images(self, vertical=0):
        rng = np.random.RandomState(44)
        left = rng.randint(0, 256, (240, 400)).astype(np.uint8)
        right = np.roll(left, (-vertical, -12), axis=(0, 1))
        return left, right

    def test_known_horizontal_and_vertical_shifts_are_measured_without_vertical_prefilter(self):
        for vertical in (0, 8):
            result, left, right, mask, fundamental = measure_sparse_epipolar(*self.images(vertical))
            self.assertGreater(result['reciprocal_ratio_matches'], 100)
            self.assertAlmostEqual(result['vertical_residual_all_p50_p95_p99_max_px'][0], vertical, delta=.2)
            self.assertFalse(result['vertical_prefilter_applied'])
            self.assertFalse(result['rectification_verified'])
            self.assertEqual(left.shape, right.shape)

    def test_blank_images_have_no_invented_correspondences(self):
        image = np.zeros((240, 400), np.uint8)
        result, left, right, mask, fundamental = measure_sparse_epipolar(image, image)
        self.assertEqual(result['reciprocal_ratio_matches'], 0)
        self.assertEqual(result['fundamental_inliers'], 0)
        self.assertIsNone(result['vertical_residual_all_p50_p95_p99_max_px'])
        self.assertIsNone(fundamental)

    def test_incompatible_images_are_rejected(self):
        with self.assertRaises(ValueError):measure_sparse_epipolar(np.zeros((4,4,3),np.uint8), np.zeros((4,4,3),np.uint8))


if __name__ == '__main__':
    unittest.main()
