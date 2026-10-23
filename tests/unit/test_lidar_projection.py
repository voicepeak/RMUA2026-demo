from dataclasses import replace
from pathlib import Path
import unittest

import numpy as np

from estimation.geometry import delta_quaternion
from estimation.history import HistoricalPose
from perception.stereo_depth.geometry import load_nominal_rig
from perception.stereo_depth.lidar_projection import project_lidar_into_stereo, sparse_visible, sample_disparity


SETTINGS = Path(__file__).resolve().parents[2] / 'configs/simulator_settings.json'


def pose(stamp, position=(0., 0., 0.), yaw=0.):
    return HistoricalPose(stamp, 0, position, tuple(delta_quaternion((0., 0., yaw))),
                          (stamp, stamp), ((.01,)*15, (.01,)*15), 0., 0., 0.)


class LidarProjectionTests(unittest.TestCase):
    def test_estimated_header_motion_is_applied_in_the_correct_direction(self):
        rig = load_nominal_rig(SETTINGS)
        point = np.array(((10., 0., .05),))
        scan, camera = pose(10.), pose(10.02, (.1, 0., 0.))
        left, z, right, zr, valid = project_lidar_into_stereo(point, scan, camera, camera, rig)
        self.assertTrue(valid.all())
        expected, depth, _ = rig.left.project_body(np.array(((9.9, 0., 0.),)))
        np.testing.assert_allclose(left, expected)
        np.testing.assert_allclose(z, depth)
        self.assertFalse(scan.current_control_eligible)

    def test_common_world_translation_does_not_change_relative_projection(self):
        rig = load_nominal_rig(SETTINGS);point = np.array(((10., 1., .05),))
        a, b = pose(10., (1., 2., 3.), .3), pose(10.01, (1.1, 2., 3.), .31)
        first = project_lidar_into_stereo(point, a, b, b, rig)
        a2 = replace(a, position_ned=tuple(np.array(a.position_ned) + (100., 200., -20.)))
        b2 = replace(b, position_ned=tuple(np.array(b.position_ned) + (100., 200., -20.)))
        second = project_lidar_into_stereo(point, a2, b2, b2, rig)
        for x,y in zip(first,second):np.testing.assert_allclose(x,y,atol=1e-10)
        with self.assertRaises(ValueError):project_lidar_into_stereo(point,a,replace(b,epoch=1),b,rig)

    def test_sparse_z_buffer_and_sampling_do_not_fill_missing_pixels(self):
        uv = np.array(((1.2, 1.2), (1.4, 1.4), (3., 3.)))
        depth = np.array((2., 4., 3.));valid = np.ones(3,dtype=bool)
        visible = sparse_visible(uv,depth,valid,5,5)
        np.testing.assert_array_equal(visible,(True,False,True))
        disparity = np.full((5,5),8.,dtype=float);mask = np.ones((5,5),dtype=bool)
        mask[2,2] = False
        values, sampled = sample_disparity(uv,disparity,mask,visible)
        np.testing.assert_array_equal(sampled,(False,False,True))
        self.assertTrue(np.isnan(values[:2]).all());self.assertEqual(values[2],8.)

    def test_nonfinite_zero_and_unconfirmed_max_range_returns_are_not_projected(self):
        rig = load_nominal_rig(SETTINGS)
        values = np.array(((np.nan,0.,0.),(np.inf,0.,0.),(0.,0.,0.),(30.,0.,0.)))
        with np.errstate(all='raise'):
            left,z,right,zr,valid = project_lidar_into_stereo(values,pose(10.),pose(10.),pose(10.),rig)
        self.assertFalse(valid.any())
        self.assertTrue(np.isnan(left).all() and np.isnan(right).all())


if __name__ == '__main__':
    unittest.main()
