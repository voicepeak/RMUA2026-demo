import unittest
import numpy as np
from estimation.geometry import delta_quaternion, rotation
from perception.visual_odometry.gyro_consistency import gyro_rotation, rotation_residual


class GyroConsistencyTests(unittest.TestCase):
    def test_constant_rate_interpolates_boundaries(self):
        samples=[(i*.01,(0.,0.,1.)) for i in range(21)]
        actual=gyro_rotation(samples,.005,.195)
        np.testing.assert_allclose(actual,rotation(delta_quaternion((0.,0.,.19))),atol=1e-12)
        self.assertLess(rotation_residual(actual,actual),1e-7)

    def test_no_extrapolation_or_gap_bridging(self):
        samples=[(1.,(0.,0.,0.)),(1.01,(0.,0.,0.)),(1.1,(0.,0.,0.))]
        for a,b in ((.99,1.01),(1.,1.02),(1.01,1.11)):
            with self.assertRaises(ValueError):gyro_rotation(samples,a,b)

    def test_rejects_reflections_and_duplicate_times(self):
        with self.assertRaises(ValueError):rotation_residual(np.diag((1.,1.,-1.)),np.eye(3))
        with self.assertRaises(ValueError):gyro_rotation([(1.,(0.,0.,0.)),(1.,(0.,0.,0.))],1.,1.01)


if __name__=='__main__':unittest.main()
