import unittest
import numpy as np
from perception.visual_odometry.inertial_fusion import InertialVisualTrajectory


class InertialVisualTests(unittest.TestCase):
    def test_stationary_specific_force_cancels_gravity(self):
        samples=[(i*.01,(0.,0.,-9.81),(1.,0.,0.,0.)) for i in range(11)]
        fusion=InertialVisualTrajectory(0.,0,(1.,0.,0.,0.))
        result=fusion.append(.1,(0.,0.,0.),samples)
        np.testing.assert_allclose(result['position_initial_body_m'],(0.,0.,0.),atol=1e-12)
        np.testing.assert_allclose(result['velocity_initial_body_m_s'],(0.,0.,0.),atol=1e-12)
        self.assertFalse(result['control_eligible'])

    def test_missing_imu_latches_failure(self):
        fusion=InertialVisualTrajectory(1.,0,(1.,0.,0.,0.))
        samples=[(1.,(0.,0.,-9.81),(1.,0.,0.,0.)),(1.1,(0.,0.,-9.81),(1.,0.,0.,0.))]
        with self.assertRaises(ValueError):fusion.append(1.1,(0.,0.,0.),samples)
        with self.assertRaises(ValueError):fusion.append(1.1,(0.,0.,0.),samples)


if __name__=='__main__':unittest.main()
