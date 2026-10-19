import math
import unittest
import numpy as np
from control.reference import benchmark_reference
from control.vel_benchmark.controller import body_velocity, VelocityCommand
from estimation.geometry import delta_quaternion
from interfaces.contracts import Health, Metadata, StateEstimate


class BenchmarkTests(unittest.TestCase):
    def test_reference_has_zero_endpoint_velocity_and_acceleration(self):
        for time,z in ((0.,0.),(3.,-1.),(10.,-1.)):
            p,v,a=benchmark_reference((0.,0.,0.),time)
            np.testing.assert_allclose(p,(0.,0.,z))
            np.testing.assert_allclose(v,0.,atol=1e-10)
            np.testing.assert_allclose(a,0.,atol=1e-10)
        t,h=1.4,1e-4
        p0,v0,_=benchmark_reference((0.,0.,0.),t)
        p1,v1,_=benchmark_reference((0.,0.,0.),t+h)
        np.testing.assert_allclose((np.array(p1)-p0)/h,v0,atol=1e-4)

    def test_yaw_ninety_world_x_becomes_body_left_and_ned_up_positive(self):
        q=tuple(delta_quaternion((0.,0.,math.pi/2)))
        state=StateEstimate(Metadata(1.,2.,0,'test'),(0.,0.,0.),(0.,0.,0.),q,(0.,0.,0.),Health.OK)
        v=body_velocity(state,(.1,0.,-.1),yaw_reference=math.pi/2)
        self.assertAlmostEqual(v[0],0.)
        self.assertLess(v[1],0.)
        self.assertGreater(v[2],0.)
        self.assertAlmostEqual(v[3],0.)
        VelocityCommand(1.,0,0,v)
        with self.assertRaises(ValueError):
            VelocityCommand(1.,0,0,(10.,0.,0.,0.))


if __name__ == '__main__':
    unittest.main()
