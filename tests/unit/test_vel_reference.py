import math
import unittest
import numpy as np
from control.reference import benchmark_reference,benchmark_yaw
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

    def test_round_trip_brakes_at_both_endpoints(self):
        for t, x in ((4., 0.), (8., .3), (9., .3), (13., 0.), (16., 0.)):
            p, v, a = benchmark_reference((0., 0., 0.), t, round_trip=True)
            self.assertAlmostEqual(p[0], x)
            self.assertAlmostEqual(v[0], 0.)
            self.assertAlmostEqual(a[0], 0.)
        for t in (5., 7., 10., 12.):
            p, v, a = benchmark_reference((0., 0., 0.), t, round_trip=True)
            p1, v1, _ = benchmark_reference((0., 0., 0.), t+1e-4, round_trip=True)
            self.assertAlmostEqual((p1[0]-p[0])/1e-4, v[0], places=4)
            self.assertAlmostEqual((v1[0]-v[0])/1e-4, a[0], places=4)
            self.assertLess(abs(v[0]), .15)
        with self.assertRaises(ValueError):
            benchmark_reference((0., 0., 0.), 5., braking=True, round_trip=True)

    def test_departure_probe_has_bounded_smooth_motion(self):
        for t,x in ((4.,0.),(7.,.4),(10.,.8),(15.,.8)):
            p,v,a=benchmark_reference((0.,0.,0.),t,departure=True)
            self.assertAlmostEqual(p[0],x)
            self.assertLessEqual(abs(v[0]),.25)
            self.assertLessEqual(np.linalg.norm(p),1.8)
        for t in (5.,7.,9.):
            p,v,a=benchmark_reference((0.,0.,0.),t,departure=True)
            p1,v1,_=benchmark_reference((0.,0.,0.),t+1e-4,departure=True)
            self.assertAlmostEqual((p1[0]-p[0])/1e-4,v[0],places=4)
            self.assertAlmostEqual((v1[0]-v[0])/1e-4,a[0],places=4)
        with self.assertRaises(ValueError):
            benchmark_reference((0.,0.,0.),5.,round_trip=True,departure=True)

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

    def test_yaw_reference_radians_and_vel_rate_degrees(self):
        self.assertEqual(benchmark_yaw(.1,4.,True),(.1,0.))
        angle,rate=benchmark_yaw(.1,6.,True)
        self.assertAlmostEqual(angle,.2)
        self.assertAlmostEqual(rate,.1)
        self.assertAlmostEqual(benchmark_yaw(.1,9.,True)[0],.3)
        s=StateEstimate(Metadata(1.,2.,0,'test'),(0.,0.,0.),(0.,0.,0.),(1.,0.,0.,0.),(0.,0.,0.),Health.OK)
        self.assertAlmostEqual(body_velocity(s,(0.,0.,0.),yaw_rate=.1)[3],math.degrees(.1))


if __name__ == '__main__':
    unittest.main()
