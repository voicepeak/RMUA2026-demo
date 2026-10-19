import json
import math
from pathlib import Path
import unittest
import numpy as np
from control.motor_allocator.allocator import MotorAllocator
from control.se3_controller.controller import Se3Controller
from estimation.geometry import delta_quaternion, rotation
from interfaces.contracts import Health, Metadata, StateEstimate


def config():
    return json.loads((Path(__file__).resolve().parents[2]/'configs/vehicle_identification.json').read_text())


def state(q=(1., 0., 0., 0.), omega=(0., 0., 0.)):
    return StateEstimate(Metadata(1., 2., 0, 'test'), (0., 0., -1.), (0., 0., 0.),
                         tuple(q), omega, Health.OK)


class FlightControlTests(unittest.TestCase):
    def test_hover_force_and_motor_order(self):
        c = config()
        allocator = MotorAllocator(c)
        wrench = Se3Controller(c).step(state(), (0., 0., -1.))
        self.assertAlmostEqual(wrench.thrust, .9*9.81)
        allocation = allocator.allocate(wrench.thrust, wrench.moment)
        np.testing.assert_allclose(allocation.pwm, (.17605,)*4, atol=.0001)
        signs = np.sign(allocator.B[1:])
        np.testing.assert_array_equal(signs, ((-1, 1, 1, -1), (1, -1, 1, -1), (1, 1, -1, -1)))

    def test_attitude_feedback_and_rate_damping_restore_level(self):
        for axis in range(3):
            angle = np.zeros(3);angle[axis] = .1
            omega = np.zeros(3);omega[axis] = .1
            w = Se3Controller(config()).step(state(delta_quaternion(angle), tuple(omega)), (0., 0., -1.))
            self.assertLess(w.moment[axis], 0.)

    def test_positive_x_reference_tilts_towards_positive_x_in_ned(self):
        w = Se3Controller(config()).step(state(), (1., 0., -1.))
        # Negative pitch rotates body thrust towards +NED x.
        self.assertLess(w.moment[1], 0.)
        self.assertAlmostEqual(w.moment[0], 0.)

    def test_yaw_rate_feedforward_produces_positive_torque(self):
        w=Se3Controller(config()).step(state(),(0.,0.,-1.),yaw_rate=.1)
        self.assertGreater(w.moment[2],0.)

    def test_allocator_saturates_and_reports_achieved_wrench(self):
        a = MotorAllocator(config())
        for thrust, moment in ((8.829, (2., 2., 1.)), (100., (0., 0., 0.)), (0., (1., 0., 0.))):
            result = a.allocate(thrust, moment)
            self.assertTrue(result.saturated)
            self.assertTrue(all(0 <= v <= .6 for v in result.pwm))
            np.testing.assert_allclose(result.achieved_wrench, a.B @ (np.array(result.pwm)*a.max_thrust))
        result = a.allocate(8.829, (.01, -.01, .005))
        self.assertFalse(result.saturated)
        np.testing.assert_allclose(result.achieved_wrench, (8.829, .01, -.01, .005), atol=1e-10)

    def test_anti_windup_freezes_integral_when_saturated(self):
        c = Se3Controller(config())
        for _ in range(1000):
            w = c.step(state(), (100., 0., -1.))
        self.assertLessEqual(abs(w.integral[0]), 1.)
        for _ in range(10):
            w = c.step(state(), (-100., 0., -1.), allow_integral=False)
        self.assertEqual(w.integral[0], 1.)

    def test_small_angle_closed_loop_roll_converges(self):
        c = config(); controller = Se3Controller(c); allocator = MotorAllocator(c)
        angle, omega = .12, 0.
        for _ in range(1000):
            w = controller.step(state(delta_quaternion((angle, 0., 0.)), (omega, 0., 0.)),
                                (0., 0., -1.), dt=.005)
            applied = allocator.allocate(w.thrust, w.moment)
            omega += applied.achieved_wrench[1]/c['inertia_kg_m2'][0]*.005
            angle += omega*.005
        self.assertLess(abs(angle), 1e-4)
        self.assertLess(abs(omega), 1e-4)


if __name__ == '__main__':
    unittest.main()
