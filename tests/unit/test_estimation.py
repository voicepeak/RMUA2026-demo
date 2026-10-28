import math
import unittest
import numpy as np
from estimation.clock import SensorClock
from estimation.geometry import rotation, delta_quaternion
from estimation.imu_gps import ImuGpsEstimator
from interfaces.contracts import Health


class ClockTests(unittest.TestCase):
    def test_rollback_changes_epoch_and_duplicate_does_not_refresh_deadline(self):
        c = SensorClock()
        self.assertFalse(c.observe(10., 1.))
        c.observe(10., 2.)
        self.assertFalse(c.fresh(2.))
        self.assertTrue(c.observe(2., 3.))
        self.assertEqual(c.epoch, 1)
        self.assertTrue(c.fresh(3.01))
        for stamp in (0., math.nan):
            with self.assertRaises(ValueError):
                c.observe(stamp, 4.)


class EstimatorTests(unittest.TestCase):
    def estimator(self):
        e = ImuGpsEstimator()
        e.initialize((1., 2., -1.), (1., 0., 0., 0.))
        e.propagate(1., (0., 0., -9.81), (0., 0., 0.))
        return e

    def test_ned_gravity_stationary_and_positive_yaw(self):
        e = self.estimator()
        for i in range(1, 101):
            e.propagate(1+i*.01, (0., 0., -9.81), (0., 0., 0.))
        np.testing.assert_allclose(e.v, 0., atol=1e-10)
        np.testing.assert_allclose(e.p, (1., 2., -1.), atol=1e-10)
        np.testing.assert_allclose(rotation(delta_quaternion((0., 0., math.pi/2))) @ (1., 0., 0.),
                                   (0., 1., 0.), atol=1e-10)

    def test_gps_zero_outlier_stale_and_reordered_rejected(self):
        e = self.estimator()
        for stamp, position in ((1., (0., 0., 0.)), (1., (100., 0., 0.)),
                                (.1, (1., 2., -1.))):
            self.assertFalse(e.update_gps(stamp, position, (1., 0., 0., 0.)))
        self.assertTrue(e.update_gps(1., (1., 2., -1.), (1., 0., 0., 0.)))
        self.assertFalse(e.update_gps(1., (1., 2., -1.), (1., 0., 0., 0.)))
        self.assertEqual(e.snapshot().health, Health.OK)
        self.assertFalse(e.update_gps(1., (0., 0., 0.), (0., 0., 0., 0.)))
        self.assertFalse(e.update_gps(1.01, (1., 2., -1.), (1., 0., 0., 0.)))

    def test_gps_loss_degrades_without_teleport_and_recovers(self):
        e = self.estimator()
        e.update_gps(1., (1., 2., -1.), (1., 0., 0., 0.))
        for i in range(1, 251):
            e.propagate(1+i*.01, (0., 0., -9.81), (0., 0., 0.))
            e.update_gps(1+i*.01, (0., 0., 0.), (1., 0., 0., 0.))
            if i == 50:
                self.assertEqual(e.snapshot().health, Health.DEGRADED)
        self.assertEqual(e.snapshot().health, Health.LOST)
        np.testing.assert_allclose(e.p, (1., 2., -1.))
        self.assertTrue(e.update_gps(3.5, (1., 2., -1.), (1., 0., 0., 0.)))
        self.assertEqual(e.snapshot().health, Health.OK)

    def test_noisy_stationary_covariance_stays_positive(self):
        e = self.estimator()
        rng = np.random.RandomState(7)
        for i in range(1, 501):
            stamp = 1+i*.01
            e.propagate(stamp, np.array((0., 0., -9.81))+rng.normal(0, .02, 3),
                        rng.normal(0, .001, 3))
            if i % 10 == 0:
                e.update_gps(stamp, np.array((1., 2., -1.))+rng.normal(0, .03, 3),
                             (1., 0., 0., 0.))
        self.assertLess(np.linalg.norm(e.p-(1., 2., -1.)), .1)
        self.assertLess(np.linalg.norm(e.v), .1)
        self.assertGreater(np.linalg.eigvalsh(e.P).min(), 0.)
        np.testing.assert_allclose(e.P, e.P.T, atol=1e-12)

    def test_gap_and_rollback_are_not_silently_integrated(self):
        e = self.estimator()
        e.update_gps(1., (1., 2., -1.), (1., 0., 0., 0.))
        e.propagate(1.2, (5., 0., -9.81), (0., 0., 0.))
        self.assertEqual(e.snapshot().health, Health.DEGRADED)
        np.testing.assert_allclose(e.v, 0.)
        with self.assertRaises(ValueError):
            e.propagate(.5, (0., 0., 0.), (0., 0., 0.))

    def test_rejected_imu_cannot_replace_accepted_angular_velocity(self):
        e = self.estimator()
        e.update_gps(1., (1., 2., -1.), (1., 0., 0., 0.))
        e.propagate(1.01, (0., 0., -9.81), (.1, .2, .3))
        before = e.snapshot()
        for stamp in (1.01, .5):
            with self.assertRaises(ValueError):
                e.propagate(stamp, (0., 0., -9.81), (9., 8., 7.))
            self.assertEqual(e.snapshot(), before)


if __name__ == "__main__":
    unittest.main()
