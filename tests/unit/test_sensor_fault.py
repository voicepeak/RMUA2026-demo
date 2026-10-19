import unittest
from adapters.ros_noetic.sensor_fault import LocalSensorFault


class FaultScheduleTests(unittest.TestCase):
    def test_window_is_simulator_time_and_does_not_affect_other_sensor(self):
        f=LocalSensorFault('GPS_ZERO',10.,.2)
        self.assertFalse(f.affects('GPS_ZERO',9.99))
        self.assertFalse(f.affects('IMU_DROP',10.1))
        self.assertTrue(f.affects('GPS_ZERO',10.))
        self.assertTrue(f.affects('GPS_ZERO',10.19))
        self.assertFalse(f.affects('GPS_ZERO',10.2))
        self.assertEqual(f.injected_samples,2)


if __name__ == '__main__':
    unittest.main()
