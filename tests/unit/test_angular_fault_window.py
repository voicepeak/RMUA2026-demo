import unittest
from evaluation.metrics.angular_fault_window import evaluate


class AngularFaultWindowTests(unittest.TestCase):
    def test_only_samples_and_commands_in_fault_window_are_counted(self):
        imu = [{'arrival_wall_monotonic': 9.7, 'stamp': 1., 'omega_body': [0, 10, 0]},
               {'arrival_wall_monotonic': 9.98, 'stamp': 2., 'omega_body': [0, .5, 0]},
               {'arrival_wall_monotonic': 9.99, 'stamp': 3., 'omega_body': [0, 3, 0]}]
        commands = [{'publish_end_wall_monotonic': 9.97}, {'publish_end_wall_monotonic': 9.985},
                    {'publish_end_wall_monotonic': 10.01}]
        result = evaluate(imu, commands, 10.)
        self.assertEqual(result['recorded_imu_samples_in_window'], 2)
        self.assertEqual(result['crossings']['0.35']['source_stamp'], 2.)
        self.assertEqual(result['crossings']['0.35']['published_commands_after_arrival_before_fault'], 1)
        self.assertEqual(result['crossings']['2.5']['source_stamp'], 3.)
        self.assertFalse(result['recovery_tested'])

    def test_absent_crossing_and_invalid_time(self):
        self.assertIsNone(evaluate([], [], 1.)['crossings']['0.35'])
        with self.assertRaises(ValueError):
            evaluate([], [], float('nan'))
