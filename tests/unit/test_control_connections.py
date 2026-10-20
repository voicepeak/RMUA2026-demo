import unittest
from adapters.ros_noetic.control_connections import expected_control_connections


class BackendConnectionTests(unittest.TestCase):
    def test_an_evaluator_reader_cannot_replace_the_simulator_connection(self):
        imu='/airsim_node/drone_1/imu/imu';pwm='/airsim_node/drone_1/rotor_pwm_cmd'
        publishers=[(imu,['/simulator'])]
        with self.assertRaises(RuntimeError):
            expected_control_connections((publishers,[(pwm,['/evaluator'])],[]),pwm)
        self.assertEqual(expected_control_connections((publishers,[(pwm,['/simulator','/evaluator'])],[]),pwm),
                         (2,['/simulator']))


if __name__=='__main__':unittest.main()
