import subprocess
import unittest
from unittest.mock import Mock, patch

from evaluation.seed_runner.pwm_identification import cleanup_simulator


class SimulatorCleanupTests(unittest.TestCase):
    @patch('evaluation.seed_runner.pwm_identification.subprocess.run')
    def test_success_requires_stop_script_and_wrapper_exit(self, run):
        run.return_value.returncode = 0
        process = Mock(); process.poll.return_value = None; process.wait.return_value = 143
        result = cleanup_simulator(process, '/stop')
        self.assertTrue(result['stop_and_launcher_completed'])
        self.assertEqual(result['stop_script_exit_code'], 0)
        process.terminate.assert_not_called()

    @patch('evaluation.seed_runner.pwm_identification.subprocess.run')
    def test_stop_failure_and_timeout_are_not_reported_as_cleanup(self, run):
        for outcome in (Mock(returncode=1), subprocess.TimeoutExpired('/stop', 15)):
            with self.subTest(outcome=outcome):
                run.reset_mock(side_effect=True, return_value=True)
                if isinstance(outcome, Exception): run.side_effect = outcome
                else: run.return_value = outcome
                process = Mock(); process.poll.return_value = None; process.wait.return_value = -15
                result = cleanup_simulator(process, '/stop')
                self.assertFalse(result['stop_and_launcher_completed'])
                process.terminate.assert_called_once()

    @patch('evaluation.seed_runner.pwm_identification.subprocess.run')
    def test_unexpected_launcher_exit_is_not_successful_cleanup(self, run):
        process = Mock(); process.poll.return_value = 1; process.returncode = 1
        result = cleanup_simulator(process, '/stop')
        self.assertFalse(result['stop_and_launcher_completed'])
        self.assertEqual(result['error'], 'SIMULATOR_EXITED_BEFORE_CLEANUP')
        run.assert_not_called()
