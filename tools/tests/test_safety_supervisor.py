import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "ros_ws" / "src" / "route_follower" / "scripts"))
from safety_supervisor import SafetySupervisor  # noqa: E402


class SafetySupervisorTests(unittest.TestCase):
    def setUp(self):
        self.safety = SafetySupervisor(pose_timeout=0.3, z_recovery_error=2.0,
                                       z_recovery_gain=5.0, stuck_speed=0.4,
                                       stuck_time=1.0, max_speed=12.0,
                                       yaw_rate_max=1.0)

    def test_pose_stale(self):
        self.assertTrue(self.safety.pose_stale(10.0, 9.5))
        self.assertFalse(self.safety.pose_stale(10.0, 9.9))
        self.assertTrue(self.safety.pose_stale(10.0, None))

    def test_z_recovery_requires_severity_and_worsening(self):
        v, active = self.safety.z_recovery(10.0, 1.5, True, True)
        self.assertFalse(active)
        self.assertAlmostEqual(v, 10.0)
        v, active = self.safety.z_recovery(10.0, 3.0, False, False)
        self.assertFalse(active)
        v, active = self.safety.z_recovery(10.0, 3.0, True, False)
        self.assertTrue(active)
        self.assertAlmostEqual(v, 5.0 * 2.0 / 3.0)

    def test_stuck_detection_uses_real_dt(self):
        prev = (0.0, 0.0, 0.0)
        pose = (0.0, 0.0, 0.0)
        self.assertFalse(self.safety.stuck_step(0.5, 5.0, prev, pose))
        self.assertFalse(self.safety.stuck_step(0.5, 5.0, prev, pose))
        self.assertTrue(self.safety.stuck_step(0.5, 5.0, prev, pose))

    def test_stuck_resets_when_moving(self):
        prev = (0.0, 0.0, 0.0)
        self.assertFalse(self.safety.stuck_step(0.5, 5.0, prev, (0.0, 0.0, 0.0)))
        self.assertFalse(self.safety.stuck_step(0.5, 5.0, prev, (5.0, 0.0, 0.0)))

    def test_finalize_limits(self):
        vx, vy, vz, yaw = self.safety.finalize(20.0, 0.0, 4.0, 3.0)
        self.assertAlmostEqual((vx ** 2 + vy ** 2) ** 0.5, 12.0)
        self.assertAlmostEqual(yaw, 1.0)


if __name__ == "__main__":
    unittest.main()
