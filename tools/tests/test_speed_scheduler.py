import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "ros_ws" / "src" / "route_follower" / "scripts"))
from speed_scheduler import SpeedScheduler  # noqa: E402


class SpeedSchedulerTests(unittest.TestCase):
    def setUp(self):
        self.sched = SpeedScheduler(cruise_speed=10.0, normal_speed_floor=4.0,
                                    a_lat_max=6.0, slope_eta=0.8, vz_up_safe=4.0,
                                    a_up=4.0, a_down=5.0)

    def target(self, **kwargs):
        base = dict(kap=0.0, kz=0.0, slope_trusted=True, miss_ratio=None,
                    z_err=0.0, z_worsening=False, pred_worse=False,
                    yaw_err_deg=0.0, vxy=10.0)
        base.update(kwargs)
        return self.sched.target(**base)

    def test_cruise_when_unlimited(self):
        v, info = self.target()
        self.assertAlmostEqual(v, 10.0)

    def test_climb_preview_overrides_floor(self):
        v, info = self.target(v_climb_preview=2.5)
        self.assertAlmostEqual(v, 2.5)
        self.assertEqual(info["reason"], "CLIMB")

    def test_curve_preview_caps_speed(self):
        v, info = self.target(v_curve_preview=6.0)
        self.assertAlmostEqual(v, 6.0)
        self.assertEqual(info["reason"], "CURVE")

    def test_map_cap_zero_holds(self):
        v, info = self.target(v_map=0.0)
        self.assertEqual(v, 0.0)
        self.assertEqual(info["reason"], "MAP")

    def test_tracking_soft_cap_cannot_beat_hard_cap(self):
        v, info = self.target(miss_ratio=2.0, v_climb_preview=3.0)
        self.assertAlmostEqual(v, 3.0)

    def test_tracking_soft_cap_uses_floor(self):
        v, info = self.target(miss_ratio=2.0)
        self.assertAlmostEqual(v, 4.5)
        self.assertEqual(info["reason"], "TRACKING")
        sched = SpeedScheduler(cruise_speed=10.0, normal_speed_floor=5.0)
        v, _ = sched.target(kap=0.0, kz=0.0, slope_trusted=True, miss_ratio=2.0,
                            z_err=0.0, z_worsening=False, pred_worse=False,
                            yaw_err_deg=0.0, vxy=10.0)
        self.assertAlmostEqual(v, 5.0)

    def test_step_clamps_to_hard_cap_immediately(self):
        v = self.sched.step(10.0, 10.0, 0.05, hard_cap=3.0)
        self.assertAlmostEqual(v, 3.0)

    def test_step_ramps_toward_target(self):
        v = self.sched.step(0.0, 10.0, 0.1)
        self.assertAlmostEqual(v, 0.4)


if __name__ == "__main__":
    unittest.main()
