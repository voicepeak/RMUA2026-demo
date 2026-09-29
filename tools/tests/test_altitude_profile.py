import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "ros_ws" / "src" / "route_follower" / "scripts"))
from altitude_profile import AltitudeProfile, ProfileBlender  # noqa: E402


def profile(start_z=0.0, goal_z=0.0, gates=(), guides=()):
    return AltitudeProfile(0.0, start_z, 100.0, goal_z, list(gates), list(guides),
                           corridor_half=1.5)


class AltitudeProfileTests(unittest.TestCase):
    def test_center_interpolates_gate_anchor(self):
        p = profile(gates=[{"s": 50.0, "z": -10.0, "valid": True}])
        self.assertAlmostEqual(p.center(50.0), -10.0)
        self.assertGreater(p.center(25.0), -10.0)
        self.assertLess(p.center(25.0), 0.0)

    def test_corridor_is_center_plus_minus_half(self):
        p = profile(goal_z=-5.0)
        ceil, floor = p.corridor(50.0)
        self.assertAlmostEqual(ceil, p.center(50.0) - 1.5)
        self.assertAlmostEqual(floor, p.center(50.0) + 1.5)

    def test_dz_ds_sign_follows_climb(self):
        # NED: z 变小 = 变高, 上升段 dz/ds 为负
        p = profile(goal_z=-10.0)
        self.assertLess(p.dz_ds(50.0), 0.0)

    def test_horizon_limits_query(self):
        self.assertAlmostEqual(AltitudeProfile.horizon_s(50.0, 10.0, 5.0), 15.0)
        self.assertAlmostEqual(AltitudeProfile.horizon_s(2.0, 10.0, 5.0), 10.0)

    def test_blender_transitions_smoothly(self):
        old = profile(goal_z=0.0)
        new = profile(goal_z=-10.0)
        blender = ProfileBlender(switch_time=1.0)
        blender.set_initial(old, 0.0)
        blender.switch(new, 10.0)
        self.assertAlmostEqual(blender.center(50.0, 10.0), old.center(50.0))
        self.assertAlmostEqual(blender.center(50.0, 11.0), new.center(50.0))
        mid = blender.center(50.0, 10.5)
        self.assertTrue(blender.is_switching(10.5))
        self.assertLess(mid, old.center(50.0))
        self.assertGreater(mid, new.center(50.0))


if __name__ == "__main__":
    unittest.main()
