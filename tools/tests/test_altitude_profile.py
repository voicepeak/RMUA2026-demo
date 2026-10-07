import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "ros_ws" / "src" / "route_follower" / "scripts"))
from altitude_profile import AltitudeProfile, ProfileBlender  # noqa: E402


def profile(start_z=0.0, goal_z=0.0, gates=(), guides=()):
    return AltitudeProfile(0.0, start_z, 100.0, goal_z, list(gates), list(guides),
                           corridor_half=1.5)


class AltitudeProfileTests(unittest.TestCase):
    def test_batch_centers_preserve_irregular_anchors_and_clamped_endpoints(self):
        p=profile(start_z=-3.,goal_z=-11.,guides=[dict(s=1.7,z=-4.),
                    dict(s=19.99,z=-5.),dict(s=20.,z=-5.01),dict(s=63.2,z=-2.)])
        stations=np.r_[[-20.,0.,1.7,19.99,20.,63.2,100.,120.],
                       np.random.default_rng(53).uniform(-5.,105.,2000)]
        np.testing.assert_allclose(p.center_many(stations),[p.center(s) for s in stations],
                                   rtol=0.,atol=1e-12)
        self.assertEqual(p.center_many(np.array([])).shape,(0,))

    def test_batch_centers_preserve_interrupted_profile_blends(self):
        b=ProfileBlender(1.);b.set_initial(profile(goal_z=0.))
        b.switch(profile(goal_z=-10.),10.)
        b.switch(profile(goal_z=-20.),10.2)
        b.switch(profile(goal_z=-5.),10.35)
        stations=np.linspace(-5.,105.,200)
        for stamp in (10.35,10.6,11.35,12.):
            np.testing.assert_allclose(b.center_many(stations,stamp),
                                       [b.center(s,stamp) for s in stations],rtol=0.,atol=1e-12)

    def test_center_interpolates_gate_anchor(self):
        p = profile(gates=[{"s": 50.0, "z": -10.0, "valid": True}])
        self.assertAlmostEqual(p.center(50.0), -10.0)
        self.assertGreater(p.center(25.0), -10.0)
        self.assertLess(p.center(25.0), 0.0)

    def test_continuous_climb_does_not_flatten_at_gate(self):
        p = profile(goal_z=-20., gates=[dict(s=50.,z=-10.,valid=True)])
        self.assertAlmostEqual(p.dz_ds(50.),-.2)
        self.assertAlmostEqual(p.dz_ds(49.999),p.dz_ds(50.001),places=5)

    def test_monotone_segments_never_overshoot(self):
        p = profile(goal_z=-11., gates=[dict(s=20.,z=-10.,valid=True)])
        values = [p.center(s) for s in range(101)]
        self.assertTrue(all(b <= a for a,b in zip(values,values[1:])))
        self.assertGreaterEqual(min(values),-11.)

    def test_interrupted_blend_preserves_value_and_slope(self):
        b=ProfileBlender(1.)
        b.set_initial(profile(goal_z=0.))
        b.switch(profile(goal_z=-10.),10.)
        value,slope=b.center(40.,10.2),b.dz_ds(40.,stamp=10.2)
        b.switch(profile(goal_z=-20.),10.2)
        self.assertAlmostEqual(b.center(40.,10.2),value)
        self.assertAlmostEqual(b.dz_ds(40.,stamp=10.2),slope)

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
