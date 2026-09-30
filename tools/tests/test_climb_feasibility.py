import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "ros_ws" / "src" / "route_follower" / "scripts"))
from climb_feasibility import ClimbFeasibility  # noqa: E402


class Capability(object):
    def __init__(self, up=3.0, down=3.0):
        self.up_value = up
        self.down_value = down

    def up(self, vxy):
        return self.up_value

    def down(self, vxy):
        return self.down_value


class ClimbFeasibilityTests(unittest.TestCase):
    def setUp(self):
        self.feas = ClimbFeasibility(preview_time=3.5, preview_min=20.0,
                                     preview_max=50.0, preview_step=5.0,
                                     response_time=0.4, eta=1.0)
        self.cap = Capability()

    def test_flat_profile_does_not_limit(self):
        result = self.feas.evaluate(0.0, -5.0, lambda s: -5.0, self.cap, 10.0)
        self.assertAlmostEqual(result["v_climb_preview"], 10.0)

    def test_flat_profile_allows_fifteen(self):
        result=self.feas.evaluate(0.,-5.,lambda s:-5.,self.cap,15.)
        self.assertAlmostEqual(result['v_climb_preview'],15.)

    def test_established_climb_does_not_pay_startup_latency_again(self):
        result=self.feas.evaluate(0.,0.,lambda s:-.5*s,self.cap,15.,velocity_up=3.)
        self.assertAlmostEqual(result['v_climb_preview'],6.)

    def test_wrong_way_vertical_motion_keeps_response_allowance(self):
        stationary=self.feas.evaluate(0.,0.,lambda s:-.5*s,self.cap,15.)
        descending=self.feas.evaluate(0.,0.,lambda s:-.5*s,self.cap,15.,velocity_up=-3.)
        self.assertLessEqual(descending['v_climb_preview'],stationary['v_climb_preview'])

    def test_rising_profile_limits_speed(self):
        # NED: z_ref 变小 = 变高; 坡度 0.5 上升, vz 能力 3 m/s
        result = self.feas.evaluate(0.0, 0.0, lambda s: -0.5 * s, self.cap, 10.0)
        self.assertLess(result["v_climb_preview"], 5.0)
        self.assertGreater(result["v_climb_preview"], 3.0)
        self.assertGreater(result["climb_worst_dz"], 0.0)

    def test_horizon_clamps_between_min_and_max(self):
        self.assertAlmostEqual(self.feas.horizon(1.0), 20.0)
        self.assertAlmostEqual(self.feas.horizon(10.0), 35.0)
        self.assertAlmostEqual(self.feas.horizon(30.0), 50.0)

    def test_descent_uses_down_capability(self):
        cap = Capability(up=3.0, down=1.0)
        result = self.feas.evaluate(0.0, 0.0, lambda s: +0.5 * s, cap, 10.0)
        # 下降能力 1 m/s 更弱, 应比上升场景更严格
        self.assertLess(result["v_climb_preview"], 3.0)
        self.assertAlmostEqual(result["climb_vz_down_available"], 1.0)


if __name__ == "__main__":
    unittest.main()
