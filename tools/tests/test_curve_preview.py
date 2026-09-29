import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "ros_ws" / "src" / "route_follower" / "scripts"))
from curve_preview import CurveBrakingEnvelope  # noqa: E402


class CurvePreviewTests(unittest.TestCase):
    def setUp(self):
        self.env = CurveBrakingEnvelope(preview_time=3.0, preview_min=20.0,
                                        preview_max=40.0, preview_step=3.0,
                                        a_brake=4.0)

    def test_straight_route_does_not_limit(self):
        result = self.env.evaluate(0.0, lambda s: 0.0, 6.0, 10.0, cruise=10.0)
        self.assertGreaterEqual(result["v_curve_preview"], 10.0)

    def test_future_curve_reduces_current_cap(self):
        result = self.env.evaluate(0.0, lambda s: 0.1, 6.0, 10.0, cruise=10.0)
        self.assertLess(result["v_curve_preview"], 10.0)
        self.assertGreater(result["v_curve_preview"], 5.0)

    def test_braking_envelope_is_never_below_future_limit(self):
        result = self.env.evaluate(0.0, lambda s: 0.1, 6.0, 10.0, cruise=10.0)
        self.assertGreaterEqual(result["v_curve_preview"], result["curve_worst_v"])


if __name__ == "__main__":
    unittest.main()
