import sys
import unittest
from unittest.mock import Mock
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "ros_ws" / "src" / "route_follower" / "scripts"))
from gate_chain import GateChain, VERIFIED, HARD  # noqa: E402
from reference_planner import RouteGeometry, ReferencePlanner  # noqa: E402


ROUTE = [(0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (100.0, 100.0, -10.0)]


class RouteGeometryTests(unittest.TestCase):
    def setUp(self):
        self.route = RouteGeometry(ROUTE)

    def test_total_length(self):
        self.assertAlmostEqual(self.route.total_s, 200.0)

    def test_point_at_midpoint(self):
        x, y, _ = self.route.point_at(50.0)
        self.assertAlmostEqual(x, 50.0)
        self.assertAlmostEqual(y, 0.0)

    def test_project_returns_progress(self):
        i, t, d, c = self.route.project((50.0, 2.0, 0.0))
        self.assertEqual(i, 0)
        self.assertAlmostEqual(d, 2.0)
        self.assertAlmostEqual(self.route.seg_s[i] + t * self.route.seg_len[i], 50.0)

    def test_project_gate(self):
        self.assertAlmostEqual(self.route.project_gate(100.0, 50.0), 150.0)

    def test_curvature_zero_on_straight(self):
        straight = RouteGeometry([(0.0, 0.0, 0.0), (50.0, 0.0, 0.0),
                                  (100.0, 0.0, 0.0)])
        self.assertAlmostEqual(straight.curvature(50.0), 0.0)

    def test_tangent_direction(self):
        tx, ty, _ = self.route.tangent(50.0)
        self.assertAlmostEqual(tx, 1.0)
        self.assertAlmostEqual(ty, 0.0)

    def test_tangent_fills_missing_gate_normal(self):
        planner = ReferencePlanner(self.route, snap_gate_to_route=False)
        gates = planner.adjust_gates([{"id": 0, "x": 50.0, "y": 0.0, "z": -3.0}],
                                     verified_ids={0})
        self.assertAlmostEqual(gates[0]["nx"], 1.0)
        self.assertAlmostEqual(gates[0]["nz"], 0.0)
        self.assertEqual(gates[0]["anchor_class"], VERIFIED)


class GateChainAnchorTests(unittest.TestCase):
    def test_verified_kept_when_slope_suspect(self):
        gates = [{"id": 0, "s": 10.0, "z": 0.0, "anchor_class": VERIFIED},
                 {"id": 1, "s": 20.0, "z": -10.0, "anchor_class": VERIFIED}]
        chain = GateChain(gates, slope_abs_max=0.05)
        self.assertTrue(chain.gates[1]["z_suspect"])
        self.assertIn(1, chain.suspect_ids())
        self.assertEqual(len(chain.anchors()), 2)

    def test_online_hard_is_demoted_when_suspect(self):
        gates = [{"id": 0, "s": 10.0, "z": 0.0, "anchor_class": HARD},
                 {"id": 1, "s": 20.0, "z": 0.0, "anchor_class": HARD},
                 {"id": 2, "s": 30.0, "z": -10.0, "anchor_class": HARD}]
        chain = GateChain(gates, slope_abs_max=0.05)
        self.assertTrue(chain.gates[0]["trusted"])
        self.assertFalse(chain.gates[1]["trusted"])
        self.assertEqual(chain.anchors(), [(10.0, 0.0)])
        self.assertEqual([g["id"] for g in chain.trend_gates()], [1, 2])


class ReferencePlannerTests(unittest.TestCase):
    def test_recorded_guides_override_fitted_extrapolation(self):
        route = RouteGeometry([(0., 0., 0.), (200., 0., 0.)])
        planner = ReferencePlanner(route)
        planner.height_prior = Mock(valid=True)
        planner.height_prior.center.return_value = -30.
        planner.height_prior.horizon.return_value = 120.
        guides = [{"s": s, "z": -5.} for s in range(60, 201, 10)]
        _, profile = planner.build(
            [{"id": 1, "x": 40., "y": 0., "z": -5., "s": 40.,
              "source": "static_yaml"}], guides, [], 0., 0., verified_ids={1})
        self.assertAlmostEqual(profile.center(100.), -5.)
        self.assertAlmostEqual(profile.center(200.), -5.)
        self.assertEqual(planner.evidence_horizon, 200.)
        self.assertEqual(planner.trend_horizon, 200.)

    def test_build_keeps_verified_gate_anchor(self):
        route = RouteGeometry(ROUTE)
        planner = ReferencePlanner(route, snap_gate_to_route=True,
                                   slope_abs_max=0.05)
        gates = [{"id": 1, "x": 100.0, "y": 0.0, "z": -8.0, "source": "static_yaml",
                  "s": 100.0}]
        chain, profile = planner.build(gates, [], [], s_now=0.0, p0z=0.0,
                                       start_anchor_z=0.0, verified_ids={1})
        self.assertEqual(len(chain.anchors()), 1)
        self.assertTrue(chain.gates[0]["trusted"])
        self.assertAlmostEqual(profile.center(100.0), -8.0)

    def test_index_helpers(self):
        route = RouteGeometry(ROUTE)
        planner = ReferencePlanner(route, start_gate=0)
        gates = [{"id": i, "x": float(10 * (i + 1)), "y": 0.0, "z": 0.0}
                 for i in range(3)]
        chain, _ = planner.build(gates, [], [], s_now=0.0, p0z=0.0,
                                 start_anchor_z=0.0)
        self.assertEqual(planner.initial_index(chain, 25.0), 2)
        self.assertEqual(planner.resolve_index(chain, {0, 1}), 2)


if __name__ == "__main__":
    unittest.main()
