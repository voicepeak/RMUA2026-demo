import sys
import unittest
from pathlib import Path

import yaml

SCRIPTS = (Path(__file__).resolve().parents[2]
           / "ros_ws" / "src" / "route_follower" / "scripts")
CONFIG = SCRIPTS.parent / "config"
sys.path.insert(0, str(SCRIPTS))
from gate_task_state import GateTaskState, PASS  # noqa: E402
from reference_planner import RouteGeometry, ReferencePlanner  # noqa: E402


class RoutePipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        route_cfg = yaml.safe_load((CONFIG / "route_1_3.yaml").read_text())
        cls.route_pts = route_cfg["routes"]["route_1_3"]
        cls.gates = yaml.safe_load((CONFIG / "gates_vision_1_3.yaml").read_text())["gates"]

    def test_route_center_does_not_fabricate_passes_for_offset_measurements(self):
        route = RouteGeometry(self.route_pts)
        planner = ReferencePlanner(route, gate_z_uses_offset=False,
                                   snap_gate_to_route=True)
        chain, profile = planner.build(self.gates, [], [], s_now=0.0, p0z=0.0,
                                       start_anchor_z=self.route_pts[0][2])
        self.assertEqual(len(chain.gates), len(self.gates))
        state = GateTaskState(half_width=1.5, half_height=1.5,
                              miss_margin=3.0, skip_s=5.0, miss_radius=6.0)
        idx = 0
        prev = None
        s = 0.0
        while s <= route.total_s - 1.0:
            x, y, _ = route.point_at(s)
            z = profile.center(s)
            pose = (x, y, z)
            if prev is not None:
                idx, events = state.step(chain.gates, idx, prev, pose, s)
                for event in events:
                    self.assertIn(event["status"], ("PASS", "MISS", "SKIP"))
            prev = (x, y, z)
            s += 1.0
        self.assertEqual(idx, len(chain.gates))
        self.assertLess(state.summary()["passed"], len(self.gates))
        self.assertGreater(state.summary()["missed"]+state.summary()["skipped"], 0)

    def test_crossing_measured_centers_passes_even_with_guidance_snapping(self):
        planner=ReferencePlanner(RouteGeometry(self.route_pts),snap_gate_to_route=True)
        chain,_=planner.build(self.gates,[],[],0.,0.)
        state=GateTaskState()
        idx=0
        for g in chain.gates:
            center=g['measurement_center']
            n=(g['nx'],g['ny'],g['nz'])
            before=tuple(c-v for c,v in zip(center,n))
            after=tuple(c+v for c,v in zip(center,n))
            idx,events=state.step(chain.gates,idx,before,after,g['s']+.1)
            self.assertTrue(events)
            self.assertEqual(events[0]['status'],PASS)
        self.assertEqual(state.summary()['passed'],len(self.gates))

    def test_verified_gate_with_extreme_height_stays_anchor(self):
        route = RouteGeometry(self.route_pts)
        planner = ReferencePlanner(route, gate_z_uses_offset=False,
                                   slope_abs_max=0.01, snap_gate_to_route=True)
        verified = set(g.get("id") for g in self.gates)
        chain, profile = planner.build(self.gates, [], [], s_now=0.0, p0z=0.0,
                                       start_anchor_z=self.route_pts[0][2],
                                       verified_ids=verified)
        self.assertTrue(all(g["trusted"] for g in chain.gates))
        self.assertTrue(any(g["z_suspect"] for g in chain.gates))
        self.assertEqual(len(chain.anchors()), len(self.gates))


if __name__ == "__main__":
    unittest.main()
