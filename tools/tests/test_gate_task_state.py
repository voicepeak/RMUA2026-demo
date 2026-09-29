import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "ros_ws" / "src" / "route_follower" / "scripts"))
from gate_task_state import GateTaskState, PASS, MISS, SKIP  # noqa: E402


def gate(x=0.0, y=0.0, z=0.0, s=10.0, gid=0, normal=(1.0, 0.0, 0.0)):
    return {"id": gid, "x": x, "y": y, "z": z, "s": s,
            "nx": normal[0], "ny": normal[1], "nz": normal[2]}


class GateTaskStateTests(unittest.TestCase):
    def setUp(self):
        self.state = GateTaskState(half_width=1.5, half_height=1.5,
                                   miss_margin=3.0, skip_s=5.0, miss_radius=6.0)

    def test_plane_crossing_inside_aperture_passes(self):
        gates = [gate(s=10.0)]
        idx, events = self.state.step(gates, 0, (-2.0, 0.0, 0.0), (2.0, 0.0, 0.0), 10.0)
        self.assertEqual(idx, 1)
        self.assertEqual(events[0]["status"], PASS)
        self.assertEqual(events[0]["basis"], "gate_plane_crossing")
        self.assertIn(0, self.state.passed_gate_ids)
        self.assertNotIn(0, self.state.missed_gate_ids)

    def test_plane_crossing_outside_aperture_misses(self):
        gates = [gate(s=10.0)]
        idx, events = self.state.step(gates, 0, (-2.0, 0.0, 4.0), (2.0, 0.0, 4.0), 10.0)
        self.assertEqual(idx, 1)
        self.assertEqual(events[0]["status"], MISS)
        self.assertIn(0, self.state.missed_gate_ids)
        self.assertNotIn(0, self.state.passed_gate_ids)

    def test_far_plane_crossing_is_ignored(self):
        gates = [gate(s=10.0)]
        idx, events = self.state.step(gates, 0, (-2.0, 30.0, 0.0), (2.0, 30.0, 0.0), 10.0)
        self.assertEqual(idx, 0)
        self.assertEqual(events, [])

    def test_no_crossing_but_behind_progress_marks_miss_then_skip(self):
        gates = [gate(s=10.0)]
        idx, events = self.state.step(gates, 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 14.0)
        self.assertEqual(events[0]["status"], MISS)
        self.state.reset()
        idx, events = self.state.step(gates, 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 16.0)
        self.assertEqual(events[0]["status"], SKIP)

    def test_ledgers_are_separated(self):
        gates = [gate(gid=0, s=10.0)]
        self.state.step(gates, 0, (-2.0, 0.0, 4.0), (2.0, 0.0, 4.0), 10.0)
        summary = self.state.summary()
        self.assertEqual(summary, {"resolved": 1, "passed": 0, "missed": 1, "skipped": 0})


if __name__ == "__main__":
    unittest.main()
