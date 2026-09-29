import copy
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluate_run import evaluate


def gate(uid="g0", x=0):
    return dict(uid=uid, verified=True, verification="synthetic fixture", center=[x, 0, 0],
                normal=[1, 0, 0], axis_u=[0, 1, 0], axis_v=[0, 0, 1], half_width=1, half_height=1)


def course(*gates, complete=True):
    return dict(frame="NED", complete=complete,
                expected_gate_count=len(gates) if complete else None, gates=list(gates))


def poses(points, stamps=None):
    return [dict(stamp=t, x=p[0], y=p[1], z=p[2])
            for t, p in zip(stamps if stamps is not None else np.arange(len(points)) * .1, points)]


class EvaluationTests(unittest.TestCase):
    def test_plane_interpolation_and_body_reserve(self):
        result = evaluate(course(gate()), poses([[-1, .2, .3], [1, .4, .5]]))
        self.assertEqual(result["geometric_verdict"], "PASS")
        self.assertAlmostEqual(result["events"][0]["clearance_m"], .35)
        self.assertEqual(result["official_result"], "UNKNOWN")
        self.assertEqual(result["race_success"], "UNKNOWN")

    def test_offset_gate_is_not_snapped_to_route(self):
        g = gate()
        g["center"][1] = 3
        result = evaluate(course(g), poses([[-1, 0, 0], [1, 0, 0]]))
        self.assertEqual(result["geometric_verdict"], "FAIL")

    def test_tilted_gate_uses_local_axes(self):
        g = gate()
        rotation = np.array([[1, 0, 0], [0, 2**-.5, -2**-.5], [0, 2**-.5, 2**-.5]])
        for k in ("normal", "axis_u", "axis_v"):
            g[k] = (rotation @ g[k]).tolist()
        result = evaluate(course(g), poses([rotation @ [-1, .7, .7], rotation @ [1, .7, .7]]))
        self.assertEqual(result["geometric_verdict"], "PASS")

    def test_boundary_and_reverse_do_not_pass(self):
        self.assertEqual(evaluate(course(gate()), poses([[-1, .75, 0], [1, .75, 0]]))["geometric_verdict"], "FAIL")
        self.assertEqual(evaluate(course(gate()), poses([[1, 0, 0], [-1, 0, 0]]))["geometric_verdict"], "UNKNOWN")

    def test_gaps_resets_and_teleports_cannot_prove_pass(self):
        for points, stamps in [([[-1, 0, 0], [1, 0, 0]], [0, 1]),
                               ([[-1, 0, 0], [1, 0, 0]], [1, 0]),
                               ([[-10, 0, 0], [10, 0, 0]], [0, .1])]:
            with self.subTest(stamps=stamps):
                result = evaluate(course(gate()), poses(points, stamps))
                self.assertEqual(result["geometric_verdict"], "UNKNOWN")
                self.assertFalse(result["events"])

    def test_multiple_gates_in_one_segment_ordered_by_intersection(self):
        result = evaluate(course(gate("a", -.5), gate("b", .5)), poses([[-1, 0, 0], [1, 0, 0]]))
        self.assertEqual([e["uid"] for e in result["events"]], ["a", "b"])
        self.assertEqual(result["geometric_verdict"], "PASS")
        reversed_course = course(gate("b", .5), gate("a", -.5))
        self.assertEqual(evaluate(reversed_course, poses([[-1, 0, 0], [1, 0, 0]]))["geometric_verdict"], "FAIL")

    def test_repeat_crossing_is_not_double_counted(self):
        result = evaluate(course(gate()), poses([[-1, 0, 0], [1, 0, 0], [-1, 0, 0], [1, 0, 0]]))
        self.assertEqual(len(result["events"]), 1)

    def test_missing_earlier_crossing_does_not_invent_miss(self):
        result = evaluate(course(gate("a", -5), gate("b", 0)), poses([[-1, 0, 0], [1, 0, 0]]))
        self.assertEqual(result["geometric_verdict"], "UNKNOWN")
        self.assertEqual(result["gates"][0]["status"], "UNKNOWN")

    def test_unverified_incomplete_unseen_and_empty_remain_unknown(self):
        g = gate()
        g["verified"] = False
        for spec in (course(g), course(gate(), complete=False), course(), course(gate("far", 5))):
            self.assertEqual(evaluate(spec, poses([[-1, 0, 0], [1, 0, 0]]))["geometric_verdict"], "UNKNOWN")

    def test_invalid_geometry_rejected(self):
        for key, value in [("normal", [2, 0, 0]), ("center", [float("nan"), 0, 0]),
                           ("half_width", -.1), ("axis_u", [1, 0, 0])]:
            g = gate()
            g[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                evaluate(course(g), [])
        with self.assertRaises(ValueError):
            evaluate(course(gate(), copy.deepcopy(gate())), [])


if __name__ == "__main__":
    unittest.main()
