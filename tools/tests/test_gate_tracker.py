import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "ros_ws" / "src" / "rmua_gate_vision" / "scripts"))
from gate_tracker import GateTracker  # noqa: E402


class GateTrackerTests(unittest.TestCase):
    def test_assoc_radius_prevents_nested_gate_merge(self):
        tracker = GateTracker(assoc_radius=4.0)
        t1 = tracker.update(np.array([0.0, 0.0, -3.0]), 10.0, 1.0)
        t2 = tracker.update(np.array([10.0, 0.0, -3.0]), 12.0, 1.1)
        self.assertNotEqual(t1.id, t2.id)
        self.assertEqual(len(tracker.tracks), 2)

    def test_regressing_stamp_does_not_update(self):
        tracker = GateTracker()
        t = tracker.update(np.array([0.0, 0.0, -3.0]), 10.0, 5.0)
        support = t.support
        t.update(np.array([9.0, 9.0, -3.0]), 10.0, 4.0, beta=1.0)
        self.assertEqual(t.support, support)
        self.assertTrue(np.allclose(t.mean, [0.0, 0.0, -3.0]))

    def test_sigma_forgets_far_outlier_after_recent_window(self):
        tracker = GateTracker()
        t = tracker.update(np.array([0.0, 0.0, -3.0]), 10.0, 0.0)
        t.update(np.array([50.0, 0.0, -3.0]), 10.0, 1.0, beta=1.0)
        for k in range(12):
            t.update(np.array([0.0, 0.0, -3.0]), 10.0, 2.0 + 0.1 * k, beta=1.0)
        self.assertLess(float(np.max(t.sigma())), 0.01)

    def test_prune_never_removes_hard_anchor(self):
        tracker = GateTracker(max_age=1.0)
        t = tracker.update(np.array([0.0, 0.0, -3.0]), 10.0, 0.0)
        t.hard_anchor = True
        self.assertEqual(tracker.prune(100.0), [])
        self.assertIn(t.id, tracker.tracks)

    def test_refresh_hard_needs_support_and_confidence_without_geometry(self):
        tracker = GateTracker(min_support_hard=3, sigma_hard=0.6)
        quiet = tracker.update(np.array([0.0, 0.0, -3.0]), 10.0, 0.0)
        for k in range(8):
            quiet.update(np.array([0.0, 0.0, -3.0]), 10.0, 1.0 + k, 1.0, conf=0.9)
        weak = tracker.update(np.array([40.0, 0.0, -3.0]), 50.0, 0.0)
        for k in range(8):
            weak.update(np.array([40.0, 0.0, -3.0]), 50.0, 1.0 + k, 1.0, conf=0.5)
        tracker.refresh_hard()
        self.assertTrue(quiet.hard_anchor)
        self.assertFalse(weak.hard_anchor)


if __name__ == "__main__":
    unittest.main()
