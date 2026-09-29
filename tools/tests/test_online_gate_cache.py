import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "ros_ws" / "src" / "route_follower" / "scripts"))
from online_gate_cache import OnlineGateCache  # noqa: E402


def observation(s=100.0, x=0.0, y=0.0, z=-3.0, support=10, sigma_x=0.5,
                sigma_y=0.5, sigma_z=0.3, geometry_valid=True, confidence=0.9,
                hard=True, last_seen=10.0, tid=0):
    return dict(id=tid, x=x, y=y, z=z, s=s, support=support, sigma_x=sigma_x,
                sigma_y=sigma_y, sigma_z=sigma_z, geometry_valid=geometry_valid,
                confidence=confidence, hard_anchor=hard, last_seen=last_seen)


def project(g):
    return g["s"]


class OnlineGateCacheTests(unittest.TestCase):
    def setUp(self):
        self.cache = OnlineGateCache()

    def ingest(self, obs, s_now=0.0, now=10.5, static_s=(), completed=()):
        return self.cache.ingest([obs], static_s, s_now, now, project, set(completed))

    def test_accepts_stable_hard_anchor_ahead(self):
        self.assertTrue(self.ingest(observation(s=100.0)))
        self.assertEqual(len(self.cache.gates), 1)
        gate = self.cache.gates[0]
        self.assertEqual(gate["id"], 100000)
        self.assertAlmostEqual(gate["s"], 100.0)

    def test_rejects_static_overlap(self):
        self.assertFalse(self.ingest(observation(s=103.0), static_s=[100.0]))
        self.assertEqual(self.cache.gates, [])

    def test_rejects_untrusted_observations(self):
        cases = dict(
            soft=observation(hard=False),
            stale=observation(last_seen=8.0),
            future=observation(last_seen=11.0),
            low_support=observation(support=4),
            wide_sigma_xy=observation(sigma_x=1.5),
            wide_sigma_z=observation(sigma_z=0.8),
            weak_dense=observation(geometry_valid=False, confidence=0.5),
        )
        for name, obs in cases.items():
            with self.subTest(name=name):
                self.assertFalse(self.ingest(obs))
                self.assertEqual(self.cache.gates, [])

    def test_rejects_gates_behind_current_progress(self):
        self.assertFalse(self.ingest(observation(s=1.5), s_now=0.0))
        self.assertFalse(self.ingest(observation(s=2.0), s_now=0.0))
        self.assertTrue(self.ingest(observation(s=2.5), s_now=0.0))

    def test_dense_stereo_needs_repeated_support(self):
        self.assertFalse(self.ingest(observation(geometry_valid=False, support=5)))
        self.assertTrue(self.ingest(observation(geometry_valid=False, support=8)))

    def test_match_updates_ema_and_keeps_identity(self):
        self.ingest(observation(s=100.0, x=0.0, last_seen=10.0))
        self.ingest(observation(s=101.0, x=1.0, last_seen=11.0), now=11.5)
        self.assertEqual(len(self.cache.gates), 1)
        gate = self.cache.gates[0]
        self.assertEqual(gate["id"], 100000)
        self.assertAlmostEqual(gate["x"], 0.3)
        self.assertAlmostEqual(gate["s"], 100.3)

    def test_completed_gate_is_neither_updated_nor_reused(self):
        self.ingest(observation(s=100.0))
        gid = self.cache.gates[0]["id"]
        changed = self.cache.ingest([observation(s=101.0, x=0.5, last_seen=11.0)],
                                    [], 0.0, 11.5, project, {gid})
        self.assertFalse(changed)
        self.assertEqual(len(self.cache.gates), 1)
        self.assertAlmostEqual(self.cache.gates[0]["x"], 0.0)

    def test_non_newer_observation_does_not_update(self):
        self.ingest(observation(s=100.0, last_seen=10.0))
        self.assertFalse(self.ingest(observation(s=100.5, x=2.0, last_seen=10.0),
                                     now=10.2))
        self.assertAlmostEqual(self.cache.gates[0]["x"], 0.0)

    def test_track_identity_survives_large_xy_jump(self):
        self.ingest(observation(s=100.0, x=0.0, last_seen=10.0))
        self.ingest(observation(s=101.0, x=8.0, last_seen=11.0), now=11.5)
        self.assertEqual(len(self.cache.gates), 1)
        self.assertAlmostEqual(self.cache.gates[0]["x"], 2.4)

    def test_track_id_change_reassociates_spatially_and_by_new_id(self):
        self.ingest(observation(s=100.0, x=0.0, last_seen=10.0, tid=1))
        changed = self.ingest(observation(s=100.5, x=0.5, last_seen=11.0, tid=2),
                              now=11.5)
        self.assertTrue(changed)
        self.assertEqual(len(self.cache.gates), 1)
        self.assertIs(self.cache.by_track_id[1], self.cache.gates[0])
        self.assertIs(self.cache.by_track_id[2], self.cache.gates[0])

    def test_static_exclusion_covers_8m(self):
        self.assertFalse(self.ingest(observation(s=107.5), static_s=[100.0]))
        self.assertTrue(self.ingest(observation(s=110.5), static_s=[100.0]))


if __name__ == "__main__":
    unittest.main()
