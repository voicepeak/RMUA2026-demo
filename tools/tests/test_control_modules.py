import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "ros_ws" / "src" / "route_follower" / "scripts"))
from mission_state import MissionState, TRACK, RECON, HOLD, ABORT  # noqa: E402
from xy_tracker import XYTracker  # noqa: E402
from z_controller import ZController  # noqa: E402


class XYTrackerTests(unittest.TestCase):
    def setUp(self):
        self.tracker = XYTracker(lookahead_base=8.0, lookahead_kv=0.7,
                                 k_pursuit=1.2, xy_converge=0.5,
                                 gate_blend_start=25.0, gate_blend_full=8.0,
                                 exit_blend_distance=4.0)

    def test_lookahead_scales_with_speed(self):
        self.assertAlmostEqual(self.tracker.lookahead(10.0), 15.0)

    def test_route_target_without_gates(self):
        tx, ty = self.tracker.target((0.0, 0.0), 0.0, 10.0, None, 1e9, None,
                                     lambda s: (s, 0.0, 0))
        self.assertAlmostEqual(tx, 15.0)
        self.assertAlmostEqual(ty, 0.0)

    def test_gate_pull_uses_smoothstep_bounds(self):
        self.assertAlmostEqual(self.tracker.gate_pull_weight(25.0), 0.0)
        self.assertAlmostEqual(self.tracker.gate_pull_weight(8.0), 0.5)
        self.assertAlmostEqual(self.tracker.gate_pull_weight(0.0), 0.5)
        self.assertGreater(self.tracker.gate_pull_weight(12.0), 0.0)

    def test_exit_blend_continuous_with_gate_pull(self):
        self.assertAlmostEqual(self.tracker.exit_weight(0.0),
                               self.tracker.gate_pull_weight(0.0))
        self.assertAlmostEqual(self.tracker.exit_weight(4.0), 0.0)

    def test_target_blends_towards_gate_center(self):
        gate = {"x": 10.0, "y": 5.0, "s": 0.0}
        tx, ty = self.tracker.target((0.0, 0.0), 0.0, 10.0, gate, 8.0, None,
                                     lambda s: (s, 0.0, 0))
        self.assertAlmostEqual(tx, 0.5 * 15.0 + 0.5 * 10.0)
        self.assertAlmostEqual(ty, 0.5 * 5.0)


class ZControllerTests(unittest.TestCase):
    def setUp(self):
        self.ctrl = ZController(k_z=1.0, k_ff_z=1.2, z_rate_max=4.0,
                                vz_up_limit=4.0, vz_down_limit=3.0,
                                vz_accel_limit=6.0)

    def test_reference_rate_limit(self):
        z, limited = self.ctrl.reference(lambda s: 0.0, 0.0, 0.1)
        self.assertFalse(limited)
        z, limited = self.ctrl.reference(lambda s: 1.0, 0.0, 0.1)
        self.assertTrue(limited)
        self.assertAlmostEqual(z, 0.4)
        z, limited = self.ctrl.reference(lambda s: 1.0, 0.0, 0.1)
        self.assertAlmostEqual(z, 0.8)

    def test_feedback_sign_in_ned(self):
        result = self.ctrl.track(z_ref=0.0, z_actual=1.0, dzds_preview=0.0,
                                 v=10.0, dt=0.1)
        self.assertGreater(result["vz_fb"], 0.0)
        self.assertAlmostEqual(result["vz_ff"], 0.0)

    def test_feedforward_sign_on_descent(self):
        result = self.ctrl.track(z_ref=0.0, z_actual=0.0, dzds_preview=0.1,
                                 v=10.0, dt=0.1)
        self.assertLess(result["vz_ff"], 0.0)

    def test_clamp_and_accel_limit(self):
        result = self.ctrl.track(z_ref=0.0, z_actual=100.0, dzds_preview=0.0,
                                 v=0.0, dt=0.01)
        self.assertLessEqual(result["vz_clamped"], 4.0)
        self.assertLessEqual(result["vz_cmd"], 0.06 + 1e-9)

    def test_anticipate_disabled_by_default(self):
        ctrl = ZController(k_anticipate=0.0)
        result = ctrl.track(z_ref=0.0, z_actual=0.0, dzds_preview=0.0, v=5.0,
                            dt=0.1, next_gate={"z": -20.0}, d_gate=10.0)
        self.assertFalse(result["anticipate_active"])
        self.assertAlmostEqual(result["vz_target"], 0.0)

    def test_recon_uses_constant_climb(self):
        result = self.ctrl.recon(z_actual=0.0, recon_start_z=0.0,
                                 recon_max_climb=4.0, recon_climb=0.6, dt=0.1)
        self.assertTrue(result["recon_climbing"])
        self.assertGreater(result["vz_cmd"], 0.0)
        result = self.ctrl.recon(z_actual=-5.0, recon_start_z=0.0,
                                 recon_max_climb=4.0, recon_climb=0.6, dt=0.1)
        self.assertFalse(result["recon_climbing"])


class MissionStateTests(unittest.TestCase):
    def test_track_without_gate_map(self):
        mission = MissionState()
        mode, v_map, transition = mission.update(False, None, 0.0, 0.0)
        self.assertEqual(mode, TRACK)
        self.assertIsNone(v_map)

    def test_track_limits_by_visibility(self):
        mission = MissionState(map_brake_a=1.5)
        mode, v_map, _ = mission.update(True, 50.0, 0.0, 0.0)
        self.assertEqual(mode, TRACK)
        self.assertAlmostEqual(v_map, (2.0 * 1.5 * 53.0) ** 0.5)

    def test_lost_horizon_enters_recon_then_hold(self):
        mission = MissionState(recon_speed=2.0, recon_max_dist=10.0)
        mode, v_map, transition = mission.update(True, None, 0.0, 0.0)
        self.assertEqual(mode, RECON)
        self.assertAlmostEqual(v_map, 2.0)
        self.assertEqual(transition["to"], RECON)
        mode, v_map, _ = mission.update(True, None, 4.0, -0.5)
        self.assertEqual(mode, RECON)
        mode, v_map, transition = mission.update(True, None, 11.0, -1.0)
        self.assertEqual(mode, HOLD)
        self.assertEqual(v_map, 0.0)
        self.assertEqual(transition["to"], HOLD)

    def test_abort_is_terminal(self):
        mission = MissionState()
        mission.abort()
        mode, v_map, _ = mission.update(True, 50.0, 0.0, 0.0)
        self.assertEqual(mode, ABORT)
        self.assertEqual(v_map, 0.0)


if __name__ == "__main__":
    unittest.main()
