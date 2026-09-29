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

    def configure(self, gates):
        from reference_planner import RouteGeometry
        self.tracker.configure(RouteGeometry([(0,0,0),(200,0,0)]), gates)

    def test_valid_off_center_crossing_keeps_route(self):
        self.configure([dict(s=20., x=20., y=.8)])
        self.assertAlmostEqual(self.tracker.point_at(20.)[1], 0.)

    def test_crossing_does_not_stop_or_reverse(self):
        gate = dict(s=20., x=20., y=0.)
        self.configure([gate, dict(s=40.,x=40.,y=0.)])
        self.tracker.previous_velocity = (6.,0.)
        for s in (19.9,20.,20.1,21.,24.):
            target = self.tracker.target((s,0),s,6.,gate,20-s,gate,None)
            vx,vy = self.tracker.velocity((s,0),target,6.)
            self.assertAlmostEqual(vx,6.)
            self.assertAlmostEqual(vy,0.)

    def test_offset_gate_path_stays_inside_opening(self):
        self.configure([dict(s=20.,x=20.,y=3.),dict(s=40.,x=40.,y=-3.)])
        for s,y in ((20.,3.),(40.,-3.)):
            self.assertLessEqual(abs(self.tracker.point_at(s)[1]-y),1.100001)

    def test_task_gate_switch_does_not_change_target(self):
        self.configure([dict(s=20.,x=20.,y=0.)])
        before=self.tracker.target((20,0),20,6,dict(s=20,x=20,y=0),0,None,None)
        after=self.tracker.target((20,0),20,6,None,1e9,dict(s=20,x=20,y=0),None)
        self.assertEqual(before,after)


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
