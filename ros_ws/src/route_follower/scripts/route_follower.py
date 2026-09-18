#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RMUA 2026 Stage 7: 全 Gate 连续穿越 + 强 Gate 对准 + 避障预留。

架构:
  Route Follower  -> v_route_desired (XY)
  Gate Manager    -> 状态机 PATH->CAPTURE->ALIGN->CROSS->EXIT (+RECOVER)
  Gate Guidance   -> v_gate_desired (XY)
  Altitude Profile-> z_center(s) / 动态走廊 / z_ref (融合 gate.z)
  Command Arbiter -> v_final (唯一决定最终速度; 避障预留最高安全优先级)
  仅 Arbiter 之后才 World->Body -> vel_body_cmd

坐标: AirSim NED; vel_body_cmd 的 vz 向上为正。
"""

import math
import os
import sys

import rospy
import tf.transformations as tft
import yaml
from geometry_msgs.msg import Point, PoseStamped

from airsim_ros.msg import VelCmd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from altitude_profile import AltitudeProfile          # noqa: E402
from gate_manager import GateManager, ALIGN, CROSS, EXIT, RECOVER, CAPTURE  # noqa: E402
from gate_guidance import GateGuidance               # noqa: E402
from command_arbiter import CommandArbiter           # noqa: E402
from avoidance_interface import AvoidanceCommand     # noqa: E402


class RouteFollower(object):

    def __init__(self):
        cfg = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "config"))
        self.route_file = rospy.get_param("~route_file", os.path.join(cfg, "route_1_3.yaml"))
        self.route_name = rospy.get_param("~route_name", "route_1_3")
        self.gates_file = rospy.get_param("~gates_file", os.path.join(cfg, "gates_1_3.yaml"))
        self.guides_file = rospy.get_param("~guides_file", os.path.join(cfg, "none.yaml"))
        self.gate_z_uses_offset = bool(rospy.get_param("~gate_z_uses_offset", True))
        self.start_anchor_z = rospy.get_param("~start_anchor_z", -999.0)
        self.start_anchor_z = None if self.start_anchor_z == -999.0 else self.start_anchor_z

        self.forward_speed = rospy.get_param("~forward_speed", 2.0)
        self.k_cross = rospy.get_param("~k_cross", 0.7)
        self.lookahead = rospy.get_param("~lookahead", 5.0)
        self.soft_boundary = rospy.get_param("~soft_boundary", 2.0)
        self.hard_boundary = rospy.get_param("~hard_boundary", 3.0)
        self.vmax_xy = rospy.get_param("~vmax_xy", 2.5)
        self.segment_switch_t = rospy.get_param("~segment_switch_t", 0.85)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 1.5)

        self.k_z = rospy.get_param("~k_z", 0.6)
        self.vmax_z = rospy.get_param("~vmax_z", 1.5)
        self.z_err_slow = rospy.get_param("~z_err_slow", 1.0)
        self.z_rate_max = rospy.get_param("~z_rate_max", 1.0)
        self.corridor_half = rospy.get_param("~corridor_half", 1.5)

        self.accel = int(rospy.get_param("~accel", 8))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.dt = 1.0 / self.control_rate

        # Gate 参数
        self.capture_start = rospy.get_param("~capture_start", 22.0)
        self.full_control = rospy.get_param("~full_control", 10.0)
        self.align_xy_tol = rospy.get_param("~align_xy_tol", 0.5)
        self.align_z_tol = rospy.get_param("~align_z_tol", 0.35)
        self.aperture_xy_tol = rospy.get_param("~aperture_xy_tol", 1.2)
        self.aperture_z_tol = rospy.get_param("~aperture_z_tol", 0.6)
        self.approach_distance = rospy.get_param("~approach_distance", 4.0)
        self.exit_distance = rospy.get_param("~exit_distance", 3.0)
        self.max_retry = rospy.get_param("~max_retry", 3)
        self.cross_speed = rospy.get_param("~cross_speed", 1.2)
        self.gate_k_near = rospy.get_param("~gate_k_near", 2.0)

        self.pose = None
        self.yaw = 0.0
        self.seg = None
        self.z_offset = 0.0
        self.reached = False
        self.avoidance = AvoidanceCommand()

        self.route = self.load_route(self.route_file, self.route_name)
        self.gates = self.load_list(self.gates_file, "gates")
        self.guides = self.load_list(self.guides_file, "altitude_guides")
        self.build_progress()
        for g in self.gates:
            g["s"] = self.project_s(g)
        self.gates.sort(key=lambda g: g["s"])
        self.gate_mgr = None
        self.profile = None
        self.guidance = None
        self.arbiter = CommandArbiter(self.avoidance)

        rospy.loginfo("route '%s': %d pts; gates=%d", self.route_name, len(self.route), len(self.gates))
        self.cmd_pub = rospy.Publisher("/airsim_node/drone_1/vel_body_cmd", VelCmd, queue_size=1)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.pose_cb)
        rospy.Timer(rospy.Duration(self.dt), self.control_loop)

    # ---------- loaders ----------
    @staticmethod
    def load_route(path, name):
        with open(path) as f:
            pts = yaml.safe_load(f)["routes"][name]
        return [Point(float(p[0]), float(p[1]), float(p[2])) for p in pts]

    @staticmethod
    def load_list(path, key):
        if not path or not os.path.exists(path):
            return []
        with open(path) as f:
            d = yaml.safe_load(f)
        return d.get(key, []) if d else []

    def build_progress(self):
        self.seg_len, self.seg_s = [], [0.0]
        acc = 0.0
        for a, b in zip(self.route[:-1], self.route[1:]):
            L = math.hypot(b.x - a.x, b.y - a.y)
            self.seg_len.append(L)
            acc += L
            self.seg_s.append(acc)

    def project_segment(self, i, p):
        a, b = self.route[i], self.route[i + 1]
        dx, dy = b.x - a.x, b.y - a.y
        den = dx * dx + dy * dy
        if den < 1e-9:
            return 0.0, a, 0.0
        t = max(0.0, min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / den))
        return t, Point(a.x + t * dx, a.y + t * dy, a.z + t * (b.z - a.z)), den ** 0.5

    def project_s(self, g):
        best_s, best_d = 0.0, 1e9
        for i in range(len(self.route) - 1):
            a, b = self.route[i], self.route[i + 1]
            dx, dy = b.x - a.x, b.y - a.y
            den = dx * dx + dy * dy
            t = 0.0 if den < 1e-9 else max(0.0, min(1.0, ((g["x"] - a.x) * dx + (g["y"] - a.y) * dy) / den))
            cx, cy = a.x + t * dx, a.y + t * dy
            d = math.hypot(g["x"] - cx, g["y"] - cy)
            if d < best_d:
                best_d, best_s = d, self.seg_s[i] + t * self.seg_len[i]
        return best_s

    def progress(self, i, t):
        return self.seg_s[i] + t * self.seg_len[i]

    def init_segment(self, p):
        best, best_d, best_c = 0, 1e9, None
        for i in range(len(self.route) - 1):
            _, c, _ = self.project_segment(i, p)
            d = math.hypot(c.x - p.x, c.y - p.y)
            if d < best_d:
                best, best_d, best_c = i, d, c
        self.seg = best
        self.z_offset = p.z - best_c.z

        off = self.z_offset if self.gate_z_uses_offset else 0.0
        gates_adj = [dict(g, z=g["z"] + off) for g in self.gates]
        start_z = self.start_anchor_z if self.start_anchor_z is not None else p.z
        goal_z = gates_adj[-1]["z"] if gates_adj else start_z
        guides_adj = [{"s": gd["s"], "z": gd["z"] + off} for gd in self.guides if gd.get("s") is not None]

        self.gate_mgr = GateManager(
            gates_adj, capture_start=self.capture_start, full_control=self.full_control,
            align_xy_tol=self.align_xy_tol, align_z_tol=self.align_z_tol,
            aperture_xy_tol=self.aperture_xy_tol, aperture_z_tol=self.aperture_z_tol,
            approach_distance=self.approach_distance, exit_distance=self.exit_distance,
            max_retry=self.max_retry)
        self.profile = AltitudeProfile(
            self.seg_s[0], start_z, self.seg_s[-1], goal_z, gates_adj, guides_adj,
            corridor_half=self.corridor_half, gate_blend_start=self.capture_start,
            gate_blend_full=6.0, z_rate_max=self.z_rate_max)
        self.guidance = GateGuidance(cross_speed=self.cross_speed,
                                     k_near=self.gate_k_near,
                                     approach_distance=self.approach_distance)

        t0, _, _ = self.project_segment(best, p)
        s_now = self.progress(best, t0)
        while self.gate_mgr.idx < len(gates_adj) and gates_adj[self.gate_mgr.idx]["s"] <= s_now - 2.0:
            self.gate_mgr.idx += 1
        rospy.loginfo("entry seg=%d z_offset=%.2f gates=%d start_z=%.2f",
                      best, self.z_offset, len(gates_adj), start_z)

    def lookahead(self, i, t):
        a, b = self.route[i], self.route[i + 1]
        dx, dy = b.x - a.x, b.y - a.y
        L = math.hypot(dx, dy) or 1.0
        return Point(a.x + dx / L * self.lookahead, a.y + dy / L * self.lookahead, a.z)

    # ---------- callbacks ----------
    def pose_cb(self, m):
        self.pose = m.pose
        _, _, self.yaw = tft.euler_from_quaternion(
            [m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w])

    def publish(self, vx, vy, vz):
        c = VelCmd()
        c.header.stamp = rospy.Time.now()
        c.header.frame_id = "drone_1"
        c.vx, c.vy, c.vz = vx, vy, vz
        c.yawRate = 0.0
        c.va = self.accel
        c.stop = 0
        self.cmd_pub.publish(c)

    # ---------- main ----------
    def control_loop(self, _e):
        if self.pose is None:
            return
        p = self.pose.position
        pn = [p.x, p.y, p.z]
        if self.seg is None:
            self.init_segment(p)

        t, c, _ = self.project_segment(self.seg, p)
        d_cross = math.hypot(c.x - p.x, c.y - p.y)
        if t > self.segment_switch_t and self.seg < len(self.route) - 2:
            self.seg += 1
        s_prog = self.progress(self.seg, t)

        # ---- Route desired (XY) ----
        a, b = self.route[self.seg], self.route[self.seg + 1]
        ux, uy = b.x - a.x, b.y - a.y
        L = math.hypot(ux, uy)
        ux, uy = (ux / L, uy / L) if L > 1e-6 else (0.0, 0.0)
        vfx, vfy = self.forward_speed * ux, self.forward_speed * uy
        vcx, vcy = self.k_cross * (c.x - p.x), self.k_cross * (c.y - p.y)
        if d_cross < self.soft_boundary:
            scale = 1.0
        elif d_cross < self.hard_boundary:
            scale, vcx, vcy = 0.5, vcx * 2, vcy * 2
        else:
            scale, vcx, vcy = 0.0, vcx * 3, vcy * 3
        v_route = (scale * vfx + vcx, scale * vfy + vcy)

        # ---- Gate Manager / Guidance ----
        info = self.gate_mgr.step(pn, self.dt)
        if info["passed"]:
            gid = info["gate"].get("id", self.gate_mgr.idx - 1)
            rospy.loginfo("GATE %d PASSED (idx->%d)", gid, self.gate_mgr.idx)
        v_gate = self.guidance.compute(info, pn)[:2]
        v_final, chosen = self.arbiter.arbitrate(v_route, v_gate, info["state"], info["alpha"])

        # ---- Altitude Profile + 动态走廊 ----
        ng = info["gate"]
        z_gate = {"z": float(info["G"][2]), "valid": True} if ng is not None else None
        z_ref, z_mode, alpha_z = self.profile.compute(
            s_prog, p.z, z_gate, info["d_g"], self.dt)
        z_ref = self.profile.clamp_corridor(z_ref, s_prog)
        z_err = p.z - z_ref
        vz = max(-self.vmax_z, min(self.vmax_z, self.k_z * z_err))

        if abs(z_err) > self.z_err_slow:
            v_final = (v_final[0] * 0.5, v_final[1] * 0.5)

        # ---- 水平限速 ----
        sp = math.hypot(*v_final)
        if sp > self.vmax_xy and sp > 1e-6:
            v_final = (v_final[0] * self.vmax_xy / sp, v_final[1] * self.vmax_xy / sp)

        # ---- 终点 ----
        goal = self.route[-1]
        d_goal = math.hypot(goal.x - p.x, goal.y - p.y)
        if (self.gate_mgr.idx >= len(self.gates)
                and self.seg >= len(self.route) - 2 and t > 0.95
                and d_goal < self.goal_tolerance):
            self.publish(0.0, 0.0, 0.0)
            if not self.reached:
                self.reached = True
                rospy.loginfo("GOAL_REACHED")
            return

        # ---- World -> Body ----
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        vx_b = cy * v_final[0] + sy * v_final[1]
        vy_b = -sy * v_final[0] + cy * v_final[1]
        self.publish(vx_b, vy_b, vz)

        g = info["gate"]
        rospy.loginfo_throttle(
            2.0,
            "%s|%s idx=%d dG=%.1f a=%.2f lat=%.2f vert=%.2f en=%.2f zRef=%.2f zErr=%.2f "
            "kxy=%.1f spam=%.1f vR(%.1f,%.1f) vG(%.1f,%.1f) vF(%.1f,%.1f) avoid=%s",
            chosen, info["state"], self.gate_mgr.idx, info["d_g"], info["alpha"],
            info["lat"], info["vert"], info["e_n"], z_ref, z_err,
            self.guidance.k_xy(info["d_g"]), self.guidance.speed(info["d_g"]),
            v_route[0], v_route[1], v_gate[0], v_gate[1], v_final[0], v_final[1],
            self.avoidance.active)


if __name__ == "__main__":
    rospy.init_node("route_follower")
    RouteFollower()
    rospy.spin()
