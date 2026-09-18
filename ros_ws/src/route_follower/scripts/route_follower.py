#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RMUA 2026 Stage 7 v3: Multi-Gate Lookahead + Ordered Chain + 动态 Z + 提速。

核心变化 (相对 v2):
  - 取消 Gate 强吸附; 改为 "参考路径 + Look-ahead 跟踪"
  - Gate Chain: 顺序约束 + Z 坡度离群剔除; 门作为软/硬控制点 (主要影响 Z)
  - 动态 Look-ahead: L = L0 + kv * v
  - 速度调度: cruise 与曲率/ Z 坡度限速取 min; 仅预测穿门失败时减速
  - Gate Pass 仍保留: 过门平面 + 门洞容差
  - 保留避障接口 (bypass)

坐标: AirSim NED; vel_body_cmd 的 vz 向上为正。
"""

import math
import os
import sys

import numpy as np
import rospy
import tf.transformations as tft
import yaml
from geometry_msgs.msg import Point, PoseStamped

from airsim_ros.msg import VelCmd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from altitude_profile import AltitudeProfile       # noqa: E402
from gate_chain import GateChain                  # noqa: E402
from command_arbiter import CommandArbiter        # noqa: E402
from avoidance_interface import AvoidanceCommand  # noqa: E402


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

        # speed
        self.cruise_speed = rospy.get_param("~cruise_speed", 4.0)
        self.max_speed = rospy.get_param("~max_speed", 6.0)
        self.min_speed = rospy.get_param("~min_speed", 2.0)
        self.gate_slow_speed = rospy.get_param("~gate_slow_speed", 1.5)
        self.k_curv = rospy.get_param("~k_curv", 1.0)
        self.eta_z = rospy.get_param("~eta_z", 1.0)
        self.lookahead_base = rospy.get_param("~lookahead_base", 5.0)
        self.lookahead_kv = rospy.get_param("~lookahead_kv", 1.5)
        self.k_pursuit = rospy.get_param("~k_pursuit", 1.2)

        # z
        self.k_z = rospy.get_param("~k_z", 0.6)
        self.vmax_z = rospy.get_param("~vmax_z", 1.5)
        self.z_err_slow = rospy.get_param("~z_err_slow", 1.0)
        self.z_rate_max = rospy.get_param("~z_rate_max", 1.5)
        self.corridor_half = rospy.get_param("~corridor_half", 1.5)
        self.gate_blend_start = rospy.get_param("~gate_blend_start", 25.0)
        self.gate_blend_full = rospy.get_param("~gate_blend_full", 8.0)

        # gate chain / pass
        self.aperture_xy_tol = rospy.get_param("~aperture_xy_tol", 1.2)
        self.aperture_z_tol = rospy.get_param("~aperture_z_tol", 0.6)
        self.slope_factor = rospy.get_param("~slope_factor", 3.0)
        self.slope_abs_max = rospy.get_param("~slope_abs_max", 0.6)
        self.xy_converge = rospy.get_param("~xy_converge", 0.5)
        self.snap_gate_to_route = bool(rospy.get_param("~snap_gate_to_route", True))
        self.start_gate = int(rospy.get_param("~start_gate", 0))
        self.end_gate = int(rospy.get_param("~end_gate", -1))

        self.accel = int(rospy.get_param("~accel", 8))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.dt = 1.0 / self.control_rate

        self.pose = None
        self.yaw = 0.0
        self.seg = None
        self.z_offset = 0.0
        self.reached = False
        self.prev_s_plane = None
        self.avoidance = AvoidanceCommand()
        self.arbiter = CommandArbiter(self.avoidance)

        self.route = self.load_route(self.route_file, self.route_name)
        self.gates = self.load_list(self.gates_file, "gates")
        self.guides = self.load_list(self.guides_file, "altitude_guides")
        self.build_route()

        rospy.loginfo("route '%s': %d pts, %.0f m; gates=%d", self.route_name, len(self.route),
                      self.seg_s[-1], len(self.gates))
        self.cmd_pub = rospy.Publisher("/airsim_node/drone_1/vel_body_cmd", VelCmd, queue_size=1)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.pose_cb)
        rospy.Timer(rospy.Duration(self.dt), self.control_loop)

    # ---------- load ----------
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

    def build_route(self):
        self.seg_len, self.seg_s = [], [0.0]
        acc = 0.0
        for a, b in zip(self.route[:-1], self.route[1:]):
            L = math.hypot(b.x - a.x, b.y - a.y)
            self.seg_len.append(L)
            acc += L
            self.seg_s.append(acc)

    def point_at(self, s):
        s = max(0.0, min(self.seg_s[-1], s))
        for i in range(len(self.seg_len)):
            if self.seg_s[i] <= s <= self.seg_s[i + 1]:
                r = (s - self.seg_s[i]) / self.seg_len[i] if self.seg_len[i] > 1e-9 else 0.0
                a, b = self.route[i], self.route[i + 1]
                return (a.x + r * (b.x - a.x), a.y + r * (b.y - a.y), i)
        a, b = self.route[-2], self.route[-1]
        return (b.x, b.y, len(self.route) - 2)

    def project(self, p):
        best = (0, 0.0, 1e9, None)
        for i in range(len(self.route) - 1):
            a, b = self.route[i], self.route[i + 1]
            dx, dy = b.x - a.x, b.y - a.y
            den = dx * dx + dy * dy
            t = 0.0 if den < 1e-9 else max(0.0, min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / den))
            cx, cy = a.x + t * dx, a.y + t * dy
            d = math.hypot(p.x - cx, p.y - cy)
            if d < best[2]:
                best = (i, t, d, Point(cx, cy, a.z + t * (b.z - a.z)))
        return best

    def curvature(self, s):
        _, _, i = self.point_at(s)
        i = max(1, min(i, len(self.route) - 3))
        a, b, c = self.route[i - 1], self.route[i], self.route[i + 1]
        v1 = np.array([b.x - a.x, b.y - a.y])
        v2 = np.array([c.x - b.x, c.y - b.y])
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        ang = abs(math.atan2(v1[0] * v2[1] - v1[1] * v2[0], v1 @ v2))
        ds = 0.5 * (n1 + n2)
        return ang / max(ds, 1e-3)

    def z_slope(self, s):
        h = 5.0
        z1 = self.profile.center(max(0.0, s - h))
        z2 = self.profile.center(min(self.seg_s[-1], s + h))
        return (z2 - z1) / (2 * h)

    def project_gate_s(self, g):
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

    # ---------- init ----------
    def init_once(self, p):
        i, t, d, c = self.project(p)
        self.seg = i
        self.z_offset = p.z - c.z
        off = self.z_offset if self.gate_z_uses_offset else 0.0
        gates_adj = [dict(g, z=g["z"] + off) for g in self.gates]
        for g in gates_adj:
            g["s"] = self.project_gate_s(g)
            if self.snap_gate_to_route:      # 门在道路中心: XY 吸附到路线中心线(消除视觉横向偏差)
                tx_, ty_, _ = self.point_at(g["s"])
                g["x"], g["y"] = tx_, ty_
        self.chain = GateChain(gates_adj, slope_factor=self.slope_factor,
                               slope_abs_max=self.slope_abs_max)
        start_z = self.start_anchor_z if self.start_anchor_z is not None else p.z
        prof_gates = [dict(g, valid=(not g["z_suspect"])) for g in self.chain.gates]
        valid_prof = [g for g in prof_gates if g["valid"]]
        goal_z = valid_prof[-1]["z"] if valid_prof else start_z
        guides_adj = [{"s": gd["s"], "z": gd["z"] + off} for gd in self.guides if gd.get("s") is not None]
        self.profile = AltitudeProfile(
            0.0, start_z, self.seg_s[-1], goal_z,
            prof_gates, guides_adj,
            corridor_half=self.corridor_half, gate_blend_start=self.gate_blend_start,
            gate_blend_full=self.gate_blend_full, z_rate_max=self.z_rate_max)
        # 用 slope-filtered anchors 重建 center
        self.suspect = self.chain.suspect_ids()

        s_now = self.seg_s[i] + t * self.seg_len[i]
        self.gate_idx = self.start_gate
        while self.gate_idx < len(self.chain.gates) and self.chain.gates[self.gate_idx]["s"] <= s_now - 2.0:
            self.gate_idx += 1
        rospy.loginfo("init seg=%d z_offset=%.2f gates=%d suspects=%s start_gate=%d",
                      i, self.z_offset, len(self.chain.gates), self.suspect, self.gate_idx)

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
        if self.seg is None:
            self.init_once(p)
        pn = np.array([p.x, p.y, p.z])

        i, t, d_cross, c = self.project(p)
        self.seg = i
        s_now = self.seg_s[i] + t * self.seg_len[i]

        # 甩在身后的门直接跳过 (错过穿门判定时不至于卡住)
        while (self.gate_idx < len(self.chain.gates)
               and self.chain.gates[self.gate_idx]["s"] < s_now - 12.0):
            rospy.logwarn("SKIP gate %s (left behind)", self.chain.gates[self.gate_idx].get("id"))
            self.gate_idx += 1
            self.prev_s_plane = None

        ng = self.chain.gates[self.gate_idx] if self.gate_idx < len(self.chain.gates) else None
        d_g = 1e9
        if ng is not None:
            G = np.array([ng["x"], ng["y"], ng["z"]])
            n = np.array([ng.get("nx", 1.0), ng.get("ny", 0.0), ng.get("nz", 0.0)])
            n = n / (np.linalg.norm(n) + 1e-9)
            r = pn - G
            e_n = float(r @ n)
            vert = float(r[2] - e_n * n[2])
            lat = float(math.hypot(r[0] - e_n * n[0], r[1] - e_n * n[1]))
            d_g = float(np.linalg.norm(r))
            if (self.prev_s_plane is not None and self.prev_s_plane < 0.0 <= e_n
                    and lat < self.aperture_xy_tol and abs(vert) < self.aperture_z_tol):
                rospy.loginfo("GATE %d PASSED (lat=%.2f vert=%.2f)", ng.get("id", self.gate_idx), lat, vert)
                self.gate_idx += 1
                self.prev_s_plane = None
                ng = self.chain.gates[self.gate_idx] if self.gate_idx < len(self.chain.gates) else None
            else:
                self.prev_s_plane = e_n

        # ---- 速度调度 ----
        v = self.cruise_speed
        kap = self.curvature(s_now)
        v_curv = self.cruise_speed / (1.0 + self.k_curv * kap)
        kz = abs(self.z_slope(s_now))
        v_zlim = self.eta_z * self.vmax_z / (kz + 1e-3)
        v = min(self.cruise_speed, v_curv, v_zlim)
        # 预测穿门失败才减速
        if ng is not None and d_g < 25.0 and (lat > self.aperture_xy_tol or abs(vert) > self.aperture_z_tol):
            v = min(v, self.gate_slow_speed)
        v = max(self.min_speed, min(self.max_speed, v))

        # ---- 动态 look-ahead 目标 ----
        L = self.lookahead_base + self.lookahead_kv * v
        tx, ty, _ = self.point_at(s_now + L)
        if ng is not None:
            a = (self.gate_blend_start - d_g) / max(1e-6, self.gate_blend_start - self.gate_blend_full)
            a = max(0.0, min(1.0, a))
            w = self.xy_converge * a
            tx = (1.0 - w) * tx + w * ng["x"]
            ty = (1.0 - w) * ty + w * ng["y"]
        vx = self.k_pursuit * (tx - p.x)
        vy = self.k_pursuit * (ty - p.y)
        v_route = self.arbiter.arbitrate((vx, vy), (0.0, 0.0), "PATH", 0.0)[0]

        # ---- 动态 Z 走廊 + gate z 融合 ----
        z_gate = {"z": float(ng["z"]), "valid": True} if ng is not None else None
        z_ref, z_mode, _ = self.profile.compute(s_now, p.z, z_gate, d_g, self.dt)
        z_ref = self.profile.clamp_corridor(z_ref, s_now)
        z_err = p.z - z_ref
        vz = max(-self.vmax_z, min(self.vmax_z, self.k_z * z_err))

        # 限速
        sp = math.hypot(*v_route)
        if sp > v and sp > 1e-6:
            v_route = (v_route[0] * v / sp, v_route[1] * v / sp)

        # ---- 到达 ----
        if self.end_gate >= 0 and self.gate_idx > self.end_gate:
            self.publish(0.0, 0.0, 0.0)
            self.reached = True
            return
        if self.gate_idx >= len(self.chain.gates) and s_now > self.seg_s[-1] - 5.0:
            self.publish(0.0, 0.0, 0.0)
            if not self.reached:
                rospy.loginfo("GOAL_REACHED")
            self.reached = True
            return

        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        vx_b = cy * v_route[0] + sy * v_route[1]
        vy_b = -sy * v_route[0] + cy * v_route[1]
        self.publish(vx_b, vy_b, vz)

        rospy.loginfo_throttle(
            2.0,
            "PATH s=%.0f v=%.2f kap=%.3f kz=%.3f idx=%d dG=%.1f lat=%.2f vert=%.2f "
            "zRef=%.2f zErr=%.2f CROSS=%.2f L=%.1f avoid=%s",
            s_now, v, kap, kz, self.gate_idx, d_g, lat if ng is not None else 0.0,
            vert if ng is not None else 0.0, z_ref, z_err, d_cross, L, self.avoidance.active)


if __name__ == "__main__":
    rospy.init_node("route_follower")
    RouteFollower()
    rospy.spin()
