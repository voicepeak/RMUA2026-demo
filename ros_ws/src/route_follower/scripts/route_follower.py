#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RMUA 2026 第四阶段: XY 路径 + Gate 高度锚点 -> z_ref(s)。

在第三阶段基础上, Z 不再直接来自 route z 或固定值, 而是由 Altitude Planner
根据 "路径进度 s" 与 "有效 Gate 高度锚点" 生成:
  - 相邻锚点 Smoothstep 插值 -> z_route(s)
  - 靠近下一道有效 Gate 时按距离做 Gate Z Blend
  - Gate Z 异常保护 (跳变过大则该帧不采用)
  - z_ref 变化率限制
  - Gate 需 valid=true 才参与 (占位/未采集的门 valid=false)

XY 路径跟踪、Boundary Guard、Gate 顺序管理与三点穿越保持不变。
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
from altitude_planner import AltitudePlanner  # noqa: E402


class RouteFollower(object):

    def __init__(self):
        cfg_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "config"))
        self.route_file = rospy.get_param("~route_file",
                                          os.path.join(cfg_dir, "route_1_3.yaml"))
        self.route_name = rospy.get_param("~route_name", "route_1_3")
        self.gates_file = rospy.get_param("~gates_file",
                                          os.path.join(cfg_dir, "gates_1_3.yaml"))
        self.guides_file = rospy.get_param("~guides_file",
                                           os.path.join(cfg_dir, "altitude_guides_1_3.yaml"))
        self.gate_z_uses_offset = bool(rospy.get_param("~gate_z_uses_offset", True))

        self.forward_speed = rospy.get_param("~forward_speed", 2.0)
        self.k_cross = rospy.get_param("~k_cross", 0.7)
        self.lookahead = rospy.get_param("~lookahead", 5.0)
        self.soft_boundary = rospy.get_param("~soft_boundary", 2.0)
        self.hard_boundary = rospy.get_param("~hard_boundary", 3.0)

        self.k_z = rospy.get_param("~k_z", 0.6)
        self.vmax_z = rospy.get_param("~vmax_z", 1.5)
        self.vmax_xy = rospy.get_param("~vmax_xy", 2.5)
        self.z_ref = rospy.get_param("~z_ref", -3.5)
        self.use_route_z = bool(rospy.get_param("~use_route_z", False))
        # 起始高度锚点覆盖 (默认用进入时实际高度)
        self.start_anchor_z = rospy.get_param("~start_anchor_z", -999.0)
        self.start_anchor_z = None if self.start_anchor_z == -999.0 else self.start_anchor_z

        # Z 合法高度包络 (pose 坐标系)
        self.z_safe_min = rospy.get_param("~z_safe_min", -4.9)
        self.z_safe_max = rospy.get_param("~z_safe_max", 8.0)
        self.z_soft_margin = rospy.get_param("~z_soft_margin", 0.5)

        # Gate / Altitude Planner
        self.gate_blend_start = rospy.get_param("~gate_blend_start", 15.0)
        self.gate_blend_full = rospy.get_param("~gate_blend_full", 5.0)
        self.gate_align_dist = rospy.get_param("~gate_align_dist", 12.0)
        self.gate_k = rospy.get_param("~gate_k", 1.2)
        self.gate_speed = rospy.get_param("~gate_speed", 1.5)
        self.gate_xy_tol = rospy.get_param("~gate_xy_tol", 1.2)
        self.gate_z_tol = rospy.get_param("~gate_z_tol", 0.6)
        self.z_rate_max = rospy.get_param("~z_rate_max", 1.0)
        self.gate_z_max_jump = rospy.get_param("~gate_z_max_jump", 15.0)
        self.z_err_slow = rospy.get_param("~z_err_slow", 1.0)

        self.segment_switch_t = rospy.get_param("~segment_switch_t", 0.85)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 1.5)
        self.accel = int(rospy.get_param("~accel", 8))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.dt = 1.0 / self.control_rate

        self.pose = None
        self.yaw = 0.0
        self.seg = None
        self.z_offset = 0.0
        self.reached = False

        self.route = self.load_route(self.route_file, self.route_name)
        self.gates = self.load_gates(self.gates_file)
        self.guides = self.load_guides(self.guides_file)
        self.build_progress()
        for g in self.gates:
            g["s"] = self.project_s(g)
        self.gates.sort(key=lambda g: g["s"])
        self.gate_idx = 0
        self.prev_s = None
        self.planner = None
        rospy.loginfo("route '%s': %d pts, %.1f m; gates: %d",
                      self.route_name, len(self.route), self.route_length(),
                      len(self.gates))

        self.cmd_pub = rospy.Publisher(
            "/airsim_node/drone_1/vel_body_cmd", VelCmd, queue_size=1)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt",
                         PoseStamped, self.pose_cb)
        rospy.Timer(rospy.Duration(self.dt), self.control_loop)

    # ---------------- loaders / geometry ----------------
    @staticmethod
    def load_route(path, name):
        with open(path) as f:
            data = yaml.safe_load(f)
        routes = data["routes"]
        if name not in routes:
            raise KeyError("route '%s' not in %s (have %s)" % (name, path, list(routes)))
        pts = [Point(float(p[0]), float(p[1]), float(p[2])) for p in routes[name]]
        if len(pts) < 2:
            raise ValueError("route needs at least 2 points")
        return pts

    @staticmethod
    def load_gates(path):
        if not path or not os.path.exists(path):
            return []
        with open(path) as f:
            data = yaml.safe_load(f)
        return data.get("gates", []) if data else []

    @staticmethod
    def load_guides(path):
        if not path or not os.path.exists(path):
            return []
        with open(path) as f:
            data = yaml.safe_load(f)
        return data.get("altitude_guides", []) if data else []

    def route_length(self):
        total = 0.0
        for a, b in zip(self.route[:-1], self.route[1:]):
            total += math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2 + (b.z - a.z) ** 2)
        return total

    def build_progress(self):
        # XY 弧长 (用于 path progress s); z 由 Altitude Planner 单独处理
        self.seg_len, self.seg_s = [], [0.0]
        acc = 0.0
        for a, b in zip(self.route[:-1], self.route[1:]):
            L = math.hypot(b.x - a.x, b.y - a.y)
            self.seg_len.append(L)
            acc += L
            self.seg_s.append(acc)

    def project_segment(self, idx, p):
        a, b = self.route[idx], self.route[idx + 1]
        dx, dy = b.x - a.x, b.y - a.y
        denom = dx * dx + dy * dy
        if denom < 1e-9:
            return 0.0, a, 0.0
        t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / denom
        t = max(0.0, min(1.0, t))
        c = Point(a.x + t * dx, a.y + t * dy, a.z + t * (b.z - a.z))
        return t, c, denom ** 0.5

    def project_s(self, g):
        best_s, best_d = 0.0, float("inf")
        for i in range(len(self.route) - 1):
            a, b = self.route[i], self.route[i + 1]
            dx, dy = b.x - a.x, b.y - a.y
            denom = dx * dx + dy * dy
            t = 0.0 if denom < 1e-9 else ((g["x"] - a.x) * dx + (g["y"] - a.y) * dy) / denom
            t = max(0.0, min(1.0, t))
            cx, cy = a.x + t * dx, a.y + t * dy
            d = math.hypot(g["x"] - cx, g["y"] - cy)
            if d < best_d:
                best_d, best_s = d, self.seg_s[i] + t * self.seg_len[i]
        return best_s

    def progress(self, idx, t):
        return self.seg_s[idx] + t * self.seg_len[idx]

    def init_segment(self, p):
        best, best_d, best_c = 0, float("inf"), None
        for i in range(len(self.route) - 1):
            _, c, _ = self.project_segment(i, p)
            d = math.hypot(c.x - p.x, c.y - p.y)
            if d < best_d:
                best, best_d, best_c = i, d, c
        self.seg = best
        self.z_offset = p.z - best_c.z

        # 中途启动时跳过已经走过的 Gate
        t0, _, _ = self.project_segment(best, p)
        s_now = self.progress(best, t0)
        while self.gate_idx < len(self.gates) and self.gates[self.gate_idx]["s"] <= s_now - 2.0:
            self.gate_idx += 1

        # 构建 Altitude Planner: START 用进入时实际高度(或覆盖值), GOAL 用最后一个有效 Gate Z
        start_z = self.start_anchor_z if self.start_anchor_z is not None else p.z
        valid_z = [g["z"] + (self.z_offset if self.gate_z_uses_offset else 0.0)
                   for g in self.gates if g.get("valid", False)]
        goal_z = valid_z[-1] if valid_z else start_z
        anchors = [{"s": g["s"],
                    "z": g["z"] + (self.z_offset if self.gate_z_uses_offset else 0.0),
                    "valid": g.get("valid", False)} for g in self.gates]
        guides = [{"s": gd["s"],
                   "z": gd["z"] + (self.z_offset if self.gate_z_uses_offset else 0.0)}
                  for gd in self.guides if gd.get("s") is not None]
        self.planner = AltitudePlanner(
            0.0, start_z, self.seg_s[-1], goal_z, anchors, guides,
            self.gate_blend_start, self.gate_blend_full,
            self.z_rate_max, self.gate_z_max_jump)
        rospy.loginfo("entry segment=%d d_cross=%.2f z_offset=%.2f valid_gates=%d start_z=%.2f",
                      best, best_d, self.z_offset, len(valid_z), start_z)

    def lookahead_point(self, idx, t, look):
        remaining = look
        while True:
            a, b = self.route[idx], self.route[idx + 1]
            dx, dy = b.x - a.x, b.y - a.y
            seglen = math.hypot(dx, dy)
            if seglen < 1e-6:
                if idx < len(self.route) - 2:
                    idx += 1
                    t = 0.0
                    continue
                return self.route[-1]
            to_end = (1.0 - t) * seglen
            if remaining <= to_end:
                ratio = (t * seglen + remaining) / seglen
                return Point(a.x + ratio * dx, a.y + ratio * dy,
                             a.z + ratio * (b.z - a.z))
            remaining -= to_end
            if idx < len(self.route) - 2:
                idx += 1
                t = 0.0
            else:
                return b

    # ---------------- callbacks ----------------
    def pose_cb(self, msg):
        self.pose = msg.pose
        _, _, yaw = tft.euler_from_quaternion([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w])
        self.yaw = yaw

    def publish_cmd(self, vx, vy, vz):
        cmd = VelCmd()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "drone_1"
        cmd.vx = vx
        cmd.vy = vy
        cmd.vz = vz
        cmd.yawRate = 0.0
        cmd.va = self.accel
        cmd.stop = 0
        self.cmd_pub.publish(cmd)

    # ---------------- main loop ----------------
    def control_loop(self, _event):
        if self.pose is None:
            return
        p = self.pose.position
        if self.seg is None:
            self.init_segment(p)

        t, c, _ = self.project_segment(self.seg, p)
        d_cross = math.hypot(c.x - p.x, c.y - p.y)
        if t > self.segment_switch_t and self.seg < len(self.route) - 2:
            self.seg += 1

        a, b = self.route[self.seg], self.route[self.seg + 1]
        ux, uy = b.x - a.x, b.y - a.y
        ulen = math.hypot(ux, uy)
        ux, uy = (ux / ulen, uy / ulen) if ulen > 1e-6 else (0.0, 0.0)
        q = self.lookahead_point(self.seg, t, self.lookahead)

        # ---- 路径 XY 速度 ----
        vfx, vfy = self.forward_speed * ux, self.forward_speed * uy
        vcx, vcy = self.k_cross * (c.x - p.x), self.k_cross * (c.y - p.y)
        if d_cross < self.soft_boundary:
            scale = 1.0
        elif d_cross < self.hard_boundary:
            scale, vcx, vcy = 0.5, vcx * 2, vcy * 2
        else:
            scale, vcx, vcy = 0.0, vcx * 3, vcy * 3
        vwx = scale * vfx + vcx
        vwy = scale * vfy + vcy

        # ---- Gate 信息 (仅有效门参与 Z/对准) ----
        s_prog = self.progress(self.seg, t)
        plan_gate = None
        d_g = float("inf")
        gate = None
        if self.gate_idx < len(self.gates):
            gate = self.gates[self.gate_idx]
            gz = gate["z"] + (self.z_offset if self.gate_z_uses_offset else 0.0)
            dx, dy, dz = p.x - gate["x"], p.y - gate["y"], p.z - gz
            d_g = math.sqrt(dx * dx + dy * dy + dz * dz)
            s_plane = dx * gate["nx"] + dy * gate["ny"] + dz * gate.get("nz", 0.0)
            d_perp = math.hypot(dx - s_plane * gate["nx"], dy - s_plane * gate["ny"])
            plan_gate = {"s": gate["s"], "z": gz, "valid": gate.get("valid", False)}

            # 穿门判定 (由平面负侧到正侧, 横向/高度满足容差)
            if (self.prev_s is not None and self.prev_s < 0.0 <= s_plane
                    and d_perp < self.gate_xy_tol and abs(dz) < self.gate_z_tol):
                rospy.loginfo("GATE %d PASSED (d_perp=%.2f dz=%.2f)",
                              gate.get("id", self.gate_idx), d_perp, dz)
                self.gate_idx += 1
                self.prev_s = None
            else:
                self.prev_s = s_plane

        # ---- Altitude Planner: z_ref(s) ----
        z_ref, alt_mode, alpha = self.planner.compute(
            s_prog, p.z, plan_gate, d_g, self.dt)

        # ---- Gate 三点对准 (仅有效门, 靠近时) ----
        if gate is not None and gate.get("valid", False) and d_g < self.gate_align_dist:
            if s_plane < -1.0:
                tx, ty = gate["x"], gate["y"]
            else:
                dd = gate.get("approach_distance", 4.0)
                tx, ty = gate["x"] + dd * gate["nx"], gate["y"] + dd * gate["ny"]
            gsp = gate.get("speed", self.gate_speed)
            vgx, vgy = self.gate_k * (tx - p.x), self.gate_k * (ty - p.y)
            sp = math.hypot(vgx, vgy)
            if sp > gsp and sp > 1e-6:
                vgx *= gsp / sp
                vgy *= gsp / sp
            vwx = (1.0 - alpha) * vwx + alpha * vgx
            vwy = (1.0 - alpha) * vwy + alpha * vgy

        # ---- Z Safety Clamp ----
        z_ref = max(self.z_safe_min, min(self.z_safe_max, z_ref))
        z_err = p.z - z_ref
        vz = self.k_z * z_err
        vz = max(-self.vmax_z, min(self.vmax_z, vz))
        if p.z <= self.z_safe_min + self.z_soft_margin:
            vz = min(vz, 0.0)
        if p.z >= self.z_safe_max - self.z_soft_margin:
            vz = max(vz, 0.0)

        state = alt_mode
        if abs(z_err) > self.z_err_slow:
            vwx *= 0.5
            vwy *= 0.5
            state += "+ZSLOW"

        # ---- 水平限速 ----
        sp = math.hypot(vwx, vwy)
        if sp > self.vmax_xy and sp > 1e-6:
            vwx *= self.vmax_xy / sp
            vwy *= self.vmax_xy / sp

        # ---- 终点 ----
        goal = self.route[-1]
        d_goal = math.hypot(goal.x - p.x, goal.y - p.y)
        if (self.gate_idx >= len(self.gates)
                and self.seg >= len(self.route) - 2 and t > 0.95
                and d_goal < self.goal_tolerance):
            self.publish_cmd(0.0, 0.0, 0.0)
            if not self.reached:
                self.reached = True
                rospy.loginfo("GOAL_REACHED")
            return
        self.reached = False

        # ---- World -> Body ----
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        vx_b = cy * vwx + sy * vwy
        vy_b = -sy * vwx + cy * vwy
        self.publish_cmd(vx_b, vy_b, vz)

        rospy.loginfo_throttle(
            2.0,
            "%s s=%.1f SEG %d/%d CROSS=%.2f GATE %d/%d zRef=%.2f zErr=%.2f "
            "a=%.2f VW(%.2f,%.2f,%.2f) dgoal=%.1f",
            state, s_prog, self.seg, len(self.route) - 2, d_cross,
            self.gate_idx, len(self.gates), z_ref, z_err, alpha,
            vwx, vwy, vz, d_goal)


if __name__ == "__main__":
    rospy.init_node("route_follower")
    RouteFollower()
    rospy.spin()
