#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RMUA 2026 第三阶段: 3D 路径 + 检测门穿越。

在第二阶段 (Route Manager + Path Follower + Boundary Guard) 基础上新增:
  - Z Profile      : z_ref 沿路径线性插值, 并做 path<->gate 高度融合
  - Gate Manager   : gate_index 顺序管理下一道门
  - Gate Alignment : 靠近门时对准门中心, 用 Approach->Center->Exit 穿过
  - Gate Pass      : 通过门平面 + 横向/高度在容差内 => gate_index++

坐标: AirSim NED; vel_body_cmd 的 vz 向上为正。route/gate 的 z 通过进入时
自动标定的 z_offset 映射到 pose 坐标系。
"""

import math
import os

import rospy
import tf.transformations as tft
import yaml
from geometry_msgs.msg import Point, PoseStamped

from airsim_ros.msg import VelCmd


class RouteFollower(object):

    def __init__(self):
        cfg_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "config"))
        self.route_file = rospy.get_param("~route_file",
                                          os.path.join(cfg_dir, "route_1_3.yaml"))
        self.route_name = rospy.get_param("~route_name", "route_1_3")
        self.gates_file = rospy.get_param("~gates_file",
                                          os.path.join(cfg_dir, "gates_1_3.yaml"))
        self.gate_z_uses_offset = bool(rospy.get_param("~gate_z_uses_offset", True))

        self.forward_speed = rospy.get_param("~forward_speed", 2.0)
        self.k_cross = rospy.get_param("~k_cross", 0.7)
        self.lookahead = rospy.get_param("~lookahead", 5.0)
        self.soft_boundary = rospy.get_param("~soft_boundary", 2.0)
        self.hard_boundary = rospy.get_param("~hard_boundary", 3.0)

        self.k_z = rospy.get_param("~k_z", 0.6)
        self.vmax_z = rospy.get_param("~vmax_z", 1.5)
        self.vmax_xy = rospy.get_param("~vmax_xy", 2.5)
        self.z_ref = rospy.get_param("~z_ref", -1.5)
        self.use_route_z = bool(rospy.get_param("~use_route_z", True))

        # Z 合法高度包络 (pose 坐标系, 实测后填入; 默认很宽)
        self.z_safe_min = rospy.get_param("~z_safe_min", -20.0)
        self.z_safe_max = rospy.get_param("~z_safe_max", 10.0)
        self.z_soft_margin = rospy.get_param("~z_soft_margin", 0.5)

        self.gate_blend_start = rospy.get_param("~gate_blend_start", 25.0)
        self.gate_blend_full = rospy.get_param("~gate_blend_full", 10.0)
        self.gate_align_dist = rospy.get_param("~gate_align_dist", 12.0)
        self.gate_k = rospy.get_param("~gate_k", 1.2)
        self.gate_speed = rospy.get_param("~gate_speed", 1.5)
        self.gate_xy_tol = rospy.get_param("~gate_xy_tol", 1.2)
        self.gate_z_tol = rospy.get_param("~gate_z_tol", 0.6)
        self.z_err_slow = rospy.get_param("~z_err_slow", 1.0)

        self.segment_switch_t = rospy.get_param("~segment_switch_t", 0.85)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 1.5)
        self.accel = int(rospy.get_param("~accel", 8))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))

        self.pose = None
        self.yaw = 0.0
        self.seg = None
        self.z_offset = 0.0
        self.reached = False

        self.route = self.load_route(self.route_file, self.route_name)
        self.gates = self.load_gates(self.gates_file)
        self.gate_idx = 0
        self.prev_s = None
        rospy.loginfo("route '%s': %d pts, %.1f m; gates: %d",
                      self.route_name, len(self.route), self.route_length(),
                      len(self.gates))

        self.cmd_pub = rospy.Publisher(
            "/airsim_node/drone_1/vel_body_cmd", VelCmd, queue_size=1)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt",
                         PoseStamped, self.pose_cb)
        rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_loop)

    # ---------------- loaders ----------------
    @staticmethod
    def load_route(path, name):
        with open(path) as f:
            data = yaml.safe_load(f)
        routes = data["routes"]
        if name not in routes:
            raise KeyError("route '%s' not in %s" % (name, path))
        pts = [Point(float(p[0]), float(p[1]), float(p[2])) for p in routes[name]]
        if len(pts) < 2:
            raise ValueError("route needs >= 2 points")
        return pts

    @staticmethod
    def load_gates(path):
        if not path or not os.path.exists(path):
            return []
        with open(path) as f:
            data = yaml.safe_load(f)
        return data.get("gates", []) if data else []

    def route_length(self):
        total = 0.0
        for a, b in zip(self.route[:-1], self.route[1:]):
            total += math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2 + (b.z - a.z) ** 2)
        return total

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

    def init_segment(self, p):
        best, best_d, best_c = 0, float("inf"), None
        for i in range(len(self.route) - 1):
            _, c, _ = self.project_segment(i, p)
            d = math.hypot(c.x - p.x, c.y - p.y)
            if d < best_d:
                best, best_d, best_c = i, d, c
        self.seg = best
        self.z_offset = p.z - best_c.z
        rospy.loginfo("entry segment=%d d_cross=%.2f z_offset=%.2f",
                      best, best_d, self.z_offset)

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

    # ---------------- gate helper ----------------
    def gate_info(self, g, p):
        gz = g["z"] + (self.z_offset if self.gate_z_uses_offset else 0.0)
        dx, dy, dz = p.x - g["x"], p.y - g["y"], p.z - gz
        d_g = math.sqrt(dx * dx + dy * dy + dz * dz)
        s = dx * g["nx"] + dy * g["ny"] + dz * g.get("nz", 0.0)
        rx = dx - s * g["nx"]
        ry = dy - s * g["ny"]
        d_perp = math.hypot(rx, ry)
        return gz, d_g, s, d_perp, dz

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

        # ---- 路径 z ----
        if self.use_route_z:
            z_path = a.z + t * (b.z - a.z) + self.z_offset
        else:
            z_path = self.z_ref

        # ---- Gate 逻辑 ----
        state = "PATH"
        if self.gate_idx < len(self.gates):
            g = self.gates[self.gate_idx]
            gz, d_g, s, d_perp, dz = self.gate_info(g, p)
            alpha = (self.gate_blend_start - d_g) / max(
                1e-6, self.gate_blend_start - self.gate_blend_full)
            alpha = max(0.0, min(1.0, alpha))

            # 穿门判定: 由平面负侧到正侧, 且横向/高度满足容差
            if (self.prev_s is not None and self.prev_s < 0.0 <= s
                    and d_perp < self.gate_xy_tol and abs(dz) < self.gate_z_tol):
                rospy.loginfo("GATE %d PASSED (d_perp=%.2f dz=%.2f)",
                              g.get("id", self.gate_idx), d_perp, dz)
                self.gate_idx += 1
                self.prev_s = None
            else:
                self.prev_s = s

            if self.gate_idx < len(self.gates):
                g = self.gates[self.gate_idx]
                gz = g["z"] + (self.z_offset if self.gate_z_uses_offset else 0.0)
                # path z 与 gate z 融合
                z_ref = (1.0 - alpha) * z_path + alpha * gz
                # 靠近门: 对准门中心(未过平面) / 推向出口(已过平面)
                if d_g < self.gate_align_dist:
                    if s < -1.0:
                        tx, ty = g["x"], g["y"]
                    else:
                        d = g.get("approach_distance", 4.0)
                        tx, ty = g["x"] + d * g["nx"], g["y"] + d * g["ny"]
                    gsp = g.get("speed", self.gate_speed)
                    vgx, vgy = self.gate_k * (tx - p.x), self.gate_k * (ty - p.y)
                    sp = math.hypot(vgx, vgy)
                    if sp > gsp and sp > 1e-6:
                        vgx *= gsp / sp
                        vgy *= gsp / sp
                    vwx = (1.0 - alpha) * vwx + alpha * vgx
                    vwy = (1.0 - alpha) * vwy + alpha * vgy
                    state = "GATE"
            else:
                z_ref = z_path
        else:
            z_ref = z_path

        # ---- 高度误差 -> vz (机体 z 向上为正) ----
        # Z Safety Clamp: 任何来源的 z_ref 都限制在安全包络内
        z_ref = max(self.z_safe_min, min(self.z_safe_max, z_ref))
        z_err = p.z - z_ref
        vz = self.k_z * z_err
        vz = max(-self.vmax_z, min(self.vmax_z, vz))

        # 软边界: 接近上限禁止继续上升, 接近下限禁止继续下降
        # 正 vz = 上升(z 减小)
        if p.z <= self.z_safe_min + self.z_soft_margin:   # 接近合法最高点
            vz = min(vz, 0.0)                             # 禁止继续上升
        if p.z >= self.z_safe_max - self.z_soft_margin:   # 接近合法最低点
            vz = max(vz, 0.0)                             # 禁止继续下降

        # ---- 高度误差大时减速 ----
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
            1.0,
            "%s SEG %d/%d t=%.2f CROSS=%.2f GATE %d/%d zErr=%.2f "
            "VW(%.2f,%.2f,%.2f) dgoal=%.1f",
            state, self.seg, len(self.route) - 2, t, d_cross,
            self.gate_idx, len(self.gates), z_err, vwx, vwy, vz, d_goal)


if __name__ == "__main__":
    rospy.init_node("route_follower")
    RouteFollower()
    rospy.spin()
