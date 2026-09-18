#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RMUA 2026 第二阶段: 赛道约束下的路径跟踪。

  Route Manager  : 加载 YAML 参考路径, 管理 current_segment
  Path Follower  : 最近点投影 C, 横向偏差 d_cross, Look-ahead 点 Q
  Boundary Guard : 按 d_cross 降低前进速度/增强回中
  Height Control : 按 z_ref 控制高度 (模拟器机体 z 向上为正)
  Velocity Ctrl  : 原 P 控制 + 限幅 + World->Body -> vel_body_cmd

坐标: AirSim NED; vel_body_cmd 的 vz 向上为正。route 的 z 通过进入时自动
标定的 z_offset 映射到 pose 坐标系。
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
        rospy.loginfo("route '%s': %d points, %.1f m",
                      self.route_name, len(self.route), self.route_length())

        self.cmd_pub = rospy.Publisher(
            "/airsim_node/drone_1/vel_body_cmd", VelCmd, queue_size=1)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt",
                         PoseStamped, self.pose_cb)
        rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_loop)

    # ---------------- Route Manager ----------------
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
        # 用进入时的实际高度标定 route z 与 pose 坐标系的偏置
        self.z_offset = p.z - best_c.z
        rospy.loginfo("entry segment = %d (d_cross=%.2f m) z_offset=%.2f",
                      best, best_d, self.z_offset)

    def lookahead_point(self, idx, t, look):
        # 沿参考路径从 C 前进 look 米, 可跨越多个线段
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

        # 前进速度
        vfx, vfy = self.forward_speed * ux, self.forward_speed * uy
        # 中心线纠偏
        vcx, vcy = self.k_cross * (c.x - p.x), self.k_cross * (c.y - p.y)

        # Boundary Guard
        if d_cross < self.soft_boundary:
            scale = 1.0
        elif d_cross < self.hard_boundary:
            scale, vcx, vcy = 0.5, vcx * 2, vcy * 2
        else:
            scale, vcx, vcy = 0.0, vcx * 3, vcy * 3
        vwx = scale * vfx + vcx
        vwy = scale * vfy + vcy

        # 高度: z_ref, 模拟器机体 vz 向上为正
        if self.use_route_z:
            z_ref = a.z + t * (b.z - a.z) + self.z_offset
        else:
            z_ref = self.z_ref
        z_err = p.z - z_ref
        vz = self.k_z * z_err
        vz = max(-self.vmax_z, min(self.vmax_z, vz))

        # 水平限速
        sp = math.hypot(vwx, vwy)
        if sp > self.vmax_xy and sp > 1e-6:
            vwx *= self.vmax_xy / sp
            vwy *= self.vmax_xy / sp

        # 终点
        goal = self.route[-1]
        d_goal = math.hypot(goal.x - p.x, goal.y - p.y)
        if self.seg >= len(self.route) - 2 and t > 0.95 and d_goal < self.goal_tolerance:
            self.publish_cmd(0.0, 0.0, 0.0)
            if not self.reached:
                self.reached = True
                rospy.loginfo("GOAL_REACHED")
            return
        self.reached = False

        # World -> Body
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        vx_b = cy * vwx + sy * vwy
        vy_b = -sy * vwx + cy * vwy
        self.publish_cmd(vx_b, vy_b, vz)

        rospy.loginfo_throttle(
            1.0,
            "SEG %d/%d t=%.2f CROSS=%.2f LOOK(%.1f,%.1f,%.1f) "
            "VW(%.2f,%.2f,%.2f) VB(%.2f,%.2f) zErr=%.2f dgoal=%.1f",
            self.seg, len(self.route) - 2, t, d_cross,
            q.x, q.y, q.z, vwx, vwy, vz, vx_b, vy_b, z_err, d_goal)


if __name__ == "__main__":
    rospy.init_node("route_follower")
    RouteFollower()
    rospy.spin()
