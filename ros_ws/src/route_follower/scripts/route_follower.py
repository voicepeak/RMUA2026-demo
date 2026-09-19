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

import json
import math
import os
import sys
import threading

import numpy as np
import rospy
import tf.transformations as tft
import yaml
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String

from airsim_ros.msg import VelCmd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from altitude_profile import AltitudeProfile       # noqa: E402
from gate_chain import GateChain                  # noqa: E402
from command_arbiter import CommandArbiter        # noqa: E402
from avoidance_interface import AvoidanceCommand  # noqa: E402
from yaw_controller import YawController          # noqa: E402
from speed_scheduler import SpeedScheduler        # noqa: E402


class RouteFollower(object):

    def __init__(self):
        cfg = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "config"))
        self.route_file = rospy.get_param("~route_file", os.path.join(cfg, "route_1_3.yaml"))
        self.route_name = rospy.get_param("~route_name", "route_1_3")
        self.gates_file = rospy.get_param("~gates_file", os.path.join(cfg, "gates_vision_1_3.yaml"))
        self.guides_file = rospy.get_param("~guides_file", os.path.join(cfg, "none.yaml"))
        self.gate_z_uses_offset = bool(rospy.get_param("~gate_z_uses_offset", True))
        self.start_anchor_z = rospy.get_param("~start_anchor_z", -999.0)
        self.start_anchor_z = None if self.start_anchor_z == -999.0 else self.start_anchor_z

        # speed (v6: 预测/动力学限速)
        self.cruise_speed = rospy.get_param("~cruise_speed", 10.0)
        self.max_speed = rospy.get_param("~max_speed", 12.0)
        self.normal_speed_floor = rospy.get_param("~normal_speed_floor", 4.0)
        self.a_lat_max = rospy.get_param("~a_lat_max", 6.0)
        self.a_up = rospy.get_param("~a_up", 4.0)
        self.a_down = rospy.get_param("~a_down", 5.0)
        self.lookahead_base = rospy.get_param("~lookahead_base", 8.0)
        self.lookahead_kv = rospy.get_param("~lookahead_kv", 0.7)
        self.k_pursuit = rospy.get_param("~k_pursuit", 1.2)

        # z
        self.k_z = rospy.get_param("~k_z", 0.6)
        self.z_rate_max = rospy.get_param("~z_rate_max", 1.5)
        self.corridor_half = rospy.get_param("~corridor_half", 1.5)
        self.gate_blend_start = rospy.get_param("~gate_blend_start", 25.0)
        self.gate_blend_full = rospy.get_param("~gate_blend_full", 8.0)

        # gate chain / pass
        self.aperture_xy_tol = rospy.get_param("~aperture_xy_tol", 1.2)
        self.aperture_z_tol = rospy.get_param("~aperture_z_tol", 1.0)
        # 门洞半宽/半高: >0 用该值; =0 则用 Gate Map 的四角尺寸; 默认给足余量以吸收地图 z 偏差
        self.gate_pass_half_width = rospy.get_param("~gate_pass_half_width", 1.5)
        self.gate_pass_half_height = rospy.get_param("~gate_pass_half_height", 1.5)
        self.gate_miss_margin = rospy.get_param("~gate_miss_margin", 3.0)
        self.gate_skip_s = rospy.get_param("~gate_skip_s", 5.0)
        self.slope_factor = rospy.get_param("~slope_factor", 3.0)
        self.slope_abs_max = rospy.get_param("~slope_abs_max", 0.6)
        self.xy_converge = rospy.get_param("~xy_converge", 0.5)
        self.snap_gate_to_route = bool(rospy.get_param("~snap_gate_to_route", True))

        # v4: Z 前视/前馈 + 坡度限速 + 掉高保护 + 速度/Z 斜坡
        self.z_preview_time = rospy.get_param("~z_preview_time", 2.0)
        self.z_preview_min = rospy.get_param("~z_preview_min", 8.0)
        self.vz_up_safe = rospy.get_param("~vz_up_safe", 3.0)
        self.vz_up_limit = rospy.get_param("~vz_up_limit", 3.0)
        self.vz_down_limit = rospy.get_param("~vz_down_limit", 2.5)
        self.vz_accel_limit = rospy.get_param("~vz_accel_limit", 3.0)
        self.slope_eta = rospy.get_param("~slope_eta", 0.8)
        self.pred_time = rospy.get_param("~pred_time", 1.5)

        # v6: v_tracking 软限速阈值/系数 (方案 8, 29 节)
        self.z_slow1 = rospy.get_param("~z_slow1", 0.4)
        self.z_slow2 = rospy.get_param("~z_slow2", 0.8)
        self.z_slow3 = rospy.get_param("~z_slow3", 1.2)
        self.z_f2 = rospy.get_param("~z_f2", 0.8)
        self.z_f3 = rospy.get_param("~z_f3", 0.6)
        self.z_f4 = rospy.get_param("~z_f4", 0.4)
        self.gate_f1 = rospy.get_param("~gate_f1", 0.8)
        self.gate_f2 = rospy.get_param("~gate_f2", 0.6)
        self.gate_f3 = rospy.get_param("~gate_f3", 0.45)
        self.yaw_slow1 = rospy.get_param("~yaw_slow1", 10.0)
        self.yaw_slow2 = rospy.get_param("~yaw_slow2", 20.0)
        self.yaw_slow3 = rospy.get_param("~yaw_slow3", 30.0)
        self.yaw_f1 = rospy.get_param("~yaw_f1", 0.9)
        self.yaw_f2 = rospy.get_param("~yaw_f2", 0.8)
        self.yaw_f3 = rospy.get_param("~yaw_f3", 0.6)

        # v6: Yaw Path Following (方案 22~29, 34 节)
        self.yaw_control = bool(rospy.get_param("~yaw_control", True))
        self.yaw_lookahead = rospy.get_param("~yaw_lookahead", 25.0)
        self.k_yaw = rospy.get_param("~k_yaw", 1.2)
        self.yaw_rate_max = rospy.get_param("~yaw_rate_max", 1.0)
        self.yaw_accel_limit = rospy.get_param("~yaw_accel_limit", 2.0)
        self.k_vision = rospy.get_param("~k_vision", 0.15)      # FOV correction < K_path
        self.fov_n_gates = int(rospy.get_param("~fov_n_gates", 3))

        # v5: Z 外推 Horizon (方案 16~17 节)
        self.z_horizon_time = rospy.get_param("~z_horizon_time", 2.0)
        self.z_horizon_min = rospy.get_param("~z_horizon_min", 10.0)
        self.z_horizon_max = rospy.get_param("~z_horizon_max", 20.0)

        # v5: 订阅持久 Gate Map (方案 7, 16 节)
        self.use_gate_map = bool(rospy.get_param("~use_gate_map", False))
        self.gate_map_topic = rospy.get_param("~gate_map_topic", "/rmua/gate_map")
        self.gate_map_min_support = int(rospy.get_param("~gate_map_min_support", 2))
        self.sigma_z_max = rospy.get_param("~sigma_z_max", 0.6)

        self.v_cmd_prev = 0.0
        self.vz_prev = 0.0
        self.z_prev = None
        self.start_gate = int(rospy.get_param("~start_gate", 0))
        self.end_gate = int(rospy.get_param("~end_gate", -1))

        self.accel = int(rospy.get_param("~accel", 8))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.dt = 1.0 / self.control_rate

        self.pose = None
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.seg = None
        self.z_offset = 0.0
        self.reached = False
        self.prev_s_plane = None
        self.prev_s = None
        self.prev_pose = None
        self.chain = None
        self.profile = None
        self._map_sig = None
        self.avoidance = AvoidanceCommand()
        self.arbiter = CommandArbiter(self.avoidance)
        self.yaw_ctrl = YawController(lookahead=self.yaw_lookahead, k=self.k_yaw,
                                      rate_max=self.yaw_rate_max)
        self.speed_sched = SpeedScheduler(
            cruise_speed=self.cruise_speed, max_speed=self.max_speed,
            normal_speed_floor=self.normal_speed_floor, a_lat_max=self.a_lat_max,
            slope_eta=self.slope_eta, vz_up_safe=self.vz_up_safe,
            a_up=self.a_up, a_down=self.a_down,
            z_slow1=self.z_slow1, z_slow2=self.z_slow2, z_slow3=self.z_slow3,
            z_f2=self.z_f2, z_f3=self.z_f3, z_f4=self.z_f4,
            gate_f1=self.gate_f1, gate_f2=self.gate_f2, gate_f3=self.gate_f3,
            yaw_slow1=self.yaw_slow1, yaw_slow2=self.yaw_slow2, yaw_slow3=self.yaw_slow3,
            yaw_f1=self.yaw_f1, yaw_f2=self.yaw_f2, yaw_f3=self.yaw_f3)
        self._lock = threading.RLock()
        self.start_z0 = None
        self.no_future_gate = True
        self.yaw_rate_prev = 0.0
        self.z_err_prev = None
        self.speed_info = {}
        self.v_target = 0.0

        self.route = self.load_route(self.route_file, self.route_name)
        self.gates = self.load_list(self.gates_file, "gates")
        self.guides = self.load_list(self.guides_file, "altitude_guides")
        self.build_route()

        rospy.loginfo("route '%s': %d pts, %.0f m; gates=%d", self.route_name, len(self.route),
                      self.seg_s[-1], len(self.gates))
        self.cmd_pub = rospy.Publisher("/airsim_node/drone_1/vel_body_cmd", VelCmd, queue_size=1)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.pose_cb)
        if self.use_gate_map:
            rospy.Subscriber(self.gate_map_topic, String, self.gate_map_cb)
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

    # ---------- init / rebuild ----------
    def _build(self, gates_raw, s_now, p0z):
        """由原始 gate 列表重建 chain 与 altitude profile。"""
        off = self.z_offset if self.gate_z_uses_offset else 0.0
        gates_adj = []
        for g in gates_raw:
            gg = dict(g)
            gg["z"] = gg["z"] + off
            gg["s"] = self.project_gate_s(gg)
            if self.snap_gate_to_route:      # 门在道路中心: XY 吸附到路线中心线
                tx_, ty_, _ = self.point_at(gg["s"])
                gg["x"], gg["y"] = tx_, ty_
            gates_adj.append(gg)
        chain = GateChain(gates_adj, slope_factor=self.slope_factor,
                          slope_abs_max=self.slope_abs_max)
        for g in chain.gates:                    # 方案 22: sigma_z 过大 -> soft/suspect
            if abs(g.get("sigma_z", 0.0)) > self.sigma_z_max:
                g["z_suspect"] = True
        start_z = self.start_anchor_z if self.start_anchor_z is not None else p0z
        prof_gates = [dict(g, valid=(not g["z_suspect"])) for g in chain.gates]
        valid_prof = [g for g in prof_gates if g["valid"]]
        goal_z = valid_prof[-1]["z"] if valid_prof else start_z
        guides_adj = [{"s": gd["s"], "z": gd["z"] + off}
                      for gd in self.guides if gd.get("s") is not None]
        profile = AltitudeProfile(
            0.0, start_z, self.seg_s[-1], goal_z,
            prof_gates, guides_adj,
            corridor_half=self.corridor_half, gate_blend_start=self.gate_blend_start,
            gate_blend_full=self.gate_blend_full, z_rate_max=self.z_rate_max)
        idx = self.start_gate
        while idx < len(chain.gates) and chain.gates[idx]["s"] <= s_now - 2.0:
            idx += 1
        return chain, profile, idx

    def init_once(self, p):
        i, t, d, c = self.project(p)
        self.seg = i
        self.z_offset = p.z - c.z
        self.start_z0 = p.z
        s_now = self.seg_s[i] + t * self.seg_len[i]
        with self._lock:
            self.chain, self.profile, self.gate_idx = self._build(self.gates, s_now, p.z)
            self.suspect = self.chain.suspect_ids()
        rospy.loginfo("init seg=%d z_offset=%.2f gates=%d suspects=%s start_gate=%d",
                      i, self.z_offset, len(self.chain.gates), self.suspect, self.gate_idx)

    def gate_map_cb(self, msg):
        """持久 Gate Map 更新 (方案 7 节: Perception/Planning 解耦)。"""
        try:
            gates = json.loads(msg.data)
        except ValueError:
            return
        gates = [g for g in gates if g.get("valid", True)
                 and g.get("support", 0) >= self.gate_map_min_support]
        if not gates:
            return
        sig = tuple((g.get("id"), round(g["x"], 2), round(g["y"], 2), round(g["z"], 2))
                    for g in gates)
        if sig == self._map_sig:
            return
        with self._lock:
            if self.pose is None:
                return
            p = self.pose.position
            i, t, d, c = self.project(p)
            s_now = self.seg_s[i] + t * self.seg_len[i]
            p0z = self.start_z0 if self.start_z0 is not None else p.z
            chain, profile, idx = self._build(gates, s_now, p0z)
            if self.profile is not None:            # 保持 Z 速率限幅连续
                profile.prev_z_ref = self.profile.prev_z_ref
            self.chain, self.profile, self.gate_idx = chain, profile, idx
            self._map_sig = sig
        rospy.loginfo_throttle(2.0, "GATE_MAP update: %d gates, next_idx=%d",
                               len(gates), self.gate_idx)

    # ---------- callbacks ----------
    def pose_cb(self, m):
        self.pose = m.pose
        q = m.pose.orientation
        self.roll, self.pitch, self.yaw = tft.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])

    def publish(self, vx, vy, vz, yaw_rate=0.0):
        c = VelCmd()
        c.header.stamp = rospy.Time.now()
        c.header.frame_id = "drone_1"
        c.vx, c.vy, c.vz = vx, vy, vz
        # 模拟器 VelCmd.yawRate 单位是 度/秒 (官方示例注释 "yaw, deg");
        # 控制器内部用 rad/s, 这里统一换算后下发。
        c.yawRate = math.degrees(yaw_rate)
        c.va = self.accel
        c.stop = 0
        self.cmd_pub.publish(c)

    def _gate_metrics(self, ng, pn):
        """门平面几何: (d_g, lat, vert, e_n); ng=None 时返回 d_g=1e9。"""
        if ng is None:
            return 1e9, 0.0, 0.0, 0.0
        G = np.array([ng["x"], ng["y"], ng["z"]])
        n = np.array([ng.get("nx", 1.0), ng.get("ny", 0.0), ng.get("nz", 0.0)])
        n = n / (np.linalg.norm(n) + 1e-9)
        r = pn - G
        e_n = float(r @ n)
        vert = float(r[2] - e_n * n[2])
        lat = float(math.hypot(r[0] - e_n * n[0], r[1] - e_n * n[1]))
        return float(np.linalg.norm(r)), lat, vert, e_n

    def _gate_half_extents(self, ng):
        """门洞半宽/半高: 优先用 Gate Map 的四角尺寸, 否则退回容差参数。"""
        hw = self.gate_pass_half_width
        hh = self.gate_pass_half_height
        if hw <= 0.0:
            w = ng.get("width")
            hw = 0.5 * float(w) if w else self.aperture_xy_tol
        if hh <= 0.0:
            h = ng.get("height")
            hh = 0.5 * float(h) if h else self.aperture_z_tol
        return hw, hh

    def _predict_gate_miss(self, ng, pn):
        """用实际世界速度预测到门平面时的穿门偏差比 max(lat/hw, |vert|/hh)。
        返回 None 表示不接近/无有效预测 (不因门限速)。(方案 3~5)"""
        if ng is None or self.prev_pose is None or self.dt <= 0:
            return None
        v_world = (pn - np.asarray(self.prev_pose, dtype=float)) / self.dt
        G = np.array([ng["x"], ng["y"], ng["z"]])
        n = np.array([ng.get("nx", 1.0), ng.get("ny", 0.0), ng.get("nz", 0.0)])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            return None
        n = n / nn
        e_n = float((pn - G) @ n)
        closing = float(v_world @ n)
        if closing <= 0.2:
            return None
        tt = -e_n / closing
        if tt < 0.0 or tt > 2.0 * self.pred_time:
            return None
        pp = pn + v_world * tt
        r = pp - G
        rn = float(r @ n)
        vert_p = float(r[2] - rn * n[2])
        lat_p = float(math.hypot(r[0] - rn * n[0], r[1] - rn * n[1]))
        hw, hh = self._gate_half_extents(ng)
        return max(lat_p / max(hw, 1e-3), abs(vert_p) / max(hh, 1e-3))

    def _clamp_corr_gate(self, profile, z, s, ng, alpha):
        """走廊钳位; 向门融合时把门锚点纳入走廊, 避免把 z_ref 钳离门 (方案 35 节)。"""
        ceil, floor = profile.corridor(s)
        if alpha > 0.0 and ng is not None:
            c = profile.center(s)
            zg = ng["z"]
            ceil = min(ceil, min(c, zg) - self.corridor_half)
            floor = max(floor, max(c, zg) + self.corridor_half)
        return max(ceil, min(floor, z))

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

        chain, profile = self.chain, self.profile

        ng = chain.gates[self.gate_idx] if self.gate_idx < len(chain.gates) else None
        d_g, lat, vert, e_n = self._gate_metrics(ng, pn)

        # 过门判定: route 进度 s 跨越门 s 时, 插值出穿越点并检查是否在门洞内。
        # (比平面法向符号+固定容差稳, 且用 Gate Map 的真实门尺寸判断)
        while (ng is not None and self.prev_s is not None and self.prev_pose is not None
               and self.prev_s < ng["s"] <= s_now):
            a = (ng["s"] - self.prev_s) / max(1e-6, s_now - self.prev_s)
            pv = np.asarray(self.prev_pose, dtype=float)
            pc = pv + a * (pn - pv)
            rx, ry, _ = self.point_at(ng["s"])
            lat_c = float(math.hypot(pc[0] - rx, pc[1] - ry))
            vert_c = float(pc[2] - ng["z"])
            hw, hh = self._gate_half_extents(ng)
            ok = (lat_c < hw and abs(vert_c) < hh)
            rospy.loginfo("GATE %s %s (lat=%.2f/%.2f vert=%.2f/%.2f)",
                          ng.get("id", self.gate_idx), "PASSED" if ok else "MISSED",
                          lat_c, hw, vert_c, hh)
            self.gate_idx += 1
            self.prev_s_plane = None
            ng = chain.gates[self.gate_idx] if self.gate_idx < len(chain.gates) else None
            d_g, lat, vert, e_n = self._gate_metrics(ng, pn)

        # 兜底: 门已远在身后 (初始化/瞬移/丢帧) -> 直接推进, 避免卡死
        while (ng is not None and self.gate_idx < len(chain.gates)
               and chain.gates[self.gate_idx]["s"] < s_now - self.gate_miss_margin):
            behind = (s_now - chain.gates[self.gate_idx]["s"] >= self.gate_skip_s)
            rospy.logwarn("%s gate %s (s=%.1f, s_now=%.1f)",
                          "SKIP" if behind else "MISSED",
                          chain.gates[self.gate_idx].get("id"),
                          chain.gates[self.gate_idx]["s"], s_now)
            self.gate_idx += 1
            self.prev_s_plane = None
            ng = chain.gates[self.gate_idx] if self.gate_idx < len(chain.gates) else None
            d_g, lat, vert, e_n = self._gate_metrics(ng, pn)

        # ---- 未来门判定 + NO_FUTURE_GATE 保护 (方案 16 节) ----
        v_s = max(self.v_cmd_prev, 0.5)
        future_gates = [g for g in chain.gates[self.gate_idx:] if g["s"] > s_now + 1.0]
        self.no_future_gate = (ng is None) or (len(future_gates) == 0)

        # ---- Z 前视 Horizon: 禁止无限 slope 外推 (方案 17 节) ----
        Lz = max(self.z_preview_min, self.z_preview_time * v_s)
        z_horizon = max(self.z_horizon_min,
                        min(self.z_horizon_max, self.z_horizon_time * v_s))
        s_ff = profile.horizon_s(s_now + Lz, s_now, z_horizon)
        kz_prev = 0.0 if self.no_future_gate else profile.dz_ds(s_ff)

        # ---- Yaw Path Following v6: 提前看向未来赛道 + 斜坡 (方案 17~29) ----
        yaw_calc = 0.0
        yaw_target = self.yaw
        yaw_err = 0.0
        e_img = 0.0
        yaw_source = "HOLD"
        if self.yaw_control:
            if future_gates:
                yaw_source = "GATE_CHAIN"
                tgt = self.yaw_ctrl.target_from_gates(
                    (p.x, p.y), [(g["x"], g["y"]) for g in future_gates])
            else:
                yaw_source = "ROUTE"
                rx, ry, _ = self.point_at(s_now + self.yaw_lookahead)
                tgt = self.yaw_ctrl.target_from_route((p.x, p.y), (rx, ry))
            yaw_calc, yaw_target, yaw_err = self.yaw_ctrl.rate(
                (p.x, p.y), self.yaw, tgt)
            if self.k_vision != 0.0 and future_gates:
                R_wb = tft.euler_matrix(self.roll, self.pitch, self.yaw)[:3, :3]
                gw = [(g["x"], g["y"], g["z"]) for g in future_gates[:self.fov_n_gates]]
                e_img = self.yaw_ctrl.vision_fov_error(
                    gw, np.array([p.x, p.y, p.z]), R_wb, self.fov_n_gates)
        yaw_rate_target = max(-self.yaw_rate_max,
                              min(self.yaw_rate_max, yaw_calc + self.k_vision * e_img))
        dyr = self.yaw_accel_limit * self.dt           # Yaw 斜坡 (方案 25)
        yaw_rate = max(self.yaw_rate_prev - dyr,
                       min(self.yaw_rate_prev + dyr, yaw_rate_target))
        self.yaw_rate_prev = yaw_rate

        # ---- 速度调度 v6: 曲线/坡度物理限速 + 预测 tracking 软限速 (方案 1~16) ----
        kap = self.curvature(s_now)
        e_z0 = p.z - profile.center(s_now)
        z_dot = 0.0
        if self.z_prev is not None and self.dt > 0:
            z_dot = (p.z - self.z_prev) / self.dt
        self.z_prev = p.z
        s_pred = profile.horizon_s(s_now + max(2.0, v_s * self.pred_time),
                                   s_now, z_horizon)
        e_pred = (p.z + z_dot * self.pred_time) - profile.center(s_pred)
        z_worsening = (self.z_err_prev is not None
                       and abs(e_z0) > abs(self.z_err_prev) + 1e-4)
        pred_worse = abs(e_pred) > abs(e_z0) + 1e-3
        self.z_err_prev = e_z0
        miss_ratio = self._predict_gate_miss(ng, pn)
        slope_trusted = (ng is None or
                         (not ng.get("z_suspect", False)
                          and ng.get("hard_anchor", True)))
        v_target, self.speed_info = self.speed_sched.target(
            kap, kz_prev, slope_trusted, miss_ratio, e_z0,
            z_worsening, pred_worse, math.degrees(yaw_err))
        v = self.speed_sched.step(self.v_cmd_prev, v_target, self.dt)
        self.v_cmd_prev = v
        self.v_target = v_target

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
        v_route = self.arbiter.arbitrate((vx, vy))

        # ---- Z: 反馈 + 前馈 (vz_ff = -dz/ds_preview * v) ----
        z_gate = {"z": float(ng["z"]), "valid": True} if ng is not None else None
        z_ref, z_mode, alpha = profile.compute(s_now, p.z, z_gate, d_g, self.dt)
        z_ref = self._clamp_corr_gate(profile, z_ref, s_now, ng, alpha)
        z_err = p.z - z_ref
        vz_fb = self.k_z * z_err
        vz_ff = -kz_prev * v
        vz_target = max(-self.vz_down_limit, min(self.vz_up_limit, vz_fb + vz_ff))
        dvz = max(-self.vz_accel_limit * self.dt,
                  min(self.vz_accel_limit * self.dt, vz_target - self.vz_prev))
        vz = self.vz_prev + dvz
        self.vz_prev = vz

        # 水平限速
        sp = math.hypot(*v_route)
        if sp > v and sp > 1e-6:
            v_route = (v_route[0] * v / sp, v_route[1] * v / sp)

        self.prev_pose = (p.x, p.y, p.z)
        self.prev_s = s_now

        # ---- 到达 ----
        if self.end_gate >= 0 and self.gate_idx > self.end_gate:
            self.publish(0.0, 0.0, 0.0, 0.0)
            self.reached = True
            return
        if self.gate_idx >= len(chain.gates) and s_now > self.seg_s[-1] - 5.0:
            self.publish(0.0, 0.0, 0.0, 0.0)
            if not self.reached:
                rospy.loginfo("GOAL_REACHED")
            self.reached = True
            return

        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        vx_b = cy * v_route[0] + sy * v_route[1]
        vy_b = -sy * v_route[0] + cy * v_route[1]
        self.publish(vx_b, vy_b, vz, yaw_rate)

        si = self.speed_info
        rospy.loginfo_throttle(
            2.0,
            "PATH s=%.0f v=%.2f/%.2f CAP=%s(vC=%.1f vS=%.1f vT=%.1f) noFG=%s idx=%d "
            "dG=%.1f lat=%.2f vert=%.2f miss=%s | yaw_src=%s yaw=%.1f yawT=%.1f "
            "yawErr=%.1f yawCalc=%.1f yawSentDeg=%.1f eImg=%.3f | zRef=%.2f zErr=%.2f "
            "ePred=%.2f zWorse=%s kz=%.3f vz=%.2f",
            s_now, v, v_target, si.get("reason", ""),
            si.get("v_curve", 0.0), si.get("v_slope", 0.0), si.get("v_tracking", 0.0),
            self.no_future_gate, self.gate_idx, d_g, lat, vert,
            ("%.2f" % miss_ratio) if miss_ratio is not None else "-",
            yaw_source, math.degrees(self.yaw), math.degrees(yaw_target),
            math.degrees(yaw_err), math.degrees(yaw_calc), math.degrees(yaw_rate), e_img,
            z_ref, z_err, e_pred, z_worsening, kz_prev, vz)


if __name__ == "__main__":
    rospy.init_node("route_follower")
    RouteFollower()
    rospy.spin()
