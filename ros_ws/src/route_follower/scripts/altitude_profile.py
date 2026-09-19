#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Altitude Profile + 动态 Z 走廊。

- z_center(s): 由 Gate 锚点/Altitude Guide 用 Smoothstep 插值得到赛道中心高度
- z_ref: 靠近 Gate 时向 gate.z 融合, 并做变化率限制与异常保护
- corridor(s): 沿赛道的局部安全高度包络 [z_ceiling, z_floor] (NED, 上界更负)

取代全局固定的 z_safe_min/z_safe_max。
"""


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return 3.0 * t * t - 2.0 * t * t * t


class AltitudeProfile(object):

    def __init__(self, start_s, start_z, goal_s, goal_z, gates, guides,
                 corridor_half=1.5, gate_blend_start=20.0, gate_blend_full=6.0,
                 z_rate_max=1.0, gate_z_max_jump=15.0):
        self.corridor_half = corridor_half
        self.gate_blend_start = gate_blend_start
        self.gate_blend_full = gate_blend_full
        self.z_rate_max = z_rate_max
        self.gate_z_max_jump = gate_z_max_jump

        gate_anchors = [(float(g["s"]), float(g["z"]))
                        for g in gates if g.get("valid", False) and g.get("s") is not None]
        anchors = [(float(start_s), float(start_z))] + gate_anchors
        for gd in (guides or []):
            if gd.get("s") is None:
                continue
            gs, gz = float(gd["s"]), float(gd["z"])
            if any(abs(gs - s) < 1.0 for s, _ in gate_anchors):
                continue
            anchors.append((gs, gz))
        anchors.append((float(goal_s), float(goal_z)))
        anchors.sort(key=lambda a: a[0])
        self.anchors = anchors
        self.prev_z_ref = None
        self.rate_limited = False       # 上一帧 z_ref 是否被 z_rate_max 限住 (调试用)

    def center(self, s):
        a = self.anchors
        if s <= a[0][0]:
            return a[0][1]
        if s >= a[-1][0]:
            return a[-1][1]
        for i in range(len(a) - 1):
            s0, z0 = a[i]
            s1, z1 = a[i + 1]
            if s0 <= s <= s1:
                if s1 - s0 < 1e-6:
                    return z1
                return z0 + smoothstep((s - s0) / (s1 - s0)) * (z1 - z0)
        return a[-1][1]

    def corridor(self, s):
        c = self.center(s)
        return c - self.corridor_half, c + self.corridor_half   # (ceiling, floor)

    def anchor_pair(self, s):
        """返回包住 s 的两个锚点 (s0,z0),(s1,z1)。"""
        a = self.anchors
        for i in range(len(a) - 1):
            if a[i][0] <= s <= a[i + 1][0]:
                return a[i], a[i + 1]
        return a[-1], a[-1]

    @staticmethod
    def horizon_s(s, s_now, horizon):
        """把查询点限制在 [s_now, s_now+horizon] 内 (方案 17 节: Z 外推有 Horizon)。"""
        if horizon is None or horizon <= 0.0:
            return s
        return max(s_now, min(s, s_now + horizon))

    def dz_ds(self, s, h=4.0):
        s0 = max(self.anchors[0][0], s - h)
        s1 = min(self.anchors[-1][0], s + h)
        if s1 - s0 < 1e-6:
            return 0.0
        return (self.center(s1) - self.center(s0)) / (s1 - s0)

    def compute(self, s, current_z, next_gate, dist_to_gate, dt):
        z = self.center(s)
        alpha = 0.0
        mode = "ROUTE"
        if next_gate is not None and next_gate.get("valid", False):
            d = dist_to_gate
            if d <= self.gate_blend_full:
                alpha = 1.0
            elif d < self.gate_blend_start:
                alpha = ((self.gate_blend_start - d) /
                         max(1e-6, self.gate_blend_start - self.gate_blend_full))
            alpha = max(0.0, min(1.0, alpha))
            zg = next_gate["z"]
            if abs(zg - current_z) > self.gate_z_max_jump:
                alpha = 0.0
            else:
                z = (1.0 - alpha) * z + alpha * zg
                if alpha > 0.0:
                    mode = "GATE"
        self.rate_limited = False
        if self.prev_z_ref is not None and dt > 0.0:
            md = self.z_rate_max * dt
            if z > self.prev_z_ref + md:
                z = self.prev_z_ref + md
                self.rate_limited = True
            elif z < self.prev_z_ref - md:
                z = self.prev_z_ref - md
                self.rate_limited = True
        self.prev_z_ref = z
        return z, mode, alpha

    def clamp_corridor(self, z, s):
        ceil, floor = self.corridor(s)
        return max(ceil, min(floor, z))
