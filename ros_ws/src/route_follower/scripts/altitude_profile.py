#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Altitude Profile: 由 Gate/Guide 锚点插值出的赛道高度几何 + 平滑切换。

方案职责划分 (文档第 12/33 节):
  - AltitudeProfile 只负责几何: z_center(s) / dz_ds(s) / corridor(s);
  - 不再做 "靠近下一门时再融合一次 gate.z" 的重复控制 (若门已是锚点,
    center(s) 已包含它);
  - z_ref 的变化率限制与反馈/前馈由 ZController 负责;
  - GateMap 在线刷新时用 ProfileBlender 在 profile_switch_time 内平滑过渡
    旧/新 profile, 避免 z_ref 与 dz/ds 同时跳变。
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


class ProfileBlender(object):
    """在线重建 profile 时, 在 switch_time 秒内做 smoothstep 混合 (方案 12.2)。"""

    def __init__(self, switch_time=0.4):
        self.switch_time = max(1e-3, float(switch_time))
        self.current = None
        self.previous = None
        self.switch_stamp = None

    def set_initial(self, profile, stamp=None):
        self.current = profile
        self.previous = None
        self.switch_stamp = None

    def switch(self, profile, stamp):
        if self.current is None:
            self.set_initial(profile, stamp)
            return
        self.previous = self.current
        self.current = profile
        self.switch_stamp = stamp

    def beta(self, stamp):
        if self.previous is None or self.switch_stamp is None or stamp is None:
            return 1.0
        return smoothstep(min(1.0, max(0.0, (stamp - self.switch_stamp) / self.switch_time)))

    def is_switching(self, stamp):
        return self.previous is not None and self.beta(stamp) < 1.0

    def center(self, s, stamp=None):
        z = self.current.center(s)
        b = self.beta(stamp)
        if b < 1.0:
            z = (1.0 - b) * self.previous.center(s) + b * z
        return z

    def dz_ds(self, s, h=4.0, stamp=None):
        z = self.current.dz_ds(s, h)
        b = self.beta(stamp)
        if b < 1.0:
            z = (1.0 - b) * self.previous.dz_ds(s, h) + b * z
        return z

    def corridor(self, s):
        return self.current.corridor(s)

    def anchor_pair(self, s):
        return self.current.anchor_pair(s)

    def horizon_s(self, s, s_now, horizon):
        return AltitudeProfile.horizon_s(s, s_now, horizon)
