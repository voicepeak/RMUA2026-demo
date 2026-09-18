#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Altitude Planner: 用 Gate 作为高度锚点, 在 XY 路径进度 s 上生成 z_ref(s)。

输入: path progress s, 当前高度, 下一道有效 Gate, 与下一道门的距离
输出: z_ref, altitude_mode, gate_blend_alpha

锚点层级:
  START(s=0) / GOAL(s=total) 为边界; 有效 GATE 为高度锚点。
  Gate 优先于普通插值; 无效(valid=false)或异常(z 跳变过大)的 Gate 不参与。
"""


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return 3.0 * t * t - 2.0 * t * t * t


class AltitudePlanner(object):

    def __init__(self, start_s, start_z, goal_s, goal_z, gates,
                 gate_blend_start=15.0, gate_blend_full=5.0,
                 z_rate_max=1.0, gate_z_max_jump=15.0):
        self.gate_blend_start = gate_blend_start
        self.gate_blend_full = gate_blend_full
        self.z_rate_max = z_rate_max
        self.gate_z_max_jump = gate_z_max_jump

        anchors = [(float(start_s), float(start_z))]
        for g in gates:
            if g.get("valid", False) and g.get("s") is not None:
                anchors.append((float(g["s"]), float(g["z"])))
        anchors.append((float(goal_s), float(goal_z)))
        anchors.sort(key=lambda a: a[0])
        self.anchors = anchors
        self.prev_z_ref = None

    def route_z(self, s):
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
                t = (s - s0) / (s1 - s0)
                return z0 + smoothstep(t) * (z1 - z0)
        return a[-1][1]

    def compute(self, s, current_z, next_gate, dist_to_gate, dt):
        z_route = self.route_z(s)
        alpha = 0.0
        mode = "ROUTE"
        if next_gate is not None and next_gate.get("valid", False):
            if dist_to_gate <= self.gate_blend_full:
                alpha = 1.0
            elif dist_to_gate < self.gate_blend_start:
                alpha = ((self.gate_blend_start - dist_to_gate) /
                         max(1e-6, self.gate_blend_start - self.gate_blend_full))
            alpha = max(0.0, min(1.0, alpha))
            z_gate = next_gate["z"]
            if abs(z_gate - current_z) > self.gate_z_max_jump:
                alpha = 0.0          # Gate Z 异常保护
            else:
                z_route = (1.0 - alpha) * z_route + alpha * z_gate
                if alpha > 0.0:
                    mode = "GATE"

        z_ref = z_route
        if self.prev_z_ref is not None and dt > 0.0:
            max_d = self.z_rate_max * dt
            if z_ref > self.prev_z_ref + max_d:
                z_ref = self.prev_z_ref + max_d
            elif z_ref < self.prev_z_ref - max_d:
                z_ref = self.prev_z_ref - max_d
        self.prev_z_ref = z_ref
        return z_ref, mode, alpha
