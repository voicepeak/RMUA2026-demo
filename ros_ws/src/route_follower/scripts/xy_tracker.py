#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""XY Tracker: 参考路径 look-ahead + Gate 门心平滑拉拽 + 门后 exit blend。

方案 17: 门心拉拽权重用 smoothstep, 近门不再突然加强; 刚过门的
gate_exit_blend_distance 米内, 从上一门的拉拽平滑衰减到当前轨迹切线,
避免 PASS 瞬间目标方向突变。
"""

import math

from altitude_profile import smoothstep


def _clamp01(x):
    return max(0.0, min(1.0, x))


class XYTracker(object):

    def __init__(self, lookahead_base=8.0, lookahead_kv=0.7, k_pursuit=1.2,
                 xy_converge=0.5, gate_blend_start=25.0, gate_blend_full=8.0,
                 exit_blend_distance=4.0):
        self.lookahead_base = float(lookahead_base)
        self.lookahead_kv = float(lookahead_kv)
        self.k_pursuit = float(k_pursuit)
        self.xy_converge = float(xy_converge)
        self.gate_blend_start = float(gate_blend_start)
        self.gate_blend_full = float(gate_blend_full)
        self.exit_blend_distance = float(exit_blend_distance)

    def lookahead(self, v):
        return self.lookahead_base + self.lookahead_kv * v

    def gate_pull_weight(self, d_gate):
        a = _clamp01((self.gate_blend_start - d_gate) /
                     max(1e-6, self.gate_blend_start - self.gate_blend_full))
        return self.xy_converge * smoothstep(a)

    def exit_weight(self, dist_after):
        if self.exit_blend_distance <= 0.0 or dist_after < 0.0 \
                or dist_after >= self.exit_blend_distance:
            return 0.0
        return self.xy_converge * (1.0 - smoothstep(dist_after / self.exit_blend_distance))

    def target(self, p_xy, s_now, v, next_gate, d_gate, last_gate, point_at):
        tx, ty, _ = point_at(s_now + self.lookahead(v))
        if last_gate is not None:
            w_out = self.exit_weight(s_now - last_gate["s"])
            if w_out > 0.0:
                tx = (1.0 - w_out) * tx + w_out * last_gate["x"]
                ty = (1.0 - w_out) * ty + w_out * last_gate["y"]
        if next_gate is not None:
            w = self.gate_pull_weight(d_gate)
            if w > 0.0:
                tx = (1.0 - w) * tx + w * next_gate["x"]
                ty = (1.0 - w) * ty + w * next_gate["y"]
        return tx, ty

    def velocity(self, p_xy, target_xy, v_max, arbiter=None):
        vx = self.k_pursuit * (target_xy[0] - p_xy[0])
        vy = self.k_pursuit * (target_xy[1] - p_xy[1])
        sp = math.hypot(vx, vy)
        if sp > v_max and sp > 1e-6:
            vx *= v_max / sp
            vy *= v_max / sp
        if arbiter is not None:
            return arbiter.arbitrate((vx, vy))
        return vx, vy
