#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Command Arbiter: 唯一决定最终期望速度的仲裁器。

优先级: Emergency/Safety > Obstacle Avoidance > Gate > Route。
当前 avoidance 未实现 (active=False), Arbiter 等价于 Route/Gate 切换。
"""

import numpy as np

ALIGN, CROSS, EXIT, RECOVER, CAPTURE = "ALIGN", "CROSS", "EXIT", "RECOVER", "CAPTURE"


class CommandArbiter(object):

    def __init__(self, avoidance=None):
        self.avoidance = avoidance

    def arbitrate(self, v_route_xy, v_gate_xy, gate_state, alpha):
        v_route_xy = np.asarray(v_route_xy, dtype=float)
        v_gate_xy = np.asarray(v_gate_xy, dtype=float)
        # 1) 避障 (预留)
        if self.avoidance is not None and self.avoidance.active:
            return np.asarray(self.avoidance.velocity_world[:2], float), "AVOID"
        # 2) Gate 完全接管
        if gate_state in (ALIGN, CROSS, EXIT, RECOVER):
            return v_gate_xy, "GATE"
        # 3) Gate 捕获融合
        if gate_state == CAPTURE:
            return (1.0 - alpha) * v_route_xy + alpha * v_gate_xy, "BLEND"
        # 4) 普通路径
        return v_route_xy, "ROUTE"
