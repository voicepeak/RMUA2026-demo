#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Command Arbiter: 唯一决定最终 XY 期望速度的仲裁器。

优先级: Emergency/Safety > Obstacle Avoidance > Route。
当前 avoidance 未实现 (active=False), 故等价于直接采用 route 速度。
"""

import numpy as np


class CommandArbiter(object):

    def __init__(self, avoidance=None):
        self.avoidance = avoidance

    def arbitrate(self, v_route_xy):
        if self.avoidance is not None and self.avoidance.active:
            return np.asarray(self.avoidance.velocity_world[:2], dtype=float)
        return np.asarray(v_route_xy, dtype=float)
