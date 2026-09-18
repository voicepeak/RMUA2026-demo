#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""避障接口预留 (本阶段不实现)。

未来: LiDAR/Camera 避障模块写入 AvoidanceCommand, CommandArbiter 在
avoidance.active=True 时优先采用其 velocity_world。
"""


class AvoidanceCommand(object):

    def __init__(self):
        self.active = False
        self.velocity_world = (0.0, 0.0, 0.0)
        self.confidence = 0.0
        self.timestamp = None
