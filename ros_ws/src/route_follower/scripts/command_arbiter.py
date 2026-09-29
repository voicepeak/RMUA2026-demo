#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Command Arbiter: XY 来源选择 + 最终命令组装 (方案 19 方案 B)。

职责:
  - arbitrate: XY 期望速度来源选择 (Emergency/Safety > Avoidance > Route);
  - finalize: 把 XY / Z / Yaw 三个控制量组装成唯一 FinalCommand,
    并交给 SafetySupervisor 做最后硬限幅后发布。
"""

import math

import numpy as np


class FinalCommand(object):

    __slots__ = ("vx", "vy", "vz", "yaw_rate")

    def __init__(self, vx, vy, vz, yaw_rate):
        self.vx = float(vx)
        self.vy = float(vy)
        self.vz = float(vz)
        self.yaw_rate = float(yaw_rate)

    def body(self, yaw):
        """World XY -> body XY (AirSim NED, yaw = atan2(vy, vx))。"""
        cy, sy = math.cos(yaw), math.sin(yaw)
        return FinalCommand(cy * self.vx + sy * self.vy,
                            -sy * self.vx + cy * self.vy,
                            self.vz, self.yaw_rate)


class CommandArbiter(object):

    def __init__(self, avoidance=None):
        self.avoidance = avoidance

    def arbitrate(self, v_route_xy):
        if self.avoidance is not None and self.avoidance.active:
            return np.asarray(self.avoidance.velocity_world[:2], dtype=float)
        return np.asarray(v_route_xy, dtype=float)

    def finalize(self, v_xy_world, vz_cmd, yaw_rate, yaw, safety=None, mode="TRACK"):
        command = FinalCommand(v_xy_world[0], v_xy_world[1], vz_cmd, yaw_rate)
        if safety is not None:
            vx, vy, vz, yaw_rate = safety.finalize(command.vx, command.vy,
                                                   command.vz, command.yaw_rate)
            command = FinalCommand(vx, vy, vz, yaw_rate)
        return command.body(yaw)
