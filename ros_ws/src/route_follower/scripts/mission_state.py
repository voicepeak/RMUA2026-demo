#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mission State: TRACK / RECON / HOLD / ABORT (方案 10, 38.2)。

RECON 是独立 mode, 不与正常 TRACK 叠加:
    TRACK  正常参考跟踪; 若视野耗尽 -> RECON
    RECON  明确切换: vxy = recon_speed, vz = recon_climb, 有限距离内重捕获
    HOLD   RECON 超出最大距离/时间, 悬停等待 (不再是 10 m/s 盲飞)
    ABORT  安全层触发 (卡死等), 速度全零

v_map (可见性硬上限) 也由本模块给出: 视野末端 remaining 米处按
sqrt(2*a*remaining) 保守限速。
"""

import math

TRACK = "TRACK"
RECON = "RECON"
HOLD = "HOLD"
ABORT = "ABORT"


class MissionState(object):

    def __init__(self, recon_speed=2.0, recon_max_dist=30.0, map_brake_a=1.5):
        self.recon_speed = float(recon_speed)
        self.recon_max_dist = float(recon_max_dist)
        self.map_brake_a = float(map_brake_a)
        self.mode = TRACK
        self.recon_start_s = None
        self.recon_start_z = None
        self.hold_announced = False

    def reset(self):
        self.mode = TRACK
        self.recon_start_s = None
        self.recon_start_z = None
        self.hold_announced = False

    def abort(self):
        self.mode = ABORT

    def update(self, use_gate_map, horizon_s, s_now, z_now):
        """返回 (mode, v_map, transition); transition 为模式切换信息或 None。"""
        previous = self.mode
        if previous == ABORT:
            return self.mode, 0.0, None
        v_map = None
        if not use_gate_map:
            self.mode = TRACK
            self.recon_start_s = None
            self.recon_start_z = None
        elif horizon_s is not None and horizon_s > s_now + 2.0:
            self.mode = TRACK
            self.recon_start_s = None
            self.recon_start_z = None
            remaining = max(0.0, horizon_s + 3.0 - s_now)
            v_map = math.sqrt(2.0 * self.map_brake_a * remaining)
        else:
            if self.recon_start_s is None:
                self.recon_start_s = s_now
                self.recon_start_z = z_now
            if s_now - self.recon_start_s < self.recon_max_dist:
                self.mode = RECON
                v_map = self.recon_speed
            else:
                self.mode = HOLD
                v_map = 0.0
        transition = None
        if self.mode != previous:
            transition = {"from": previous, "to": self.mode, "s": s_now}
        return self.mode, v_map, transition
