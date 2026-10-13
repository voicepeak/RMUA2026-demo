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

    def __init__(self, recon_speed=2.0, recon_max_dist=30.0, map_brake_a=1.5,
                 gate_search_speed=10.0, gate_search_dist=250.0,wait_for_first_map=False):
        self.recon_speed = float(recon_speed)
        self.recon_max_dist = float(recon_max_dist)
        self.map_brake_a = float(map_brake_a)
        self.gate_search_speed=float(gate_search_speed)
        self.gate_search_dist=float(gate_search_dist)
        self.wait_for_first_map=bool(wait_for_first_map)
        self.first_map_acquired=False
        self.mode = TRACK
        self.recon_start_s = None
        self.recon_start_z = None
        self.hold_announced = False

    def reset(self):
        self.mode = TRACK
        self.recon_start_s = None
        self.recon_start_z = None
        self.hold_announced = False
        self.first_map_acquired=False

    def abort(self):
        self.mode = ABORT

    def update(self, use_gate_map, horizon_s, s_now, z_now,
               require_gate=False, gate_horizon=None,first_map_ready=False):
        """返回 (mode, v_map, transition); transition 为模式切换信息或 None。"""
        previous = self.mode
        if previous == ABORT:
            return self.mode, 0.0, None
        if use_gate_map and self.wait_for_first_map and not self.first_map_acquired:
            if not first_map_ready:
                self.mode=HOLD
                transition=({'from':previous,'to':HOLD,'s':s_now} if previous!=HOLD else None)
                return self.mode,0.,transition
            self.first_map_acquired=True
        v_map = None
        if not use_gate_map:
            self.mode = TRACK
            self.recon_start_s = None
            self.recon_start_z = None
        elif require_gate and (gate_horizon is None or gate_horizon<=s_now+2.):
            # A calibrated road height guides through the hub; it does not
            # certify an unseen opening. Preserve the spatial height profile
            # while searching along the known route at bounded speed/distance.
            if self.recon_start_s is None:
                self.recon_start_s=s_now;self.recon_start_z=z_now
            route_known=horizon_s is not None and horizon_s>s_now+2.
            distance=s_now-self.recon_start_s
            limit=self.gate_search_dist if route_known else self.recon_max_dist
            self.mode=RECON if distance<limit else HOLD
            v_map=(self.gate_search_speed if route_known else
                   min(self.recon_speed,math.sqrt(2.*self.map_brake_a*max(0.,limit-distance)))) if self.mode==RECON else 0.
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
                remaining=max(0.,self.recon_max_dist-(s_now-self.recon_start_s))
                v_map=min(self.recon_speed,math.sqrt(2.*self.map_brake_a*remaining))
            else:
                self.mode = HOLD
                v_map = 0.0
        transition = None
        if self.mode != previous:
            transition = {"from": previous, "to": self.mode, "s": s_now}
        return self.mode, v_map, transition
