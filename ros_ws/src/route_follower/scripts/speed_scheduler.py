#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Speed Scheduler v3: 动力学/预测限速 + 实测垂直能力 (方案 13~22, 33, 45 节)。

限制来源:
  v_curve   = sqrt(a_lat_max / (|κ|+ε))                横向加速度能力
  v_slope   = slope_eta * vz_available(vxy) / |dz/ds|  实测垂直能力 (非固定 3.0)
  v_tracking= 预测穿门误差 / Z 掉队 / Yaw 误差         软限制

hard_cap = min(cruise, v_curve, v_slope)   物理硬限制, 可低于 floor
soft_cap = max(normal_speed_floor, v_tracking)
v_target = min(hard_cap, soft_cap)

Z 误差只做连续降速 (方案 22), 不停车:
  <0.3 -> 1.0x, 0.3~0.6 -> 0.9x, 0.6~1.0 -> 0.75x, 1.0~1.5 -> 0.6x, >1.5 -> 0.5x
"""

import math


class SpeedScheduler(object):

    def __init__(self, cruise_speed=10.0, max_speed=12.0,
                 normal_speed_floor=4.0, a_lat_max=6.0,
                 slope_eta=0.8, vz_up_safe=4.0, a_up=4.0, a_down=5.0,
                 vz_capability=None,
                 z_slow1=0.3, z_slow2=0.6, z_slow3=1.0, z_slow4=1.5,
                 z_f2=0.9, z_f3=0.75, z_f4=0.6, z_f5=0.5,
                 gate_f1=0.8, gate_f2=0.6, gate_f3=0.45,
                 yaw_slow1=10.0, yaw_slow2=20.0, yaw_slow3=30.0,
                 yaw_f1=0.9, yaw_f2=0.8, yaw_f3=0.6):
        self.cruise = float(cruise_speed)
        self.max_speed = float(max_speed)
        self.floor = float(normal_speed_floor)
        self.a_lat_max = float(a_lat_max)
        self.slope_eta = float(slope_eta)
        self.vz_up_safe = float(vz_up_safe)
        self.a_up = float(a_up)
        self.a_down = float(a_down)
        self.vz_capability = vz_capability
        self.z_slow1 = float(z_slow1)
        self.z_slow2 = float(z_slow2)
        self.z_slow3 = float(z_slow3)
        self.z_slow4 = float(z_slow4)
        self.z_f2 = float(z_f2)
        self.z_f3 = float(z_f3)
        self.z_f4 = float(z_f4)
        self.z_f5 = float(z_f5)
        self.gate_f1 = float(gate_f1)
        self.gate_f2 = float(gate_f2)
        self.gate_f3 = float(gate_f3)
        self.yaw_slow1 = float(yaw_slow1)
        self.yaw_slow2 = float(yaw_slow2)
        self.yaw_slow3 = float(yaw_slow3)
        self.yaw_f1 = float(yaw_f1)
        self.yaw_f2 = float(yaw_f2)
        self.yaw_f3 = float(yaw_f3)

    def vz_available(self, vxy):
        if self.vz_capability is not None:
            return self.vz_capability.available(vxy)
        return self.vz_up_safe

    def curve_limit(self, kap):
        return math.sqrt(self.a_lat_max / (abs(kap) + 1e-3))

    def slope_limit(self, kz, trusted=True, vxy=0.0):
        vz_safe = self.vz_available(vxy)
        v = self.slope_eta * vz_safe / (abs(kz) + 1e-3)
        if not trusted:
            v = max(v, self.floor)
        return min(self.cruise, v)

    def tracking_limit(self, miss_ratio, z_err, z_worsening, pred_worse, yaw_err_deg):
        f = 1.0
        if miss_ratio is not None:
            if miss_ratio > 1.8:
                f = min(f, self.gate_f3)
            elif miss_ratio > 1.3:
                f = min(f, self.gate_f2)
            elif miss_ratio > 1.0:
                f = min(f, self.gate_f1)
        az = abs(z_err)
        if az >= self.z_slow4 and pred_worse:
            f = min(f, self.z_f5)
        elif az >= self.z_slow3 and (z_worsening or pred_worse):
            f = min(f, self.z_f4)
        elif az >= self.z_slow2 and z_worsening:
            f = min(f, self.z_f3)
        elif az >= self.z_slow1 and z_worsening:
            f = min(f, self.z_f2)
        ay = abs(yaw_err_deg)
        if ay > self.yaw_slow3:
            f = min(f, self.yaw_f3)
        elif ay > self.yaw_slow2:
            f = min(f, self.yaw_f2)
        elif ay > self.yaw_slow1:
            f = min(f, self.yaw_f1)
        return self.cruise * f

    def target(self, kap, kz, slope_trusted, miss_ratio, z_err, z_worsening,
               pred_worse, yaw_err_deg, vxy=0.0):
        v_curve = min(self.cruise, self.curve_limit(kap))
        v_slope = self.slope_limit(kz, slope_trusted, vxy)
        v_track = self.tracking_limit(miss_ratio, z_err, z_worsening,
                                      pred_worse, yaw_err_deg)
        hard = min(self.cruise, v_curve, v_slope)
        soft = max(self.floor, v_track)
        v = min(hard, soft)
        if hard >= soft:
            reason = "TRACKING"
        elif v_curve < self.cruise and v_curve <= v_slope:
            reason = "CURVE"
        elif v_slope < self.cruise:
            reason = "SLOPE"
        else:
            reason = "CRUISE"
        info = {"v_cruise": self.cruise, "v_curve": v_curve, "v_slope": v_slope,
                "v_tracking": v_track, "hard_cap": hard, "soft_cap": soft,
                "vz_available": self.vz_available(vxy), "reason": reason}
        return v, info

    def step(self, v_prev, v_target, dt):
        a = self.a_up if v_target >= v_prev else self.a_down
        dv = max(-a * dt, min(a * dt, v_target - v_prev))
        return max(0.0, min(self.max_speed, v_prev + dv))
