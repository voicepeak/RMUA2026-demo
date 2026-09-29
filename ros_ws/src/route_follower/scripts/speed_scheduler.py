#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Speed Scheduler: 唯一水平速度 authority (方案 5, 7)。

限制来源全部统一为 cap, 取最严格值:
  v_cruise    巡航速度
  v_curve     min(局部曲率, 前方 curve braking envelope)
  v_climb     min(局部坡度, 前方 climb feasibility preview)
  v_map       在线地图可见性/搜扫上限 (RECON/MAP_HORIZON)
  v_tracking  tracking 软限制 (穿门预测 / Z 误差 / Yaw 误差)

  hard_cap = min(v_cruise, v_curve, v_climb, v_map)
  soft_cap = max(normal_speed_floor, v_tracking)
  v_target = min(hard_cap, soft_cap)

物理硬约束优先于 normal_speed_floor (方案 5.2): 陡坡真的需要 2.5 m/s 时,
即使 floor=4 也必须允许 2.5。

step() 支持 hard_cap 即时 clamp (方案 7): 平滑斜坡不能拖延物理硬限速,
最终命令 v_cmd = min(v_smooth, hard_cap)。
"""

import math

INF = float("inf")


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
            return self.vz_capability.up(vxy)
        return self.vz_up_safe

    def curve_limit(self, kap):
        return math.sqrt(self.a_lat_max / (abs(kap) + 1e-3))

    def slope_limit(self, kz, trusted=True, vxy=0.0):
        vz_safe = self.slope_eta * self.vz_available(vxy)
        v = vz_safe / (abs(kz) + 1e-3)
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
               pred_worse, yaw_err_deg, vxy=0.0,
               v_curve_preview=None, v_climb_preview=None, v_map=None):
        v_curve = min(self.cruise, self.curve_limit(kap))
        if v_curve_preview is not None:
            v_curve = min(v_curve, float(v_curve_preview))
        v_slope = self.slope_limit(kz, slope_trusted, vxy)
        v_climb = v_slope
        if v_climb_preview is not None:
            v_climb = min(v_climb, float(v_climb_preview))
        v_track = self.tracking_limit(miss_ratio, z_err, z_worsening,
                                      pred_worse, yaw_err_deg)
        hard = min(self.cruise, v_curve, v_climb)
        if v_map is not None:
            hard = min(hard, float(v_map))
        soft = max(self.floor, v_track)
        v = min(hard, soft)
        if hard >= soft:
            reason = "TRACKING" if v_track < self.cruise else "CRUISE"
        else:
            candidates = [("CURVE", v_curve), ("SLOPE", v_slope), ("CLIMB", v_climb)]
            if v_map is not None:
                candidates.append(("MAP", float(v_map)))
            reason = min(candidates, key=lambda kv: kv[1])[0]
        info = {"v_cruise": self.cruise, "v_curve": v_curve,
                "v_curve_preview": (float(v_curve_preview)
                                    if v_curve_preview is not None else None),
                "v_slope": v_slope, "v_climb": v_climb,
                "v_climb_preview": (float(v_climb_preview)
                                    if v_climb_preview is not None else None),
                "v_map": (float(v_map) if v_map is not None else None),
                "v_tracking": v_track, "hard_cap": hard, "soft_cap": soft,
                "vz_available": self.vz_available(vxy), "reason": reason}
        return v, info

    def step(self, v_prev, v_target, dt, hard_cap=None):
        a = self.a_up if v_target >= v_prev else self.a_down
        dv = max(-a * dt, min(a * dt, v_target - v_prev))
        v = max(0.0, min(self.max_speed, v_prev + dv))
        if hard_cap is not None:
            v = min(v, max(0.0, float(hard_cap)))
        return v
