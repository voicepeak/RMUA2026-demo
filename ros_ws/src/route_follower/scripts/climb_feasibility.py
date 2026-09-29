#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Climb Feasibility Preview: 前方累计高度需求 -> 水平速度上限。

背景 (方案 4):
    局部 dz/ds 限速只看当前坡度, 坡真正开始前 dz/ds≈0, 等进入坡才开始减速
    已经晚了。改为向前扫描未来 H 米的 z_ref(s), 用实测垂直能力判断
    "以当前水平速度是否来得及爬上去", 提前平滑降低 vxy。

算法 (方案 4.2~4.5):
    H = clamp(v * T_preview, H_min, H_max)
    对每个 s_i = s_now + k*step:
        dz_need = z_now - z_ref(s_i)     # NED: >0 需要向上
        t_z     = |dz_need| / vz_avail(v_test)   # up/down 分开
        v_cap_i = ds / (t_z + t_response)
    2 次迭代考虑 vz_avail 本身随 vxy 变化。
    v_climb_preview = min(v_cap_i)

这是一个**速度上限**, 不是新的 vz 控制器; 爬升仍由 Z 轴 FF+FB 完成。
"""

import math


class ClimbFeasibility(object):

    def __init__(self, preview_time=3.5, preview_min=20.0, preview_max=50.0,
                 preview_step=5.0, response_time=0.4, eta=0.8, iterations=2):
        self.preview_time = float(preview_time)
        self.preview_min = float(preview_min)
        self.preview_max = float(preview_max)
        self.preview_step = max(1.0, float(preview_step))
        self.response_time = max(0.0, float(response_time))
        self.eta = float(eta)
        self.iterations = max(1, int(iterations))

    def horizon(self, vxy):
        h = self.preview_time * max(0.0, float(vxy))
        return max(self.preview_min, min(self.preview_max, h))

    def _point_cap(self, ds, dz_up, vz_avail):
        """dz_up>0 需要爬升; 返回该点允许的 vxy。"""
        if ds <= 1e-6:
            return float("inf")
        t_z = abs(dz_up) / max(1e-3, vz_avail)
        t_required = t_z + self.response_time
        return ds / max(1e-3, t_required)

    def evaluate(self, s_now, z_now, center_fn, capability, v_test):
        """返回 dict(v_climb_preview, horizon, worst_s, worst_dz, vz_up, vz_down)。

        center_fn(s) -> z_ref(s) (可为 blended profile)
        capability: 含 up(vxy) / down(vxy) 的对象
        v_test: 迭代初值, 一般取当前 v_target / v_cmd
        """
        horizon = self.horizon(v_test)
        v = max(0.0, float(v_test))
        worst_s, worst_dz = s_now, 0.0
        for _ in range(self.iterations):
            vz_up = self.eta * capability.up(v)
            vz_down = self.eta * capability.down(v)
            cap = float("inf")
            worst_s, worst_dz = s_now, 0.0
            ds = self.preview_step
            while ds <= horizon + 1e-6:
                s_i = s_now + ds
                dz_up = float(z_now) - float(center_fn(s_i))
                vz_avail = vz_up if dz_up >= 0.0 else vz_down
                v_cap = self._point_cap(ds, dz_up, vz_avail)
                if v_cap < cap:
                    cap = v_cap
                    worst_s, worst_dz = s_i, dz_up
                ds += self.preview_step
            if cap == float("inf"):
                break
            v = min(v, cap)
        vz_up = self.eta * capability.up(v)
        vz_down = self.eta * capability.down(v)
        return {"v_climb_preview": min(v, v_test) if math.isfinite(v) else float(v_test),
                "climb_horizon": horizon, "climb_worst_s": worst_s,
                "climb_worst_dz": worst_dz, "climb_vz_up_available": vz_up,
                "climb_vz_down_available": vz_down}
