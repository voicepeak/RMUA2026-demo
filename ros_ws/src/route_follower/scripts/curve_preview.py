#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Curve Braking Envelope: 前方曲率限速 + 制动距离约束 (方案 8)。

局部限速 v = sqrt(a_lat_max / kappa_now) 只在已经进入急弯后才生效, 减速来不及。
改为向前采样未来曲率限速 v_future_i, 并用最大允许减速度反推当前允许速度:

    v_now_cap_i = sqrt(v_future_i^2 + 2 * a_brake * ds)
    v_curve_preview = min(v_now_cap_i)

结果作为 SpeedScheduler 的硬上限输入, 使转弯提前平滑减速。
"""

import math


class CurveBrakingEnvelope(object):

    def __init__(self, preview_time=3.0, preview_min=20.0, preview_max=40.0,
                 preview_step=3.0, a_brake=4.0):
        self.preview_time = float(preview_time)
        self.preview_min = float(preview_min)
        self.preview_max = float(preview_max)
        self.preview_step = max(0.5, float(preview_step))
        self.a_brake = max(0.1, float(a_brake))

    def horizon(self, vxy):
        h = self.preview_time * max(0.0, float(vxy))
        return max(self.preview_min, min(self.preview_max, h))

    def evaluate(self, s_now, curvature_fn, a_lat_max, v_test, cruise=None):
        """返回 dict(v_curve_preview, curve_horizon, curve_worst_s, curve_worst_v)。

        curvature_fn(s) -> |kappa|
        a_lat_max: 横向加速度能力 m/s^2
        v_test: 当前速度, 决定预览距离
        cruise: 若给出, 用其裁剪单点限速 (避免未来点比巡航还快)
        """
        horizon = self.horizon(v_test)
        cap = float("inf")
        worst_s, worst_v = s_now, float("inf")
        ds = self.preview_step
        while ds <= horizon + 1e-6:
            s_i = s_now + ds
            v_future = math.sqrt(a_lat_max / (abs(curvature_fn(s_i)) + 1e-3))
            if cruise is not None:
                v_future = min(v_future, float(cruise))
            v_now_cap = math.sqrt(v_future * v_future + 2.0 * self.a_brake * ds)
            if v_now_cap < cap:
                cap = v_now_cap
                worst_s, worst_v = s_i, v_future
            ds += self.preview_step
        if not math.isfinite(cap):
            cap = float("inf")
        return {"v_curve_preview": cap, "curve_horizon": horizon,
                "curve_worst_s": worst_s, "curve_worst_v": worst_v}
