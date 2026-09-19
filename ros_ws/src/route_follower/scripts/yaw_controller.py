#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Yaw Path Following: 让机头(前视相机)提前看向未来赛道。

设计目标 (方案 2~4, 19 节):
  - yaw_target 不盯最近一扇门 (会抖), 而是未来 Gate Chain 拟合曲线 / XY Route 的切线;
  - Yaw 前视距离 (20~30 m) 比位置前视 (8~15 m) 更远, 转弯前就提前转头;
  - 前视双目因此更久保持前方 Gate 在 FOV 内。

优先级 (方案 3 节):
  1) 有可靠未来 Gate Chain -> 沿 "当前位置->未来门" 折线走到 yaw_lookahead
  2) 无未来 Gate -> XY Route 在 s_now + yaw_lookahead 的切向
  3) Route 也异常 -> 保持当前 yaw (rate=0)

坐标: AirSim NED (x 前, y 右)。yaw = atan2(vy, vx)。
"""

import math

import numpy as np

# 与 rmua_gate_vision/gate_stereo.py 保持一致的前视左相机安装参数
CAM_R_BC = np.array([[0.0, 0.0, 1.0],
                     [1.0, 0.0, 0.0],
                     [0.0, 1.0, 0.0]])
CAM_T_BC = np.array([0.175, -0.15, 0.0])
CAM_FX = 831.4
CAM_CX = 480.0


def image_u_error(pw, pos, R_wb, fx=CAM_FX, cx=CAM_CX, R_bc=CAM_R_BC, t_bc=CAM_T_BC):
    """World 点 -> 左相机像素, 返回归一化横向偏差 (u-cx)/fx; 在相机后方返回 None。"""
    pb = R_wb.T @ (np.asarray(pw, dtype=float) - np.asarray(pos, dtype=float))
    pc = R_bc.T @ (pb - t_bc)
    if pc[2] <= 1e-6:
        return None
    return (fx * pc[0] / pc[2] + cx - cx) / fx


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _walk_polyline(pts, dist):
    """沿折线 pts 从起点前进 dist 米, 返回点 (x, y)。不足则返回终点。"""
    if len(pts) == 1:
        return pts[0]
    acc = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg < 1e-9:
            continue
        if acc + seg >= dist:
            r = (dist - acc) / seg
            return (a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]))
        acc += seg
    return pts[-1]


class YawController(object):

    def __init__(self, lookahead=22.0, k=1.0, rate_max=0.8):
        self.lookahead = float(lookahead)
        self.k = float(k)
        self.rate_max = float(rate_max)

    def target_from_gates(self, p_xy, future_gates):
        """future_gates: [(x, y), ...] 已按 s 排序且都在前方。
        构建 当前位置->门... 折线, 走到 lookahead 处作为 yaw_target。"""
        pts = [(float(p_xy[0]), float(p_xy[1]))] + \
              [(float(g[0]), float(g[1])) for g in future_gates]
        return _walk_polyline(pts, self.lookahead)

    def target_from_route(self, p_xy, route_point_xy):
        return (float(route_point_xy[0]), float(route_point_xy[1]))

    def rate(self, p_xy, yaw, target_xy):
        """返回 (yaw_rate, yaw_target, yaw_error)。"""
        dx = target_xy[0] - p_xy[0]
        dy = target_xy[1] - p_xy[1]
        if math.hypot(dx, dy) < 1e-3:
            return 0.0, yaw, 0.0
        desired = math.atan2(dy, dx)
        err = wrap_pi(desired - yaw)
        r = max(-self.rate_max, min(self.rate_max, self.k * err))
        return r, desired, err

    def vision_fov_error(self, gates_world, pos, R_wb, n=3):
        """近 n 个可靠未来 Gate 的加权平均图像偏差 e_img (方案 29~32)。"""
        errs = []
        for pw in gates_world[:max(1, n)]:
            e = image_u_error(pw, pos, R_wb)
            if e is not None:
                errs.append(e)
        if not errs:
            return 0.0
        return sum(errs) / len(errs)
