#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stereo Keypoint Matcher: 左右图 Gate 四角匹配 + 逐角三角化 (方案 5, 41 节)。

- 左右 Gate 先按中心/尺寸/视差做门级匹配;
- 门级匹配后按固定顺序 TL,TR,BR,BL 一一对应;
- 对每个角点 cv2.triangulatePoints -> 4 个相机系 3D 点;
- 交给 gate_geometry.GateGeometry 计算中心/法向/尺寸/质量。

这样 Gate Center 来自门框真实四角几何, 而不是门洞背景的 bbox 中心深度。
"""

import cv2
import numpy as np

from gate_stereo import CX, CY, FX, FY, BASELINE, projection_matrices


def match_gate_corners(left, right, max_dv=60.0, max_disp=400.0, max_cost=60.0):
    """左右 Gate 门级匹配, 返回 [(gl, gr, cost), ...]。"""
    pairs = []
    for gl in left:
        cl = np.asarray(gl["corners"], dtype=float)
        best, best_cost = None, 1e9
        for gr in right:
            cr = np.asarray(gr["corners"], dtype=float)
            dv = abs(float(gl["center"][1]) - float(gr["center"][1]))
            disp = float(gl["center"][0]) - float(gr["center"][0])
            if dv > max_dv or disp <= 1.0 or disp > max_disp:
                continue
            area_ratio = abs(gl.get("area", 1.0) - gr.get("area", 1.0)) / \
                max(gl.get("area", 1.0), gr.get("area", 1.0), 1e-6)
            shift = np.array([disp, 0.0])
            resid = float(np.mean(np.linalg.norm((cl - shift) - cr, axis=1)))
            cost = dv + 30.0 * area_ratio + resid
            if cost < best_cost:
                best_cost, best = cost, gr
        if best is not None and best_cost < max_cost:
            pairs.append((gl, best, best_cost))
    return pairs


def triangulate_corners(gl_corners, gr_corners):
    """输入左右 4x2 角点 (TL,TR,BR,BL), 返回左相机光学系 4x3 3D 点。"""
    _, P1, P2 = projection_matrices()
    pl = np.asarray(gl_corners, dtype=np.float64).T
    pr = np.asarray(gr_corners, dtype=np.float64).T
    X = cv2.triangulatePoints(P1, P2, pl, pr)
    w = X[3]
    w[np.abs(w) < 1e-12] = 1e-12
    return (X[:3] / w).T.astype(np.float64)


def corner_disparities(gl_corners, gr_corners):
    gl = np.asarray(gl_corners, dtype=float)
    gr = np.asarray(gr_corners, dtype=float)
    return gl[:, 0] - gr[:, 0]
