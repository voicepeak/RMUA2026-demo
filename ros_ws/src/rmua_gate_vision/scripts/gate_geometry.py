#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gate Geometry: 四角 3D 点 -> 中心/法向/尺寸/平面残差/质量 (方案 6~13, 43 节)。

输入 4 个角点 (顺序固定 TL, TR, BR, BL, 相机或世界系), 输出:
  center, normal, width, height, top/bottom_width, left/right_height,
  plane_rmse, depth_min/max, aspect, geometry_valid。

几何一致性检查 (方案 9~13):
  - 深度有效且非负;
  - 平面拟合 RMSE (不能要求四角深度相等, 斜视本来就有差);
  - 上下宽 / 左右高 不应差太离谱;
  - 宽高比合理;
  - 尺寸在合理区间。
不通过则 geometry_valid=False, 该帧不更新 Hard Anchor。
"""

import numpy as np


def fit_plane(P):
    """最小二乘(总体最小二乘/SVD)拟合平面, 返回 (center, normal, rmse)。"""
    P = np.asarray(P, dtype=float)
    c = P.mean(axis=0)
    Q = P - c
    try:
        _, _, vt = np.linalg.svd(Q, full_matrices=False)
        normal = vt[2]
    except np.linalg.LinAlgError:
        normal = np.array([0.0, 0.0, 1.0])
    n = np.linalg.norm(normal)
    normal = normal / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])
    dist = Q @ normal
    rmse = float(np.sqrt(np.mean(dist * dist)))
    return c, normal, rmse


class GateGeometry(object):

    def __init__(self, min_width=0.5, max_width=8.0, min_height=0.5,
                 max_height=8.0, min_aspect=0.4, max_aspect=2.5,
                 plane_rmse_max=0.25, side_mismatch_max=0.5, min_depth=0.5):
        self.min_width = min_width
        self.max_width = max_width
        self.min_height = min_height
        self.max_height = max_height
        self.min_aspect = min_aspect
        self.max_aspect = max_aspect
        self.plane_rmse_max = plane_rmse_max
        self.side_mismatch_max = side_mismatch_max
        self.min_depth = min_depth

    def compute(self, P, camera_origin=None):
        """P: 4x3 (TL,TR,BR,BL)。camera_origin: 用于法向朝向 (相机系填 0)。"""
        P = np.asarray(P, dtype=float).reshape(4, 3)
        g = {"geometry_valid": False, "reason": ""}
        if not np.all(np.isfinite(P)):
            g["reason"] = "nonfinite"
            return g
        depths = P[:, 2]
        top = float(np.linalg.norm(P[1] - P[0]))
        bottom = float(np.linalg.norm(P[2] - P[3]))
        left = float(np.linalg.norm(P[3] - P[0]))
        right = float(np.linalg.norm(P[2] - P[1]))
        width = 0.5 * (top + bottom)
        height = 0.5 * (left + right)
        aspect = width / height if height > 1e-6 else 0.0
        center, normal, rmse = fit_plane(P)
        if camera_origin is not None:
            origin = np.asarray(camera_origin, dtype=float)
            if float(normal @ (origin - center)) < 0.0:
                normal = -normal
        side_mismatch = (abs(top - bottom) / max(top, bottom, 1e-6) +
                         abs(left - right) / max(left, right, 1e-6)) * 0.5
        g.update({
            "center": center, "normal": normal,
            "width": width, "height": height,
            "top_width": top, "bottom_width": bottom,
            "left_height": left, "right_height": right,
            "plane_rmse": rmse, "aspect": aspect,
            "depth_min": float(depths.min()), "depth_max": float(depths.max()),
            "depth_mean": float(depths.mean()),
            "side_mismatch": float(side_mismatch),
        })
        if depths.min() <= self.min_depth or depths.max() <= self.min_depth:
            g["reason"] = "depth"
        elif rmse > self.plane_rmse_max:
            g["reason"] = "plane_rmse"
        elif not (self.min_width <= width <= self.max_width and
                  self.min_height <= height <= self.max_height):
            g["reason"] = "size"
        elif not (self.min_aspect <= aspect <= self.max_aspect):
            g["reason"] = "aspect"
        elif side_mismatch > self.side_mismatch_max:
            g["reason"] = "side_mismatch"
        else:
            g["geometry_valid"] = True
        return g
