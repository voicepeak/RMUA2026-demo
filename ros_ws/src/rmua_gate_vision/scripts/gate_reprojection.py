#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gate Reprojection: Persistent World Gate -> 当前相机 -> 预测像素 (方案 23~30, 44 节)。

用途:
  - Gate 数据关联: 预测位置 vs YOLO 检测位置, 避免前后 Gate 串错;
  - Yaw FOV Correction: 计算未来 Gate 的图像中心偏差 e_img=(u-cx)/fx。

World -> Camera 为 gate_stereo 变换的逆:
  Pb = R_WB^T (Pw - pos)
  Pc = R_BC^T (Pb - CAM_LEFT_BODY)
"""

import numpy as np

from gate_stereo import (CAM_LEFT_BODY, CX, CY, FX, FY, R_BC, quat_to_R)

IMG_W = 960
IMG_H = 720


def world_to_camera(pw, pos, quat):
    R = quat_to_R(quat)
    pb = R.T @ (np.asarray(pw, dtype=float) - np.asarray(pos, dtype=float))
    return R_BC.T @ (pb - CAM_LEFT_BODY)


def camera_to_pixel(pc):
    pc = np.asarray(pc, dtype=float)
    z = pc[2]
    if z <= 1e-6:
        return None, None, float(z)
    return FX * pc[0] / z + CX, FY * pc[1] / z + CY, float(z)


def reproject_gate(pw, pos, quat, img_w=IMG_W, img_h=IMG_H, margin=0.0):
    """返回 dict(u, v, depth, visible); 在相机后方/出画 -> visible=False。"""
    pc = world_to_camera(pw, pos, quat)
    u, v, depth = camera_to_pixel(pc)
    visible = (u is not None and -margin <= u < img_w + margin
               and -margin <= v < img_h + margin)
    return {"u": u, "v": v, "depth": depth, "visible": bool(visible)}


def fov_error(pw, pos, quat):
    """图像横向归一化偏差 e_img=(u-cx)/fx; 不可见返回 None。"""
    r = reproject_gate(pw, pos, quat)
    if not r["visible"] or r["u"] is None:
        return None
    return (r["u"] - CX) / FX
