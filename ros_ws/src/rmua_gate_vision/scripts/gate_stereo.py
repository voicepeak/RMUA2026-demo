#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双目匹配 / 三角化 / 坐标变换 (Detector 与 Planner 之间的几何层)。"""

import cv2
import numpy as np

# 官方前视双目 (settings.json): 960x720, FOV60, 基线 0.30 m, 左 Y=-0.15 右 Y=+0.15
FX = FY = 831.4
CX, CY = 480.0, 360.0
BASELINE = 0.30
CAM_LEFT_BODY = np.array([0.175, -0.15, 0.0])   # body(NED/FRD) 下左相机位置

# optical(x右,y下,z前) -> body(x前,y右,z下)
R_BC = np.array([[0.0, 0.0, 1.0],
                 [1.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0]])


def projection_matrices():
    K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1.0]])
    P1 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])
    P2 = K @ np.hstack([np.eye(3), np.array([[-BASELINE], [0.0], [0.0]])])
    return K, P1, P2


def match_gates(left, right, max_dv=60.0, max_cost=120.0):
    pairs = []
    for gl in left:
        best, best_cost = None, 1e9
        for gr in right:
            dv = abs(gl["center"][1] - gr["center"][1])
            if dv > max_dv:
                continue
            disp = gl["center"][0] - gr["center"][0]
            if disp <= 1.0 or disp > 400.0:
                continue
            size_ratio = abs(gl["area"] - gr["area"]) / max(gl["area"], gr["area"])
            cost = dv + 40.0 * size_ratio
            if cost < best_cost:
                best_cost, best = cost, gr
        if best is not None and best_cost < max_cost:
            pairs.append((gl, best, best_cost))
    return pairs


def triangulate_gate(gl, gr):
    _, P1, P2 = projection_matrices()
    pl = gl["corners"].T.astype(np.float64)
    pr = gr["corners"].T.astype(np.float64)
    X = cv2.triangulatePoints(P1, P2, pl, pr)
    X = (X[:3] / X[3]).T.astype(np.float64)      # 4x3, 左相机光学系
    center = X.mean(axis=0)
    v1 = X[1] - X[0]        # TR - TL
    v2 = X[3] - X[0]        # BL - TL
    n = np.cross(v1, v2)
    nn = np.linalg.norm(n)
    normal = n / nn if nn > 1e-9 else np.array([0.0, 0.0, 1.0])
    return X, center, normal


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def camera_to_body(P_cam):
    return (R_BC @ np.asarray(P_cam, dtype=np.float64).T).T + CAM_LEFT_BODY


def body_to_world(P_body, pos, quat):
    R = quat_to_R(quat)
    return (R @ np.asarray(P_body, dtype=np.float64).T).T + np.asarray(pos)


def gate_world(gl, gr, pos, quat):
    """返回世界系中心与法向。"""
    X, center, normal = triangulate_gate(gl, gr)
    center_body = camera_to_body(center)
    world = body_to_world(center_body, pos, quat)
    n_body = R_BC @ normal
    n_world = quat_to_R(quat) @ n_body
    nn = np.linalg.norm(n_world)
    n_world = n_world / nn if nn > 1e-9 else n_world
    return world, n_world, center, X


# ---------------- 基于稠密视差的 Gate 定位 ----------------
# Gate 是平面框, 框体像素的深度 ≈ 门中心深度; 因此对门框像素取中位视差,
# 再反投影门中心像素, 避免"空心中心取深度"的问题。

def build_sgbm(num_disp=128, block=5):
    return cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=num_disp, blockSize=block,
        P1=8 * 3 * block * block, P2=32 * 3 * block * block,
        disp12MaxDiff=1, uniquenessRatio=10,
        speckleWindowSize=100, speckleRange=2,
        mode=cv2.STEREO_SGBM_MODE_SGBM)


def compute_disparity(gray_l, gray_r, sgbm):
    return sgbm.compute(gray_l, gray_r).astype(np.float32) / 16.0


def gate_center_from_frame(cand, disp, frame_mask, min_pixels=25):
    """用门框像素的中位视差求门中心相机坐标; 返回 (center_cam(3), depth, n_pix) 或 None。"""
    x, y, w, h = cand["bbox"]
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(frame_mask.shape[1], x + w), min(frame_mask.shape[0], y + h)
    if x1 <= x0 or y1 <= y0:
        return None
    m = frame_mask[y0:y1, x0:x1]
    dv = disp[y0:y1, x0:x1][m]
    dv = dv[dv > 0]
    if dv.size < min_pixels:
        return None
    depth = FX * BASELINE / float(np.median(dv))
    cu, cv_ = cand["center"]
    center = np.array([(cu - CX) * depth / FX,
                       (cv_ - CY) * depth / FY,
                       depth])
    return center, depth, int(dv.size)

