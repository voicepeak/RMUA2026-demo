#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenCV Gate 2D 检测: 白色框体 + 橙色边缘 + 四边形几何。

输出统一接口 GateObservation:
  corners (4x2, 顺序 TL,TR,BR,BL), center (u,v), bbox, area, confidence
以后替换为深度学习时只需替换本文件, 下游 matcher/triangulator 不变。
"""

import cv2
import numpy as np


DEFAULTS = {
    "white_s_max": 50,
    "white_v_min": 140,
    "orange_h_min": 0,
    "orange_h_max": 22,
    "orange_s_min": 120,
    "orange_v_min": 120,
    "min_area": 800,
    "max_area": 400000,
    "min_ratio": 0.30,
    "orange_bonus": 0.15,
}


def order_corners(pts):
    pts = np.asarray(pts, dtype=np.float32).reshape(4, 2)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).ravel()
    return np.array([pts[np.argmin(s)], pts[np.argmin(d)],
                     pts[np.argmax(s)], pts[np.argmax(d)]], dtype=np.float32)


def detect_gates(img, params=None):
    p = dict(DEFAULTS)
    if params:
        p.update(params)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]

    white = ((S < p["white_s_max"]) & (V > p["white_v_min"])).astype(np.uint8) * 255
    orange = ((H >= p["orange_h_min"]) & (H <= p["orange_h_max"]) &
              (S >= p["orange_s_min"]) & (V >= p["orange_v_min"])).astype(np.uint8) * 255
    frame = ((white > 0) | cv2.dilate(orange, np.ones((3, 3), np.uint8)) > 0).astype(np.uint8) * 255
    frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    cnts, _ = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cands = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area < p["min_area"] or area > p["max_area"]:
            continue
        rect = cv2.minAreaRect(c)
        w, h = rect[1]
        ratio = min(w, h) / max(max(w, h), 1.0)
        if ratio < p["min_ratio"]:
            continue
        peri = cv2.arcLength(c, True)
        ap = cv2.approxPolyDP(c, 0.02 * peri, True)
        if len(ap) == 4:
            corners = ap.reshape(4, 2).astype(np.float32)
            quad = True
        else:
            corners = cv2.boxPoints(rect).astype(np.float32) if hasattr(cv2, "boxPoints") \
                else cv2.boxPoints(rect).astype(np.float32)
            quad = False
        corners = order_corners(corners)

        # 橙色验证: 框体附近橙色像素比例
        x, y, bw, bh = cv2.boundingRect(c)
        roi_o = orange[y:y + bh, x:x + bw]
        o_ratio = float((roi_o > 0).mean()) if roi_o.size else 0.0
        conf = min(0.6, area / 20000.0) + (0.2 if quad else 0.0) \
            + min(1.0, o_ratio / p["orange_bonus"]) * 0.2
        cands.append({
            "corners": corners,
            "center": corners.mean(axis=0),
            "bbox": (x, y, bw, bh),
            "area": float(area),
            "quad": quad,
            "orange_ratio": o_ratio,
            "confidence": float(conf),
            "n_approx": len(ap),
        })
    cands.sort(key=lambda d: -d["area"])
    return cands


def gate_frame_mask(img, params=None):
    """返回 Gate 框体候选像素 mask (白框 + 橙边), 用于视差取深度。"""
    p = dict(DEFAULTS)
    if params:
        p.update(params)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    white = (S < p["white_s_max"]) & (V > p["white_v_min"])
    orange = ((H >= p["orange_h_min"]) & (H <= p["orange_h_max"]) &
              (S >= p["orange_s_min"]) & (V >= p["orange_v_min"]))
    return white | orange


def draw(img, cands, color=(0, 0, 255)):
    out = img.copy()
    for d in cands:
        c = d["corners"].astype(int)
        cv2.polylines(out, [c], True, color, 2)
        for i, pt in enumerate(c):
            cv2.circle(out, tuple(pt), 4, (0, 255, 0), -1)
            cv2.putText(out, "TL TR BR BL".split()[i], tuple(pt),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cu, cv_ = d["center"].astype(int)
        cv2.putText(out, "%.2f" % d["confidence"], (cu, cv_),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    return out
