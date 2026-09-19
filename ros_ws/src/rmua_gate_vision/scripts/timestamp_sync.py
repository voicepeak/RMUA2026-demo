#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Timestamp Sync: 把位置/姿态对齐到图像时间戳 (方案 15~19, 42 节)。

问题: 相机帧 t_camera 与最近一次 IMU/pose t_latest 不同步时, 高速(10 m/s)
50ms 就移动 0.5m, 转弯时姿态差异会把 "前方距离" 错投影成 World Z 漂移。

做法:
  - 缓存最近 ~1s 的 IMU/pose;
  - 给定图像时间戳 t, 找到 t0 < t < t1;
  - 位置线性插值, 姿态四元数 SLERP;
  - 超出缓存范围则退回最近样本并标记 stale。
"""

import math
from collections import deque

import numpy as np


def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


def slerp(q0, q1, t):
    """四元数球面插值, 输入 (x,y,z,w)。"""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:                       # 走短弧
        q1 = -q1
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:                    # 几乎共线 -> 线性
        return quat_normalize(q0 + t * (q1 - q0))
    theta0 = math.acos(dot)
    theta = theta0 * t
    s0 = math.sin(theta0 - theta) / math.sin(theta0)
    s1 = math.sin(theta) / math.sin(theta0)
    return quat_normalize(s0 * q0 + s1 * q1)


class PoseBuffer(object):

    def __init__(self, max_age=1.0):
        self.max_age = float(max_age)
        self.buf = deque()              # (t, pos(np3), quat(np4))

    def add(self, t, pos, quat):
        t = float(t)
        if self.buf and t <= self.buf[-1][0]:   # 乱序/重复: 忽略
            return
        self.buf.append((t, np.asarray(pos, dtype=float).copy(),
                         quat_normalize(quat)))
        while self.buf and t - self.buf[0][0] > self.max_age:
            self.buf.popleft()

    def ready(self):
        return len(self.buf) > 0

    def sample(self, t):
        """返回 (pos, quat, stale)。无数据返回 (None, None, True)。"""
        if not self.buf:
            return None, None, True
        t = float(t)
        if t <= self.buf[0][0]:
            return self.buf[0][1].copy(), self.buf[0][2].copy(), True
        if t >= self.buf[-1][0]:
            return self.buf[-1][1].copy(), self.buf[-1][2].copy(), True
        for i in range(len(self.buf) - 1):
            t0, p0, q0 = self.buf[i]
            t1, p1, q1 = self.buf[i + 1]
            if t0 <= t <= t1:
                if t1 - t0 < 1e-9:
                    return p1.copy(), q1.copy(), False
                a = (t - t0) / (t1 - t0)
                pos = p0 + a * (p1 - p0)
                quat = slerp(q0, q1, a)
                return pos, quat, False
        return self.buf[-1][1].copy(), self.buf[-1][2].copy(), True
