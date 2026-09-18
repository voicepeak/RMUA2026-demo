#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gate Guidance: 由门状态/几何产生 v_gate (XY; Z 由 AltitudeProfile 统一给出)。"""

import math

import numpy as np

CAPTURE, ALIGN, CROSS, EXIT, RECOVER = "CAPTURE", "ALIGN", "CROSS", "EXIT", "RECOVER"


class GateGuidance(object):

    def __init__(self, k_far=0.9, k_mid=1.4, k_near=2.0,
                 speed_far=2.0, speed_mid=1.4, align_speed=1.0,
                 cross_speed=1.2, k_center=1.5, approach_distance=4.0):
        self.k_far, self.k_mid, self.k_near = k_far, k_mid, k_near
        self.speed_far, self.speed_mid = speed_far, speed_mid
        self.align_speed, self.cross_speed = align_speed, cross_speed
        self.k_center = k_center
        self.approach_distance = approach_distance

    def k_xy(self, d):
        if d > 15.0:
            return self.k_far
        if d > 10.0:
            return self.k_mid
        return self.k_near

    def speed(self, d):
        if d > 15.0:
            return self.speed_far
        if d > 10.0:
            return self.speed_mid
        return self.align_speed

    def compute(self, info, p):
        state = info["state"]
        G, n = info["G"], info["n"]
        p = np.asarray(p, dtype=float)
        if state in (CAPTURE, ALIGN, RECOVER):
            target = G - self.approach_distance * n if state == RECOVER else G
            v = self.k_xy(info["d_g"]) * (target - p)
            lim = self.speed(info["d_g"])
            h = math.hypot(v[0], v[1])
            if h > lim and h > 1e-6:
                v *= lim / h
            v[2] = 0.0
            return v
        if state in (CROSS, EXIT):
            v = self.cross_speed * n - self.k_center * info["e_plane"]
            v[2] = 0.0
            return v
        return np.zeros(3)
