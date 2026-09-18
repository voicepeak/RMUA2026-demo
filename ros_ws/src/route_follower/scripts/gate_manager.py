#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gate Manager: 全 Gate 顺序状态机 + 门平面误差几何。

状态: PATH -> CAPTURE -> ALIGN -> CROSS -> EXIT -> PATH  (失败 -> RECOVER)
"""

import math

import numpy as np

PATH, CAPTURE, ALIGN, CROSS, EXIT, RECOVER = \
    "PATH", "CAPTURE", "ALIGN", "CROSS", "EXIT", "RECOVER"


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


class GateManager(object):

    def __init__(self, gates, capture_start=22.0, full_control=10.0,
                 align_xy_tol=0.5, align_z_tol=0.35, aperture_xy_tol=1.2,
                 aperture_z_tol=0.6, approach_distance=4.0, exit_distance=3.0,
                 max_retry=3):
        self.gates = gates
        self.capture_start = capture_start
        self.full_control = full_control
        self.align_xy_tol = align_xy_tol
        self.align_z_tol = align_z_tol
        self.aperture_xy_tol = aperture_xy_tol
        self.aperture_z_tol = aperture_z_tol
        self.approach_distance = approach_distance
        self.exit_distance = exit_distance
        self.max_retry = max_retry
        self.idx = 0
        self.state = PATH
        self.retry = 0
        self.prev_e_n = None
        self.exit_from = 0.0
        self.pass_count = 0

    def next_gate(self):
        return self.gates[self.idx] if self.idx < len(self.gates) else None

    def geometry(self, g, p):
        G = np.array([g["x"], g["y"], g["z"]], dtype=float)
        n = _unit(np.array([g.get("nx", 1.0), g.get("ny", 0.0), g.get("nz", 0.0)], dtype=float))
        r = np.asarray(p, dtype=float) - G
        e_n = float(r @ n)
        e_plane = r - e_n * n
        lat = float(math.hypot(e_plane[0], e_plane[1]))
        vert = float(e_plane[2])
        d_g = float(np.linalg.norm(r))
        return G, n, r, e_n, e_plane, lat, vert, d_g

    def alpha(self, d_g):
        return float(np.clip((self.capture_start - d_g) /
                             max(1e-6, self.capture_start - self.full_control), 0.0, 1.0))

    def step(self, p, dt):
        g = self.next_gate()
        if g is None:
            return {"state": PATH, "gate": None, "alpha": 0.0, "passed": False,
                    "e_n": 0.0, "e_plane": np.zeros(3), "lat": 0.0, "vert": 0.0,
                    "d_g": 1e9, "G": np.zeros(3), "n": np.array([1.0, 0.0, 0.0])}
        G, n, r, e_n, e_plane, lat, vert, d_g = self.geometry(g, p)
        inside_align = lat < self.align_xy_tol and abs(vert) < self.align_z_tol
        inside_ap = lat < self.aperture_xy_tol and abs(vert) < self.aperture_z_tol
        passed = False

        if self.state == PATH:
            if d_g < self.capture_start:
                self.state = CAPTURE
        elif self.state == CAPTURE:
            if d_g < self.full_control:
                self.state = ALIGN
            elif d_g > self.capture_start * 1.3:
                self.state = PATH
        elif self.state == ALIGN:
            if e_n >= 0.0:                       # 已经越过平面还没穿好
                self.state = EXIT if inside_ap else RECOVER
                self.retry += 0 if inside_ap else 1
            elif inside_align:
                self.state = CROSS
            elif d_g > self.capture_start * 1.3:
                self.state = CAPTURE
        elif self.state == CROSS:
            if self.prev_e_n is not None and self.prev_e_n < 0.0 <= e_n:
                if inside_ap:
                    self.state = EXIT
                    self.exit_from = e_n
                else:
                    self.state = RECOVER
                    self.retry += 1
        elif self.state == EXIT:
            if e_n - self.exit_from >= self.exit_distance:
                passed = True
                self.pass_count += 1
                self.idx += 1
                self.state = PATH
                self.retry = 0
                self.prev_e_n = None
        elif self.state == RECOVER:
            target = G - self.approach_distance * n
            if self.retry > self.max_retry:
                self.idx += 1
                self.state = PATH
                self.retry = 0
                self.prev_e_n = None
            elif np.linalg.norm(np.asarray(p) - target) < 1.5 and e_n < 0.0:
                self.state = ALIGN

        self.prev_e_n = e_n
        return {"state": self.state, "gate": g, "alpha": self.alpha(d_g),
                "passed": passed, "passed_id": self.idx - 1 if passed else None,
                "e_n": e_n, "e_plane": e_plane, "lat": lat, "vert": vert,
                "d_g": d_g, "G": G, "n": n, "retry": self.retry,
                "inside": inside_ap}
