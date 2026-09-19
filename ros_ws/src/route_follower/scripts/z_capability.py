#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VzCapability: 实测垂直能力表 vz_available(vxy) (方案 14~20, 45 节)。

由 z_capability_probe.py 测得的 CSV 拟合/整理成 (vxy, vz_up) 分段表,
Speed Scheduler 用它替换固定的 vz_up_safe。

默认表为初始估值 (待实测覆盖)。
"""

import os

DEFAULT_TABLE = [(0.0, 4.0), (5.0, 3.6), (10.0, 3.0)]


class VzCapability(object):

    def __init__(self, table=None):
        pts = [(float(a), float(b)) for a, b in (table or DEFAULT_TABLE)]
        self.table = sorted(pts, key=lambda p: p[0])

    def available(self, vxy):
        """线性插值; 超出两端取端点。"""
        vxy = float(vxy)
        t = self.table
        if vxy <= t[0][0]:
            return t[0][1]
        if vxy >= t[-1][0]:
            return t[-1][1]
        for (x0, y0), (x1, y1) in zip(t[:-1], t[1:]):
            if x0 <= vxy <= x1:
                r = (vxy - x0) / max(1e-9, x1 - x0)
                return y0 + r * (y1 - y0)
        return t[-1][1]

    def as_list(self):
        return [(round(a, 3), round(b, 3)) for a, b in self.table]


def load_capability(path):
    """从 YAML 读取:
    vz_available:
      - vxy: 0
        vz_up: 4.0
      ...
    失败则返回默认表。
    """
    if not path or not os.path.exists(path):
        return VzCapability()
    try:
        import yaml
        with open(path) as f:
            d = yaml.safe_load(f) or {}
        pts = [(g["vxy"], g["vz_up"]) for g in d.get("vz_available", [])
               if "vxy" in g and "vz_up" in g]
        return VzCapability(pts) if pts else VzCapability()
    except Exception:
        return VzCapability()
