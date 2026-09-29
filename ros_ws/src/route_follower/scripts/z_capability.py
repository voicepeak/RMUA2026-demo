#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VzCapability: 实测/估算的垂直能力表 vz_available(vxy) (方案 14~20, 4.5, 45 节)。

上升与下降能力分开标定 (动力学不对称):
    up(vxy)    爬升率上限 m/s
    down(vxy)  下降率上限 m/s

YAML 支持新格式:
    vz_available:
      up:
        - {vxy: 0.0, vz: 4.0}
      down:
        - {vxy: 0.0, vz: 3.0}

同时兼容旧格式 (只有 vz_up / [vxy, vz_up] 列表), 旧格式的 down 取 up 表。

由 z_capability_probe.py 实测 CSV 整理后覆盖本文件。
"""

import os

DEFAULT_UP = [(0.0, 4.0), (5.0, 3.6), (10.0, 3.0)]
DEFAULT_DOWN = [(0.0, 3.0), (5.0, 3.0), (10.0, 3.0)]


def _interp(table, vxy):
    vxy = float(vxy)
    t = table
    if vxy <= t[0][0]:
        return t[0][1]
    if vxy >= t[-1][0]:
        return t[-1][1]
    for (x0, y0), (x1, y1) in zip(t[:-1], t[1:]):
        if x0 <= vxy <= x1:
            r = (vxy - x0) / max(1e-9, x1 - x0)
            return y0 + r * (y1 - y0)
    return t[-1][1]


def _parse_points(rows, keys):
    pts = []
    for g in rows:
        if "vxy" not in g:
            continue
        for key in keys:
            if key in g:
                pts.append((float(g["vxy"]), float(g[key])))
                break
    return pts


class VzCapability(object):

    def __init__(self, up_table=None, down_table=None):
        up = [(float(a), float(b)) for a, b in (up_table or DEFAULT_UP)]
        down = [(float(a), float(b)) for a, b in (down_table or up_table or DEFAULT_DOWN)]
        self.up_table = sorted(up, key=lambda p: p[0])
        self.down_table = sorted(down, key=lambda p: p[0])

    def up(self, vxy):
        return _interp(self.up_table, vxy)

    def down(self, vxy):
        return _interp(self.down_table, vxy)

    def available(self, vxy):
        """向后兼容: 旧调用方按上升能力使用。"""
        return self.up(vxy)

    def as_list(self):
        return {"up": [(round(a, 3), round(b, 3)) for a, b in self.up_table],
                "down": [(round(a, 3), round(b, 3)) for a, b in self.down_table]}


def load_capability(path):
    """读取 YAML; 失败或文件缺失时返回默认表。"""
    try:
        import yaml
    except ImportError:
        return VzCapability()
    if not path or not os.path.exists(path):
        return VzCapability()
    try:
        with open(path) as f:
            d = yaml.safe_load(f) or {}
        rows = d.get("vz_available")
        if isinstance(rows, dict):
            up = _parse_points(rows.get("up", []), ("vz", "vz_up"))
            down = _parse_points(rows.get("down", []), ("vz", "vz_down"))
            return VzCapability(up or None, down or up or None)
        if isinstance(rows, list):
            up = _parse_points(rows, ("vz_up", "vz"))
            return VzCapability(up or None, None)
    except Exception:
        return VzCapability()
    return VzCapability()
