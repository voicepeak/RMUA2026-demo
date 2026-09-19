#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Persistent Gate Map: 持久世界坐标 Gate 地图 (方案 6/7/40/51 节)。

职责:
  - 接收逐帧 Gate World Observation (含四角几何/深度/像素/时间戳),
    交给 GateTracker 维护稳定 track;
  - predict(pos, quat): 用当前相机位姿反投影所有 track, 供关联与 FOV correction;
  - 提供未来 Gate (s > s_now) 查询;
  - 持久化: track 不因单帧丢失而消失, 只有老化才清理。

输出 gate dict 同时含扁平兼容字段 (id/x/y/z/nx/ny/nz/s/support/valid/hard_anchor/
sigma_x/y/z) 与质量/几何字段 (width/height/plane_rmse/geometry_valid/confidence/
predicted_u/predicted_v/last_seen)。
"""

import json

import numpy as np
import yaml

from gate_tracker import GateTracker

try:
    from gate_reprojection import reproject_gate
except Exception:
    reproject_gate = None


class GateMap(object):

    def __init__(self, route=None, **tracker_kwargs):
        self.tracker = GateTracker(**tracker_kwargs)
        self.route = None
        self.seg_s = None
        self.seg_len = None
        if route is not None:
            self.set_route(route)

    def set_route(self, route):
        self.route = np.asarray(route, dtype=float)
        d = np.diff(self.route[:, :2], axis=0)
        self.seg_len = np.hypot(d[:, 0], d[:, 1])
        self.seg_s = np.concatenate([[0.0], np.cumsum(self.seg_len)])

    def project(self, xy):
        best_d, best_s = 1e9, 0.0
        for i in range(len(self.route) - 1):
            a, b = self.route[i], self.route[i + 1]
            d = b[:2] - a[:2]
            L2 = d @ d
            t = 0.0 if L2 < 1e-9 else float(np.clip((xy - a[:2]) @ d / L2, 0, 1))
            c = a[:2] + t * d
            dd = float(np.linalg.norm(xy - c))
            if dd < best_d:
                best_d, best_s = dd, self.seg_s[i] + t * self.seg_len[i]
        return best_s, best_d

    def tangent(self, s):
        for i in range(len(self.route) - 1):
            if self.seg_s[i] <= s <= self.seg_s[i + 1]:
                d = self.route[i + 1][:2] - self.route[i][:2]
                L = np.linalg.norm(d)
                d = d / L if L > 1e-9 else d
                return np.array([d[0], d[1], 0.0])
        return np.array([1.0, 0.0, 0.0])

    def predict(self, pos, quat):
        """按当前相机位姿反投影所有 track, 写入 predicted_u/v/depth/visible。"""
        if reproject_gate is None or pos is None or quat is None:
            return
        for t in self.tracker.tracks.values():
            r = reproject_gate(t.mean, pos, quat)
            self.tracker.set_prediction(t.id, r["u"], r["v"], r["depth"], r["visible"])

    def observe(self, world, stamp, depth=None, geom=None, uv=None, conf=None):
        w = np.asarray(world, dtype=float)
        s = 0.0
        if self.route is not None:
            s, _ = self.project(w[:2])
        return self.tracker.update(w, s, stamp, depth, geom, uv, conf)

    def prune(self, stamp):
        return self.tracker.prune(stamp)

    def _to_gate(self, t, source="yolo"):
        sig = t.sigma()
        n = self.tangent(t.s) if self.route is not None else np.array([1.0, 0.0, 0.0])
        return {
            "id": int(t.id),
            "x": float(t.mean[0]), "y": float(t.mean[1]), "z": float(t.mean[2]),
            "nx": float(n[0]), "ny": float(n[1]), "nz": float(n[2]),
            "s": float(t.s), "support": int(t.support),
            "sigma_x": float(sig[0]), "sigma_y": float(sig[1]), "sigma_z": float(sig[2]),
            "hard_anchor": bool(t.hard_anchor),
            "last_seen": float(t.last_seen),
            "width": t.width, "height": t.height,
            "plane_rmse": t.plane_rmse,
            "geometry_valid": bool(t.geometry_valid),
            "confidence": t.confidence,
            "predicted_u": t.predicted_u, "predicted_v": t.predicted_v,
            "visible": bool(t.visible),
            "valid": True, "source": source,
        }

    def gates(self, min_support=2, source="yolo", max_d_cross=None, hard_only=False):
        self.tracker.refresh_hard()
        ts = [t for t in self.tracker.tracks.values() if t.support >= min_support]
        ts.sort(key=lambda t: t.s)
        out = []
        for t in ts:
            if hard_only and not t.hard_anchor:
                continue
            g = self._to_gate(t, source)
            if max_d_cross is not None and self.route is not None:
                if self.project(np.array([g["x"], g["y"]]))[1] > max_d_cross:
                    continue
            out.append(g)
        return out

    def future(self, s_now, min_support=1, source="yolo", hard_only=False):
        return [g for g in self.gates(min_support, source, hard_only=hard_only)
                if g["s"] > s_now]

    def to_json(self, min_support=2, source="yolo"):
        return json.dumps(self.gates(min_support, source))

    def save_yaml(self, path, min_support=2, source="yolo", max_d_cross=None):
        gates = self.gates(min_support, source, max_d_cross)
        gates = [dict(g, id=i) for i, g in enumerate(gates)]
        with open(path, "w") as f:
            yaml.safe_dump({"gates": gates}, f, default_flow_style=False, sort_keys=False)
        return len(gates)
