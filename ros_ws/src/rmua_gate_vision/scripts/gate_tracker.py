#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gate Tracker: 逐帧观测 -> 持久世界坐标 track (方案 6/11/14/20/21/27/45 节)。

要点:
  - 新观测不直接覆盖旧值, 用 EMA 融合 (远距离 beta 小, 近距离 beta 大);
  - 记录 World XYZ 方差 sigma, sigma_z 用于拒绝不稳定 Z 进入 Hard Anchor;
  - 关联优先使用 **反投影预测像素** (方案 27): score = w_pixel*pixel_dist
    + w_depth*depth_diff + route_s/size 差; 无预测时退回世界距离;
  - support 足 + sigma 小 + 几何有效 -> hard_anchor, 否则 soft;
  - track 短期丢失不删除 (persistent), 由 max_age 老化。
"""

import math

import numpy as np

try:
    from gate_reprojection import reproject_gate
except Exception:                       # 允许无 cv2/ros 环境单测
    reproject_gate = None


class GateTrack(object):

    def __init__(self, tid, world, route_s, stamp, depth=None, geom=None,
                 uv=None, conf=None):
        w = np.asarray(world, dtype=float)
        self.id = int(tid)
        self.mean = w.copy()
        self.s = float(route_s) if route_s is not None else 0.0
        self.support = 1
        self.first_seen = stamp
        self.last_seen = stamp
        self.depth = depth
        self._mean = w.copy()
        self._m2 = np.zeros(3)
        self.hard_anchor = False
        # 几何 / 外观
        self.width = geom["width"] if geom else None
        self.height = geom["height"] if geom else None
        self.plane_rmse = geom["plane_rmse"] if geom else None
        self.geometry_valid = bool(geom["geometry_valid"]) if geom else False
        self.geom_frames = 1 if geom else 0
        self.geom_ok_frames = 1 if (geom and geom["geometry_valid"]) else 0
        self.confidence = float(conf) if conf is not None else None
        self.last_uv = uv
        # 反投影预测 (由节点按当前相机位姿填充)
        self.predicted_u = None
        self.predicted_v = None
        self.predicted_depth = None
        self.visible = False

    def update(self, world, route_s, stamp, beta, depth=None, geom=None,
               uv=None, conf=None):
        w = np.asarray(world, dtype=float)
        self.mean = (1.0 - beta) * self.mean + beta * w
        n_new = self.support + 1
        delta = w - self._mean
        self._mean = self._mean + delta / n_new
        self._m2 = self._m2 + delta * (w - self._mean)
        self.support = n_new
        self.last_seen = stamp
        if route_s is not None:
            self.s = float(route_s)
        if depth is not None:
            self.depth = depth
        if uv is not None:
            self.last_uv = uv
        if geom is not None:
            a = 0.3
            self.width = geom["width"] if self.width is None else (1 - a) * self.width + a * geom["width"]
            self.height = geom["height"] if self.height is None else (1 - a) * self.height + a * geom["height"]
            self.plane_rmse = geom["plane_rmse"] if self.plane_rmse is None else \
                (1 - a) * self.plane_rmse + a * geom["plane_rmse"]
            self.geometry_valid = bool(geom["geometry_valid"])
            self.geom_frames += 1
            self.geom_ok_frames += 1 if geom["geometry_valid"] else 0
        if conf is not None:
            self.confidence = conf if self.confidence is None else \
                0.8 * self.confidence + 0.2 * conf

    def sigma(self):
        if self.support < 2:
            return np.array([1e3, 1e3, 1e3])
        return np.sqrt(self._m2 / (self.support - 1))

    def geometry_ok_ratio(self):
        return self.geom_ok_frames / self.geom_frames if self.geom_frames else 0.0


class GateTracker(object):

    def __init__(self, assoc_radius=4.0, beta_far=0.1, beta_near=0.3,
                 near_dist=15.0, far_dist=50.0, max_age=3.0,
                 min_support_hard=5, sigma_hard=0.6,
                 max_pixel_dist=90.0, w_pixel=1.0, w_depth=0.6, w_route=0.5,
                 unpredicted_penalty=0.3):
        self.assoc_radius = float(assoc_radius)
        self.beta_far = float(beta_far)
        self.beta_near = float(beta_near)
        self.near_dist = float(near_dist)
        self.far_dist = float(far_dist)
        self.max_age = float(max_age)
        self.min_support_hard = int(min_support_hard)
        self.sigma_hard = float(sigma_hard)
        self.max_pixel_dist = float(max_pixel_dist)
        self.w_pixel = float(w_pixel)
        self.w_depth = float(w_depth)
        self.w_route = float(w_route)
        self.unpredicted_penalty = float(unpredicted_penalty)
        self.tracks = {}
        self._next_id = 0

    def _beta(self, depth):
        if depth is None:
            return 0.5 * (self.beta_far + self.beta_near)
        t = (self.far_dist - depth) / max(1e-6, self.far_dist - self.near_dist)
        t = max(0.0, min(1.0, t))
        return self.beta_far + t * (self.beta_near - self.beta_far)

    def set_prediction(self, tid, u, v, depth, visible):
        t = self.tracks.get(tid)
        if t is not None:
            t.predicted_u, t.predicted_v, t.predicted_depth = u, v, depth
            t.visible = bool(visible)

    def _assoc_cost(self, t, world, route_s, depth, uv):
        """返回 (cost, mode) 或 (None, None) 表示不匹配。"""
        if uv is not None and t.predicted_u is not None:
            pix = math.hypot(uv[0] - t.predicted_u, uv[1] - t.predicted_v)
            if pix > self.max_pixel_dist:
                return None, None
            cost = self.w_pixel * (pix / self.max_pixel_dist)
            if depth is not None and t.predicted_depth and t.predicted_depth > 1e-6 \
                    and depth > 1e-6:
                cost += self.w_depth * abs(math.log(depth / t.predicted_depth))
            if route_s is not None:
                cost += self.w_route * min(1.0, abs(route_s - t.s) / 15.0)
            return cost, "pixel"
        d = float(np.linalg.norm(world - t.mean))
        if d > self.assoc_radius:
            return None, None
        cost = d / self.assoc_radius
        if uv is not None:
            cost += self.unpredicted_penalty       # 有像素时优先选已有预测的 track
        return cost, "world"

    def update(self, world, route_s, stamp, depth=None, geom=None, uv=None,
               conf=None):
        w = np.asarray(world, dtype=float)
        best, best_cost = None, 1e9
        for t in self.tracks.values():
            cost, _ = self._assoc_cost(t, w, route_s, depth, uv)
            if cost is not None and cost < best_cost:
                best_cost, best = cost, t
        if best is not None:
            best.update(w, route_s, stamp, self._beta(depth), depth, geom, uv, conf)
            return best
        t = GateTrack(self._next_id, w, route_s, stamp, depth, geom, uv, conf)
        self.tracks[self._next_id] = t
        self._next_id += 1
        return t

    def prune(self, stamp):
        dead = [tid for tid, t in self.tracks.items()
                if (stamp - t.last_seen) > self.max_age]
        for tid in dead:
            del self.tracks[tid]
        return dead

    def refresh_hard(self):
        for t in self.tracks.values():
            s = t.sigma()
            geom_ok = t.geom_frames == 0 or t.geometry_valid
            t.hard_anchor = (t.support >= self.min_support_hard and
                             float(np.max(s)) <= self.sigma_hard and geom_ok)
        return self.tracks

    def sorted_tracks(self, s_now=None):
        ts = list(self.tracks.values())
        if s_now is not None:
            ts = [t for t in ts if t.s > s_now]
        ts.sort(key=lambda t: t.s)
        return ts
