#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Reference Planner: route 几何 + Gate 调整 + AltitudeProfile 构建 (方案 20, 21)。

- RouteGeometry: 纯几何 (无 ROS 依赖), 提供 point_at / project / project_gate /
  curvature / tangent;
- ReferencePlanner: 由静态 gate + 在线 cache gate 构建 GateChain 与
  AltitudeProfile; VERIFIED 门永远保留为高度锚点 (方案 11);
  在线 cache 对静态门的 correction 在这里应用 (方案 14)。
"""

import math

from altitude_profile import AltitudeProfile
from route_height_prior import RouteHeightPrior
from gate_chain import GateChain, VERIFIED, HARD, SOFT

DEFAULT_CONTROL_RATE = 20.0


class RouteGeometry(object):

    def __init__(self, points):
        self.points = [(float(p[0]), float(p[1]), float(p[2])) for p in points]
        self.seg_len = []
        self.seg_s = [0.0]
        acc = 0.0
        for a, b in zip(self.points[:-1], self.points[1:]):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            self.seg_len.append(length)
            acc += length
            self.seg_s.append(acc)

    @property
    def total_s(self):
        return self.seg_s[-1]

    def point_at(self, s):
        s = max(0.0, min(self.seg_s[-1], float(s)))
        for i in range(len(self.seg_len)):
            if self.seg_s[i] <= s <= self.seg_s[i + 1]:
                r = (s - self.seg_s[i]) / self.seg_len[i] if self.seg_len[i] > 1e-9 else 0.0
                a, b = self.points[i], self.points[i + 1]
                return (a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]), i)
        a, b = self.points[-2], self.points[-1]
        return (b[0], b[1], len(self.points) - 2)

    def project(self, p):
        px, py = float(p[0]), float(p[1])
        best = (0, 0.0, 1e9, None)
        for i in range(len(self.points) - 1):
            a, b = self.points[i], self.points[i + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            den = dx * dx + dy * dy
            t = 0.0 if den < 1e-9 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / den))
            cx, cy = a[0] + t * dx, a[1] + t * dy
            d = math.hypot(px - cx, py - cy)
            if d < best[2]:
                best = (i, t, d, (cx, cy, a[2] + t * (b[2] - a[2])))
        return best

    def project_gate(self, x, y):
        best_s, best_d = 0.0, 1e9
        for i in range(len(self.points) - 1):
            a, b = self.points[i], self.points[i + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            den = dx * dx + dy * dy
            t = 0.0 if den < 1e-9 else max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / den))
            cx, cy = a[0] + t * dx, a[1] + t * dy
            d = math.hypot(x - cx, y - cy)
            if d < best_d:
                best_d, best_s = d, self.seg_s[i] + t * self.seg_len[i]
        return best_s

    def curvature(self, s):
        _, _, i = self.point_at(s)
        i = max(1, min(i, len(self.points) - 3))
        a, b, c = self.points[i - 1], self.points[i], self.points[i + 1]
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        n1 = math.hypot(*v1)
        n2 = math.hypot(*v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        ang = abs(math.atan2(cross, dot))
        ds = 0.5 * (n1 + n2)
        return ang / max(ds, 1e-3)

    def tangent(self, s, h=4.0):
        x0, y0, _ = self.point_at(max(0.0, s - h))
        x1, y1, _ = self.point_at(min(self.seg_s[-1], s + h))
        dx, dy = x1 - x0, y1 - y0
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return (1.0, 0.0, 0.0)
        return (dx / norm, dy / norm, 0.0)


class ReferencePlanner(object):

    def __init__(self, route, start_gate=0, gate_z_uses_offset=False,
                 snap_gate_to_route=True, gate_center_pull_max=2.0,
                 slope_factor=3.0, slope_abs_max=0.6, sigma_z_max=0.6,
                 startup_z_jump_limit=1.0,
                 corridor_half=1.5, gate_blend_start=25.0, gate_blend_full=8.0,
                 z_rate_max=4.0,
                 z_extrap_m=50.0, z_extrap_slope_max=0.5,
                 z_soft_guide_max=120.0, z_soft_slope_max=0.5):
        self.route = route
        self.height_prior = RouteHeightPrior(route)
        self.trend_horizon = None
        self.evidence_horizon = None
        self.start_gate = int(start_gate)
        self.gate_z_uses_offset = bool(gate_z_uses_offset)
        self.snap_gate_to_route = bool(snap_gate_to_route)
        self.gate_center_pull_max = float(gate_center_pull_max)
        self.slope_factor = slope_factor
        self.slope_abs_max = slope_abs_max
        self.sigma_z_max = sigma_z_max
        self.startup_z_jump_limit = startup_z_jump_limit
        self.corridor_half = corridor_half
        self.gate_blend_start = gate_blend_start
        self.gate_blend_full = gate_blend_full
        self.z_rate_max = z_rate_max
        self.z_extrap_m = float(z_extrap_m)
        self.z_extrap_slope_max = float(z_extrap_slope_max)
        self.z_soft_guide_max = float(z_soft_guide_max)
        self.z_soft_slope_max = float(z_soft_slope_max)

    def _fill_normal(self, gate):
        n = (gate.get("nx"), gate.get("ny"), gate.get("nz"))
        norm = None
        try:
            norm = math.sqrt(sum(float(c) ** 2 for c in n))
        except (TypeError, ValueError):
            norm = 0.0
        if not norm or norm < 1e-6:
            tx, ty, _ = self.route.tangent(gate["s"])
            gate["nx"], gate["ny"], gate["nz"] = tx, ty, 0.0

    def adjust_gates(self, gates_raw, z_offset=0.0, corrections=None, verified_ids=()):
        verified = set(verified_ids or ())
        corrections = corrections or {}
        gates_adj = []
        for g in gates_raw:
            gg = dict(g)
            off = z_offset if self.gate_z_uses_offset else 0.0
            gg["z"] = float(gg["z"]) + off
            corr = corrections.get(gg.get("id"))
            if corr:
                gg["x"] = float(gg["x"]) + float(corr.get("dx", 0.0))
                gg["y"] = float(gg["y"]) + float(corr.get("dy", 0.0))
                gg["z"] = float(gg["z"]) + float(corr.get("dz", 0.0))
            gg["s"] = self.route.project_gate(gg["x"], gg["y"])
            # Guidance regularization must never move the measured scoring plane.
            gg["measurement_center"] = (gg["x"],gg["y"],gg["z"])
            if self.snap_gate_to_route:
                tx, ty, _ = self.route.point_at(gg["s"])
                gg["x"], gg["y"] = tx, ty
            elif self.gate_center_pull_max >= 0.0:
                tx, ty, _ = self.route.point_at(gg["s"])
                dx, dy = gg["x"] - tx, gg["y"] - ty
                d = math.hypot(dx, dy)
                if self.gate_center_pull_max > 0.0 and d > self.gate_center_pull_max:
                    gg["x"] = tx + dx / d * self.gate_center_pull_max
                    gg["y"] = ty + dy / d * self.gate_center_pull_max
            self._fill_normal(gg)
            if abs(float(gg.get("sigma_z", 0.0))) > self.sigma_z_max:
                gg["z_suspect"] = True
            if gg.get("anchor_class") not in (VERIFIED, HARD, SOFT):
                gg["anchor_class"] = VERIFIED if gg.get("id") in verified else HARD
            gates_adj.append(gg)
        return gates_adj

    def build(self, gates_raw, guides, soft_guides, s_now, p0z, z_offset=0.0,
              corrections=None, verified_ids=(), start_anchor_z=None):
        gates_adj = self.adjust_gates(gates_raw, z_offset=z_offset,
                                      corrections=corrections, verified_ids=verified_ids)
        chain = GateChain(gates_adj, slope_factor=self.slope_factor,
                          slope_abs_max=self.slope_abs_max)
        start_z = start_anchor_z if start_anchor_z is not None else p0z
        if abs(start_z - p0z) > self.startup_z_jump_limit:
            start_z = p0z
        self.height_prior.fit(chain.gates)
        self.trend_horizon = None
        anchors = chain.anchors()
        anchor_gates = [{"s": s, "z": z, "valid": True} for s, z in anchors]
        goal_z = anchors[-1][1] if anchors else start_z
        off = z_offset if self.gate_z_uses_offset else 0.0
        guides_adj = [{"s": gd["s"], "z": float(gd["z"]) + off}
                      for gd in (guides or []) if gd.get("s") is not None]
        trend = [(float(s), float(z)) for s, z in anchors]
        trend += [(g["s"], g["z"]) for g in chain.trend_gates()]
        trend.sort(key=lambda p: p[0])
        if trend and soft_guides and self.z_soft_guide_max > 0.0:
            last_s, last_z = trend[-1]
            for gs, gz in soft_guides:
                if not self.height_prior.consistent(gs,gz+off):
                    continue
                ds = gs - last_s
                if ds < 5.0 or ds > self.z_soft_guide_max:
                    continue
                dz = max(-self.z_soft_slope_max * ds,
                         min(self.z_soft_slope_max * ds, (gz + off) - last_z))
                last_s, last_z = gs, last_z + dz
                guides_adj.append({"s": last_s, "z": last_z})
                trend.append((last_s, last_z))
        self.evidence_horizon = trend[-1][0] if trend else None
        if self.height_prior.valid and trend:
            last_s,last_z=trend[-1]
            self.trend_horizon=self.height_prior.horizon(last_s)
            correction=max(-1.,min(1.,last_z-self.height_prior.center(last_s)))
            s=last_s+10.
            # Keep the height curve valid through braking and the bounded search.
            # A map horizon is a speed constraint, not a command to level the hill.
            while s <= min(self.route.total_s,self.trend_horizon+50.):
                z=self.height_prior.center(s)+correction*max(0.,1.-(s-last_s)/40.)
                guides_adj.append(dict(s=s,z=z))
                goal_z=z
                s+=10.
        elif self.z_extrap_m > 0.0 and len(trend) >= 2:
            (s1, z1), (s0, z0) = trend[-1], trend[-2]
            ds = s1 - s0
            if ds > 1.0:
                slope = (z1 - z0) / ds
                slope = max(-self.z_extrap_slope_max,
                            min(self.z_extrap_slope_max, slope))
                ext_z = z1 + slope * self.z_extrap_m
                guides_adj.append({"s": s1 + self.z_extrap_m, "z": ext_z})
                goal_z = ext_z
        profile = AltitudeProfile(
            0.0, start_z, self.route.total_s, goal_z, anchor_gates, guides_adj,
            corridor_half=self.corridor_half, gate_blend_start=self.gate_blend_start,
            gate_blend_full=self.gate_blend_full, z_rate_max=self.z_rate_max)
        return chain, profile

    def initial_index(self, chain, s_now):
        idx = self.start_gate
        while idx < len(chain.gates) and chain.gates[idx]["s"] <= s_now - 2.0:
            idx += 1
        return idx

    def resolve_index(self, chain, resolved_ids):
        idx = 0
        while idx < len(chain.gates) and chain.gates[idx].get("id") in resolved_ids:
            idx += 1
        return idx
