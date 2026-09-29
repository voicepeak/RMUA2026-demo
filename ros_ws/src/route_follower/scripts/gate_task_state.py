#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gate Task State: 过门账本 + 真实 Gate plane 交叉判定 (方案 15, 16)。

- 控制索引用 resolved (PASS/MISS/SKIP 都推进, 避免死锁);
- 任务/成绩账本分开: passed / missed / skipped, MISS 绝不等于 PASS;
- 过门判定优先使用 trajectory segment 与 Gate plane 的真实相交 + 门洞局部
  u/v 判断, 不再完全依赖 route progress s;
- 无 orientation 时用 planner 预填的 route tangent 作为近似法向。
"""

import math

import numpy as np

PASS = "PASS"
MISS = "MISS"
SKIP = "SKIP"


def gate_normal(gate, fallback=(1.0, 0.0, 0.0)):
    n = np.array([gate.get("nx", fallback[0]), gate.get("ny", fallback[1]),
                  gate.get("nz", fallback[2])], dtype=float)
    norm = float(np.linalg.norm(n))
    if norm < 1e-6:
        return np.array(fallback, dtype=float)
    return n / norm


def local_axes(n):
    """由门法向构造局部横轴 u / 竖轴 v (NED, v 向上)。"""
    up = np.array([0.0, 0.0, -1.0])
    u = np.cross(up, n)
    if float(np.linalg.norm(u)) < 1e-6:
        u = np.array([1.0, 0.0, 0.0])
    u = u / np.linalg.norm(u)
    v = np.cross(n, u)
    if float(np.linalg.norm(v)) < 1e-6:
        v = np.array([0.0, 0.0, -1.0])
    v = v / np.linalg.norm(v)
    return u, v


class GateTaskState(object):

    def __init__(self, half_width=1.5, half_height=1.5, miss_margin=3.0,
                 skip_s=5.0, miss_radius=6.0):
        self.half_width = float(half_width)
        self.half_height = float(half_height)
        self.miss_margin = float(miss_margin)
        self.skip_s = float(skip_s)
        self.miss_radius = float(miss_radius)
        self.resolved_gate_ids = set()
        self.passed_gate_ids = set()
        self.missed_gate_ids = set()
        self.skipped_gate_ids = set()
        self.last_resolved = None

    def reset(self):
        self.resolved_gate_ids = set()
        self.passed_gate_ids = set()
        self.missed_gate_ids = set()
        self.skipped_gate_ids = set()
        self.last_resolved = None

    def is_resolved(self, gate_id):
        return gate_id in self.resolved_gate_ids

    def half_extents(self, gate):
        hw = self.half_width
        hh = self.half_height
        if hw <= 0.0:
            width = gate.get("width")
            hw = 0.5 * float(width) if width else 1.5
        if hh <= 0.0:
            height = gate.get("height")
            hh = 0.5 * float(height) if height else 1.5
        return hw, hh

    def crossing(self, gate, p0, p1):
        """返回 (u_off, v_off, in_aperture) 或 None。"""
        C = np.asarray(gate.get("measurement_center",
                               (gate["x"],gate["y"],gate["z"])),dtype=float)
        n = gate_normal(gate)
        P0 = np.asarray(p0, dtype=float)
        P1 = np.asarray(p1, dtype=float)
        d0 = float((P0 - C) @ n)
        d1 = float((P1 - C) @ n)
        if abs(d0) < 1e-9 and abs(d1) < 1e-9:
            return None
        if not (d0 < 0.0 <= d1):
            return None
        denom = d0 - d1
        if abs(denom) < 1e-9:
            return None
        a = max(0.0, min(1.0, d0 / denom))
        pc = P0 + a * (P1 - P0)
        u_axis, v_axis = local_axes(n)
        r = pc - C
        u_off = float(r @ u_axis)
        v_off = float(r @ v_axis)
        hw, hh = self.half_extents(gate)
        in_aperture = abs(u_off) <= hw and abs(v_off) <= hh
        if not in_aperture and math.hypot(u_off, v_off) > self.miss_radius:
            return None
        return u_off, v_off, in_aperture

    def _resolve(self, gate, status, index, u_off, v_off):
        gate_id = gate.get("id")
        self.resolved_gate_ids.add(gate_id)
        if status == PASS:
            self.passed_gate_ids.add(gate_id)
        elif status == MISS:
            self.missed_gate_ids.add(gate_id)
        else:
            self.skipped_gate_ids.add(gate_id)
        self.last_resolved = gate
        return {"status": status, "gate_id": gate_id, "gate_index": index,
                "gate_source": gate.get("source", "static_yaml"),
                "lateral_error": u_off, "vertical_error": v_off,
                "gate_s": gate.get("s")}

    def step(self, gates, gate_idx, p0, p1, s_now):
        """推进过门状态; 返回 (gate_idx, events)。"""
        events = []
        while gate_idx < len(gates):
            gate = gates[gate_idx]
            hit = self.crossing(gate, p0, p1)
            if hit is not None:
                u_off, v_off, in_aperture = hit
                status = PASS if in_aperture else MISS
                event = self._resolve(gate, status, gate_idx, u_off, v_off)
                event["basis"] = "gate_plane_crossing"
                events.append(event)
                gate_idx += 1
                continue
            if s_now > gate["s"] + self.miss_margin:
                status = SKIP if (s_now - gate["s"]) >= self.skip_s else MISS
                event = self._resolve(gate, status, gate_idx, None, None)
                event["basis"] = "gate_behind_route_progress"
                events.append(event)
                gate_idx += 1
                continue
            break
        return gate_idx, events

    def summary(self):
        return {"resolved": len(self.resolved_gate_ids),
                "passed": len(self.passed_gate_ids),
                "missed": len(self.missed_gate_ids),
                "skipped": len(self.skipped_gate_ids)}
