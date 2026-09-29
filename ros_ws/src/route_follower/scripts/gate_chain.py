#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gate Chain: 顺序约束 + 坡度异常标记 + Anchor 分级 (方案 11)。

- 按 path progress s 单调排序
- 计算相邻门坡度并标记 z_suspect (高度可信度下降), 但**不再直接删除可靠门**
- Anchor 分级:
    VERIFIED (静态/已验证门): 永远保留为高度锚点, 坡度大只触发降速;
    HARD     (在线硬锚点): 非严重异常 (z_suspect=False) 保留为锚点;
    SOFT     (在线软门):    只作为高度趋势 guide, 不作为强锚点。
"""

import numpy as np

VERIFIED = "VERIFIED"
HARD = "HARD"
SOFT = "SOFT"


class GateChain(object):

    def __init__(self, gates, slope_factor=3.0, slope_abs_max=0.6,
                 hard_dist=25.0):
        self.slope_factor = slope_factor
        self.slope_abs_max = slope_abs_max
        self.hard_dist = hard_dist
        gs = [dict(g) for g in gates if g.get("valid", True)]
        gs.sort(key=lambda g: g["s"])
        self.gates = gs
        self._score_slopes()
        self._classify()

    def _score_slopes(self):
        n = len(self.gates)
        slopes = []
        for i in range(n - 1):
            ds = self.gates[i + 1]["s"] - self.gates[i]["s"]
            dz = self.gates[i + 1]["z"] - self.gates[i]["z"]
            slopes.append(dz / ds if ds > 1e-6 else 0.0)
        for i, g in enumerate(self.gates):
            if i == 0:
                loc = slopes[0] if slopes else 0.0
            elif i == n - 1:
                loc = slopes[-1] if slopes else 0.0
            else:
                loc = 0.5 * (slopes[i - 1] + slopes[i])
            g["slope"] = loc
            g["z_suspect"] = g.get("z_suspect", False) or abs(loc) > self.slope_abs_max
        for i, g in enumerate(self.gates):
            lo, hi = max(0, i - 2), min(n, i + 3)
            near = [abs(slopes[j]) for j in range(lo, min(hi, n - 1))]
            if near:
                med = float(np.median(near))
                g["z_suspect"] = g["z_suspect"] or (abs(g["slope"]) > self.slope_factor * max(med, 0.05))

    def _classify(self):
        for g in self.gates:
            cls = g.get("anchor_class")
            if cls not in (VERIFIED, HARD, SOFT):
                source = str(g.get("source", "")).lower()
                if g.get("verified") or "static" in source or not g.get("hard_anchor", True):
                    cls = VERIFIED if (g.get("verified") or "static" in source) else SOFT
                else:
                    cls = HARD
            g["anchor_class"] = cls
            g["trusted"] = cls == VERIFIED or (cls == HARD and not g.get("z_suspect", False))

    def anchors(self):
        """参与 Z 曲线的门 (VERIFIED 永远保留; HARD 非严重异常保留)。"""
        return [(g["s"], g["z"]) for g in self.gates if g["trusted"]]

    def trend_gates(self):
        """只作为高度趋势的门 (SOFT 或失去硬锚资格的 HARD)。"""
        return [g for g in self.gates if not g["trusted"]]

    def suspect_ids(self):
        return [g["id"] for g in self.gates if g.get("z_suspect", False)]
