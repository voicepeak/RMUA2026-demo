#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gate Chain: 顺序约束 + Z 坡度离群剔除 + Soft/Hard Anchor。

- 按 path progress s 单调排序
- 剔除 s <= current_s 的已过门
- 检查相邻门的 Δz/Δs, 明显偏离邻近中位坡度则标记 z_suspect (不参与 Z 曲线)
- 距离近且稳定的门为 Hard Anchor, 远的为 Soft Anchor
"""

import numpy as np


class GateChain(object):

    def __init__(self, gates, slope_factor=3.0, slope_abs_max=0.6,
                 hard_dist=25.0):
        self.slope_factor = slope_factor
        self.slope_abs_max = slope_abs_max
        self.hard_dist = hard_dist
        # 仅保留 valid
        gs = [dict(g) for g in gates if g.get("valid", True)]
        gs.sort(key=lambda g: g["s"])
        self.gates = gs
        self._score_slopes()

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
            g["z_suspect"] = abs(loc) > self.slope_abs_max
        # 邻域中位坡度对比
        for i, g in enumerate(self.gates):
            lo, hi = max(0, i - 2), min(n, i + 3)
            near = [abs(slopes[j]) for j in range(lo, min(hi, n - 1))]
            if near:
                med = float(np.median(near))
                g["z_suspect"] = g["z_suspect"] or (abs(g["slope"]) > self.slope_factor * max(med, 0.05))

    def anchors(self):
        """参与 Z 曲线的门 (剔除 z_suspect)。"""
        return [(g["s"], g["z"]) for g in self.gates if not g["z_suspect"]]

    def suspect_ids(self):
        return [g["id"] for g in self.gates if g["z_suspect"]]
