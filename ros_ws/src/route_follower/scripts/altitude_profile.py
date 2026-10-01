#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Altitude Profile: 由 Gate/Guide 锚点插值出的赛道高度几何 + 平滑切换。

方案职责划分 (文档第 12/33 节):
  - AltitudeProfile 只负责几何: z_center(s) / dz_ds(s) / corridor(s);
  - 不再做 "靠近下一门时再融合一次 gate.z" 的重复控制 (若门已是锚点,
    center(s) 已包含它);
  - z_ref 的变化率限制与反馈/前馈由 ZController 负责;
  - GateMap 在线刷新时用 ProfileBlender 在 profile_switch_time 内平滑过渡
    旧/新 profile, 避免 z_ref 与 dz/ds 同时跳变。
"""


from spatial_curve import SpatialCurve
import math

def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return 3.0 * t * t - 2.0 * t * t * t


class AltitudeProfile(object):

    def __init__(self, start_s, start_z, goal_s, goal_z, gates, guides,
                 corridor_half=1.5, gate_blend_start=20.0, gate_blend_full=6.0,
                 z_rate_max=1.0, gate_z_max_jump=15.0, corridor_guidance=False):
        self.corridor_half = corridor_half
        self.gate_blend_start = gate_blend_start
        self.gate_blend_full = gate_blend_full
        self.z_rate_max = z_rate_max
        self.gate_z_max_jump = gate_z_max_jump

        gate_anchors = [(float(g["s"]), float(g["z"]))
                        for g in gates if g.get("valid", False) and g.get("s") is not None]
        anchors = [(float(start_s), float(start_z))] + gate_anchors
        for gd in (guides or []):
            if gd.get("s") is None:
                continue
            gs, gz = float(gd["s"]), float(gd["z"])
            if any(abs(gs - s) < 1.0 for s, _ in gate_anchors):
                continue
            anchors.append((gs, gz))
        anchors.append((float(goal_s), float(goal_z)))
        anchors.sort(key=lambda a: a[0])
        self.anchors = anchors
        self.curve = SpatialCurve(anchors)
        if corridor_guidance and len(guides or [])>=2:
            # Recorded road height and a gate center are different constraints.
            # Interleaving both as exact anchors a few metres apart makes the
            # slope reverse at each gate, even on a continuously descending road.
            lo=min(float(g['s']) for g in guides);hi=max(float(g['s']) for g in guides)
            road=[(float(start_s),float(start_z))]
            road += [(s,z) for s,z in gate_anchors if s<lo or s>hi]
            road += [(float(g['s']),float(g['z'])) for g in guides]
            road += [(float(goal_s),float(goal_z))]
            nominal=SpatialCurve(road)
            stations=sorted(set([float(start_s),float(goal_s)]+[s for s,z in gate_anchors]+
                               [float(start_s)+2.*i for i in range(int((goal_s-start_s)/2.)+1)]))
            corrections=[]
            for gs,gz in gate_anchors:
                if not lo<=gs<=hi:continue
                value=nominal.center(gs)+sum(d*math.exp(-((gs-c)/12.)**2) for c,d in corrections)
                target=max(gz-.85,min(gz+.85,value))
                if abs(target-value)>1e-4:corrections.append((gs,target-value))
            self.anchors=[(s,nominal.center(s)+sum(d*math.exp(-((s-c)/12.)**2) for c,d in corrections)) for s in stations]
            self.curve=SpatialCurve(self.anchors)

    def center(self, s):
        return self.curve.center(s)

    def corridor(self, s):
        c = self.center(s)
        return c - self.corridor_half, c + self.corridor_half   # (ceiling, floor)

    def anchor_pair(self, s):
        """返回包住 s 的两个锚点 (s0,z0),(s1,z1)。"""
        a = self.anchors
        for i in range(len(a) - 1):
            if a[i][0] <= s <= a[i + 1][0]:
                return a[i], a[i + 1]
        return a[-1], a[-1]

    @staticmethod
    def horizon_s(s, s_now, horizon):
        """把查询点限制在 [s_now, s_now+horizon] 内 (方案 17 节: Z 外推有 Horizon)。"""
        if horizon is None or horizon <= 0.0:
            return s
        return max(s_now, min(s, s_now + horizon))

    def dz_ds(self, s, h=4.0):
        return self.curve.dz_ds(s)


class FrozenBlend:
    """Flatten an interrupted transition so successive map updates stay continuous."""
    def __init__(self, previous, current, beta):
        components = getattr(previous, 'components', [(1., previous)])
        self.components = [(w*(1-beta), p) for w,p in components if w*(1-beta)>1e-8]
        self.components.append((beta,current))
        total = sum(w for w,p in self.components)
        self.components = [(w/total,p) for w,p in self.components]
    def center(self,s): return sum(w*p.center(s) for w,p in self.components)
    def dz_ds(self,s,h=4.): return sum(w*p.dz_ds(s,h) for w,p in self.components)


class ProfileBlender(object):
    """在线重建 profile 时, 在 switch_time 秒内做 smoothstep 混合 (方案 12.2)。"""

    def __init__(self, switch_time=0.4):
        self.switch_time = max(1e-3, float(switch_time))
        self.current = None
        self.previous = None
        self.switch_stamp = None

    def set_initial(self, profile, stamp=None):
        self.current = profile
        self.previous = None
        self.switch_stamp = None

    def switch(self, profile, stamp):
        if self.current is None:
            self.set_initial(profile, stamp)
            return
        beta = self.beta(stamp)
        self.previous = (FrozenBlend(self.previous, self.current, beta)
                         if self.previous is not None and beta < 1. else self.current)
        self.current = profile
        self.switch_stamp = stamp

    def beta(self, stamp):
        if self.previous is None or self.switch_stamp is None or stamp is None:
            return 1.0
        return smoothstep(min(1.0, max(0.0, (stamp - self.switch_stamp) / self.switch_time)))

    def is_switching(self, stamp):
        return self.previous is not None and self.beta(stamp) < 1.0

    def center(self, s, stamp=None):
        z = self.current.center(s)
        b = self.beta(stamp)
        if b < 1.0:
            z = (1.0 - b) * self.previous.center(s) + b * z
        return z

    def dz_ds(self, s, h=4.0, stamp=None):
        z = self.current.dz_ds(s, h)
        b = self.beta(stamp)
        if b < 1.0:
            z = (1.0 - b) * self.previous.dz_ds(s, h) + b * z
        return z

    def corridor(self, s):
        return self.current.corridor(s)

    def anchor_pair(self, s):
        return self.current.anchor_pair(s)

    def horizon_s(self, s, s_now, horizon):
        return AltitudeProfile.horizon_s(s, s_now, horizon)
