#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Z Controller: 正常 TRACK 下唯一的 vz authority = FF + FB (方案 9, 34)。

    z_ref        <- AltitudeProfile / ProfileBlender (含 z_rate_max 限幅)
    z_err = z - z_ref
    vz_fb = k_z * z_err
    vz_ff = -k_ff_z * dz/ds_preview * v
    vz_target = vz_ff + vz_fb
    vz_clamped = clamp(vz_target, -vz_down_limit, vz_up_limit)
    vz_cmd = accel_limit(prev_vz, vz_clamped)

RECON 是独立 mode, 用常量 recon_climb 替代 FF+FB, 不叠加。

k_anticipate / z_anticipate_dist 仅为 A/B 对照保留 (方案 2.1/27):
默认 0.0 = 关闭, 正常控制不再包含 time-to-gate -> vz override。
"""


class ZController(object):

    def __init__(self, k_z=1.0, k_ff_z=1.2, z_rate_max=4.0, vz_up_limit=4.0,
                 vz_down_limit=3.0, vz_accel_limit=6.0,
                 k_anticipate=0.0, z_anticipate_dist=25.0):
        self.k_z = float(k_z)
        self.k_ff_z = float(k_ff_z)
        self.z_rate_max = float(z_rate_max)
        self.vz_up_limit = float(vz_up_limit)
        self.vz_down_limit = float(vz_down_limit)
        self.vz_accel_limit = float(vz_accel_limit)
        self.k_anticipate = float(k_anticipate)
        self.z_anticipate_dist = float(z_anticipate_dist)
        self.prev_z_ref = None
        self.prev_vz = 0.0

    def reset(self):
        self.prev_z_ref = None
        self.prev_vz = 0.0

    def reference(self, center_fn, s_now, dt):
        """z_ref = center(s_now), 并做 z_rate_max 变化率限制 (方案 40)。"""
        z = float(center_fn(s_now))
        limited = False
        if self.prev_z_ref is not None and dt > 0.0:
            md = self.z_rate_max * dt
            if z > self.prev_z_ref + md:
                z = self.prev_z_ref + md
                limited = True
            elif z < self.prev_z_ref - md:
                z = self.prev_z_ref - md
                limited = True
        self.prev_z_ref = z
        return z, limited

    def _clamp_accel(self, vz_target, dt, up_limit=None, down_limit=None):
        up = self.vz_up_limit if up_limit is None else up_limit
        down = self.vz_down_limit if down_limit is None else down_limit
        vz_clamped = max(-down, min(up, vz_target))
        step = self.vz_accel_limit * max(0.0, dt)
        dvz = max(-step, min(step, vz_clamped - self.prev_vz))
        vz = self.prev_vz + dvz
        self.prev_vz = vz
        if abs(vz_clamped - vz_target) > 1e-4:
            limit = "VZ_UP_LIMIT" if vz_target > vz_clamped else "VZ_DOWN_LIMIT"
        elif abs(vz - vz_clamped) > 1e-4:
            limit = "VZ_ACCEL"
        else:
            limit = "NONE"
        return vz_clamped, vz, limit

    def track(self, z_ref, z_actual, dzds_preview, v, dt, next_gate=None, d_gate=1e9):
        z_err = z_actual - z_ref
        vz_fb = self.k_z * z_err
        vz_ff = -self.k_ff_z * dzds_preview * v
        vz_target = vz_fb + vz_ff
        if self.k_anticipate > 0.0 and next_gate is not None \
                and 0.5 < d_gate < self.z_anticipate_dist:
            t_gate = max(d_gate / max(v, 0.6), dt)
            need_up = (z_actual - float(next_gate["z"])) / t_gate
            if need_up > max(0.0, vz_target):
                vz_target = min(self.k_anticipate * need_up, self.vz_up_limit)
        vz_clamped, vz, limit = self._clamp_accel(vz_target, dt)
        return {"z_ref": z_ref, "z_err": z_err, "vz_fb": vz_fb, "vz_ff": vz_ff,
                "vz_target": vz_target, "vz_clamped": vz_clamped, "vz_cmd": vz,
                "z_limit": limit, "anticipate_active": self.k_anticipate > 0.0}

    def recon(self, z_actual, recon_start_z, recon_max_climb, recon_climb, dt):
        """RECON 独立 mode: 常量缓慢爬升, 到上限后保持。"""
        climbing = (recon_start_z is None
                    or z_actual > recon_start_z - recon_max_climb)
        vz_target = recon_climb if climbing else 0.0
        vz_clamped, vz, limit = self._clamp_accel(vz_target, dt)
        return {"z_ref": z_actual, "z_err": 0.0, "vz_fb": 0.0, "vz_ff": 0.0,
                "vz_target": vz_target, "vz_clamped": vz_clamped, "vz_cmd": vz,
                "z_limit": limit, "recon_climbing": climbing}

    def hold(self, dt):
        vz_clamped, vz, limit = self._clamp_accel(0.0, dt)
        return {"z_ref": self.prev_z_ref if self.prev_z_ref is not None else 0.0,
                "z_err": 0.0, "vz_fb": 0.0, "vz_ff": 0.0, "vz_target": 0.0,
                "vz_clamped": vz_clamped, "vz_cmd": vz, "z_limit": limit}
