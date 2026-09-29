#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Safety Supervisor: 只做最后兜底, 不参与日常调速 (方案 6.1, 22.2, 23)。

职责:
  - pose timeout: 超时立即输出零速度并报 POSE_STALE;
  - Z 严重掉队 recovery: |z_err| > 阈值且持续恶化时强制限制水平速度
    (普通 1.0~2.0 m 区间由 SpeedScheduler tracking penalty 处理);
  - STUCK 检测: 使用真实 pose dt, 有速度指令但实际不动 -> ABORT;
  - finalize: 对最终速度/角速度做硬限幅。
"""

import math


class SafetySupervisor(object):

    def __init__(self, pose_timeout=0.3, z_recovery_error=2.0, z_recovery_gain=5.0,
                 stuck_speed=0.4, stuck_time=2.5, stuck_act_speed=0.3,
                 max_speed=12.0, yaw_rate_max=1.0):
        self.pose_timeout = float(pose_timeout)
        self.z_recovery_error = float(z_recovery_error)
        self.z_recovery_gain = float(z_recovery_gain)
        self.stuck_speed = float(stuck_speed)
        self.stuck_time = float(stuck_time)
        self.stuck_act_speed = float(stuck_act_speed)
        self.max_speed = float(max_speed)
        self.yaw_rate_max = float(yaw_rate_max)
        self.stuck_seconds = 0.0
        self.aborted = False

    def reset(self):
        self.stuck_seconds = 0.0
        self.aborted = False

    def pose_stale(self, now, pose_stamp):
        if pose_stamp is None:
            return True
        return (now - pose_stamp) > self.pose_timeout

    def z_recovery(self, v_target, z_err, worsening, pred_worse):
        """返回 (v_target, active)。只在严重且持续恶化时接管水平速度。"""
        if abs(z_err) <= self.z_recovery_error or not (worsening or pred_worse):
            return v_target, False
        v_cap = self.z_recovery_gain * self.z_recovery_error / max(1e-3, abs(z_err))
        return min(v_target, v_cap), True

    def stuck_step(self, dt, v_cmd, prev_pose, pose):
        if self.aborted or prev_pose is None or dt <= 0.0:
            return False
        v_act = math.dist(prev_pose, (pose[0], pose[1], pose[2])) / dt
        if v_cmd > self.stuck_speed and v_act < self.stuck_act_speed:
            self.stuck_seconds += dt
        else:
            self.stuck_seconds = 0.0
        if self.stuck_seconds > self.stuck_time:
            self.aborted = True
        return self.aborted

    def finalize(self, vx, vy, vz, yaw_rate):
        sp = math.hypot(vx, vy)
        if sp > self.max_speed and sp > 1e-9:
            vx *= self.max_speed / sp
            vy *= self.max_speed / sp
        yaw_rate = max(-self.yaw_rate_max, min(self.yaw_rate_max, yaw_rate))
        return vx, vy, vz, yaw_rate
