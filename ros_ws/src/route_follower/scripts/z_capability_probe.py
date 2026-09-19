#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Z Capability Probe: 实测垂直能力 vz_available(vxy) (方案 13~18, 42~43 节)。

对每个 (vx, vz) 档位:
  ramp_in(1s) -> stable(2~3s) -> ramp_out(1s)
在 stable 段取 pose_gt.z 线性拟合, 实际上升速度 = -dz/dt (NED z 向下为正)。

输出:
  <out>.csv      逐帧记录
  <out>_cap.csv  每档位汇总 (vxy, vz_cmd, actual_climb)
可直接整理成 vz_capability.yaml 给 route_follower 使用。

用法:
  rosrun route_follower z_capability_probe.py _vx_levels:="[0,5,10]" _vz_levels:="[1,2,3,4,5]"
"""

import csv
import os

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

from airsim_ros.msg import VelCmd


def parse_list(s):
    if isinstance(s, (list, tuple)):
        return [float(x) for x in s]
    return [float(x) for x in str(s).replace("[", "").replace("]", "").split(",") if x.strip() != ""]


class ZCapabilityProbe(object):

    def __init__(self):
        self.vx_levels = parse_list(rospy.get_param("~vx_levels", [0.0, 5.0, 10.0]))
        self.vz_levels = parse_list(rospy.get_param("~vz_levels", [1.0, 2.0, 3.0, 4.0, 5.0]))
        self.ramp = rospy.get_param("~ramp", 1.0)
        self.hold = rospy.get_param("~hold", 2.5)
        self.settle = rospy.get_param("~settle", 0.5)
        self.accel = int(rospy.get_param("~accel", 8))
        self.out = rospy.get_param("~out", "/tmp/opencode/z_capability.csv")
        # 先向前飞离起点区域, 避免起点结构/门框挡住垂直测试
        self.warmup_vx = rospy.get_param("~warmup_vx", 5.0)
        self.warmup_time = rospy.get_param("~warmup_time", 6.0)

        # 生成档位序列: (phase, vx, vz, duration)
        self.seq = []
        for vx in self.vx_levels:
            for vz in self.vz_levels:
                self.seq.append(("ramp_in", vx, vz, self.ramp))
                self.seq.append(("stable", vx, vz, self.hold))
                self.seq.append(("ramp_out", vx, vz, self.ramp))
        self.total = self.warmup_time + sum(s[3] for s in self.seq)

        self.pose = None
        self.yaw = 0.0
        self.t0 = None
        self.rows = []
        self.cur_level = None      # (vx, vz)
        self.samples = []          # (t, z) during stable
        self.summary = []
        self.done = False

        self.pub = rospy.Publisher("/airsim_node/drone_1/vel_body_cmd", VelCmd, queue_size=1)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.cb)
        rospy.Timer(rospy.Duration(0.02), self.loop)
        rospy.loginfo("ZCapabilityProbe: vx=%s vz=%s total=%.1fs", self.vx_levels,
                      self.vz_levels, self.total)

    def cb(self, m):
        self.pose = m.pose

    def _publish(self, vx, vz):
        c = VelCmd()
        c.header.stamp = rospy.Time.now()
        c.header.frame_id = "drone_1"
        c.vx, c.vy, c.vz = vx, 0.0, vz
        c.yawRate = 0.0
        c.va = self.accel
        c.stop = 0
        self.pub.publish(c)

    def loop(self, _e):
        if self.pose is None or self.done:
            return
        if self.t0 is None:
            self.t0 = rospy.Time.now()
        t = (rospy.Time.now() - self.t0).to_sec()
        if t >= self.total:
            self.finish()
            return
        if t < self.warmup_time:                 # 前飞离场
            self._publish(self.warmup_vx, 0.0)
            rospy.loginfo_throttle(1.0, "ZCAP warmup vx=%.1f t=%.1f", self.warmup_vx, t)
            return

        # 定位当前 phase 与档位 (vx, vz)
        acc = 0.0
        tq = t - self.warmup_time
        phase = vx = vz = None
        frac = 0.0
        for (ph, lvx, lvz, dur) in self.seq:
            if tq < acc + dur:
                phase, vx, vz, frac = ph, lvx, lvz, (tq - acc) / dur
                break
            acc += dur
        else:
            self.finish()
            return

        key = (vx, vz)
        if key != self.cur_level:            # 档位切换 -> 结算上一档
            self._finish_level()
            self.cur_level = key
            self.samples = []

        if phase == "ramp_in":
            self._publish(vx * frac, vz * frac)
        elif phase == "ramp_out":
            self._publish(vx * (1.0 - frac), vz * (1.0 - frac))
        else:
            self._publish(vx, vz)

        z = self.pose.position.z
        self.rows.append([round(t, 3), phase, vx, vz, round(z, 4)])
        if phase == "stable" and (tq - acc) >= self.settle:
            self.samples.append((t, z))      # stable 稳定段采样

        rospy.loginfo_throttle(1.0, "ZCAP %s vx=%.1f vz=%.1f z=%.2f", phase, vx, vz, z)

    def _finish_level(self):
        if self.cur_level is None or len(self.samples) < 5:
            self.samples = []
            return
        tt = np.array([s[0] for s in self.samples])
        zz = np.array([s[1] for s in self.samples])
        climb = -np.polyfit(tt, zz, 1)[0]     # dz/dt (NED) -> 上升为正
        vx, vz = self.cur_level
        self.summary.append((vx, vz, float(climb)))
        rospy.loginfo("ZCAP vx=%.1f vz_cmd=%.1f -> actual_climb=%.2f m/s", vx, vz, climb)
        self.samples = []

    def finish(self):
        self.done = True
        self._finish_level()
        d = os.path.dirname(self.out)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(self.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "phase", "vx_cmd", "vz_cmd", "z"])
            w.writerows(self.rows)
        cap_path = os.path.splitext(self.out)[0] + "_cap.csv"
        with open(cap_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["vxy", "vz_cmd", "actual_climb"])
            w.writerows(self.summary)
        # 每档位取最大实际爬升作为 vz_available
        best = {}
        for vx, vz, climb in self.summary:
            best[vx] = max(best.get(vx, 0.0), climb)
        yml = os.path.splitext(self.out)[0] + "_capability.yaml"
        with open(yml, "w") as f:
            f.write("vz_available:\n")
            for vx in sorted(best):
                f.write("  - vxy: %.1f\n    vz_up: %.2f\n" % (vx, best[vx]))
        # 悬停
        for _ in range(25):
            self._publish(0.0, 0.0)
            rospy.sleep(0.02)
        rospy.loginfo("ZCapabilityProbe done. csv=%s cap=%s yaml=%s", self.out, cap_path, yml)
        rospy.signal_shutdown("probe done")


if __name__ == "__main__":
    rospy.init_node("z_capability_probe")
    ZCapabilityProbe()
    rospy.spin()
