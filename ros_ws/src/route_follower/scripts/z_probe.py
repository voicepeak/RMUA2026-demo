#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Z 合法高度包络探测 (方案第一部分)。

在固定 XY 上, 以 step 逐档改变 z_ref 并悬停 hold 秒, 记录 pose_gt.z 是否跟随。
若 |z_actual - z_ref| 持续很大, 说明该高度非法/进入异常力场。

用法示例:
  rosrun route_follower z_probe.py _x:=2.6 _y:=0.0 _z_begin:=0.3 _z_end:=-6.0 _step:=-0.5
输出: CSV (时间, z_ref, z_actual, vz, xy_err)
"""

import csv
import os

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped

from airsim_ros.msg import VelCmd


class ZProbe(object):

    def __init__(self):
        self.x = rospy.get_param("~x", 2.6)
        self.y = rospy.get_param("~y", 0.0)
        self.z_begin = rospy.get_param("~z_begin", 0.3)
        self.z_end = rospy.get_param("~z_end", -6.0)
        self.step = rospy.get_param("~step", -0.5)
        self.hold = rospy.get_param("~hold", 3.0)
        self.k_xy = rospy.get_param("~k_xy", 0.8)
        self.k_z = rospy.get_param("~k_z", 0.6)
        self.vmax_xy = rospy.get_param("~vmax_xy", 1.5)
        self.vmax_z = rospy.get_param("~vmax_z", 1.0)
        self.accel = int(rospy.get_param("~accel", 8))
        self.out = rospy.get_param("~out", "/tmp/opencode/z_probe.csv")

        # 生成 z 档位
        self.levels = []
        z = self.z_begin
        if self.step == 0:
            self.levels = [self.z_begin]
        elif self.step > 0:
            while z <= self.z_end + 1e-9:
                self.levels.append(round(z, 3))
                z += self.step
        else:
            while z >= self.z_end - 1e-9:
                self.levels.append(round(z, 3))
                z += self.step

        self.pose = None
        self.yaw = 0.0
        self.t0 = None
        self.rows = []
        self.done = False

        self.pub = rospy.Publisher("/airsim_node/drone_1/vel_body_cmd", VelCmd, queue_size=1)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.cb)
        rospy.Timer(rospy.Duration(0.05), self.loop)
        rospy.loginfo("ZProbe levels(%d): %s  hold=%.1fs", len(self.levels), self.levels, self.hold)

    def cb(self, msg):
        self.pose = msg.pose
        q = msg.pose.orientation
        _, _, self.yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        if self.t0 is None:
            self.t0 = rospy.Time.now()

    def loop(self, _e):
        if self.pose is None:
            return
        if self.rows and not self.done:
            pass
        t = (rospy.Time.now() - self.t0).to_sec()
        total = len(self.levels) * self.hold
        if t >= total:
            if not self.done:
                self.finish()
            return
        idx = min(int(t / self.hold), len(self.levels) - 1)
        z_ref = self.levels[idx]

        p = self.pose.position
        vx = self.k_xy * (self.x - p.x)
        vy = self.k_xy * (self.y - p.y)
        sp = (vx * vx + vy * vy) ** 0.5
        if sp > self.vmax_xy and sp > 1e-6:
            vx *= self.vmax_xy / sp
            vy *= self.vmax_xy / sp
        vz = self.k_z * (p.z - z_ref)          # 机体 z 向上为正
        vz = max(-self.vmax_z, min(self.vmax_z, vz))

        cy, sy = math_cos(self.yaw), math_sin(self.yaw)
        cmd = VelCmd()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "drone_1"
        cmd.vx = cy * vx + sy * vy
        cmd.vy = -sy * vx + cy * vy
        cmd.vz = vz
        cmd.yawRate = 0.0
        cmd.va = self.accel
        cmd.stop = 0
        self.pub.publish(cmd)

        self.rows.append([round(t, 2), z_ref, round(p.z, 3), round(vz, 3),
                          round(((self.x - p.x) ** 2 + (self.y - p.y) ** 2) ** 0.5, 3)])
        rospy.loginfo_throttle(1.0, "z_ref=%.2f z_act=%.2f vz=%.2f" % (z_ref, p.z, vz))

    def finish(self):
        self.done = True
        os.makedirs(os.path.dirname(self.out), exist_ok=True)
        with open(self.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "z_ref", "z_actual", "vz", "xy_err"])
            w.writerows(self.rows)
        # 悬停
        for _ in range(5):
            c = VelCmd()
            c.header.stamp = rospy.Time.now()
            c.va = self.accel
            self.pub.publish(c)
            rospy.sleep(0.05)
        rospy.loginfo("ZProbe done, wrote %s (%d rows)", self.out, len(self.rows))

        # 每档平均
        for i, lv in enumerate(self.levels):
            seg = [r for r in self.rows if r[1] == lv and r[0] >= i * self.hold + self.hold * 0.4]
            if seg:
                avg = sum(r[2] for r in seg) / len(seg)
                rospy.loginfo("LEVEL z_ref=%.2f -> z_avg=%.2f (err=%.2f)", lv, avg, avg - lv)


def math_cos(a):
    import math
    return math.cos(a)


def math_sin(a):
    import math
    return math.sin(a)


if __name__ == "__main__":
    rospy.init_node("z_probe")
    ZProbe()
    rospy.spin()
