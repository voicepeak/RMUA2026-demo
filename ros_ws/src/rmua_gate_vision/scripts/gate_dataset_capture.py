#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO 数据采集: 边飞边存 左/右目成对图像 + pose。

输出目录:
  <out>/raw/<seq>_L.png, <seq>_R.png
  <out>/raw/poses.csv   (seq,t,x,y,z,qx,qy,qz,qw)
用于后续人工标注 (YOLOv8-pose 四角关键点)。
"""

import csv
import os

import cv2
import cv_bridge
import message_filters
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image


class DatasetCapture(object):

    def __init__(self):
        self.out = rospy.get_param("~out", os.path.expanduser("~/rmua_gate_dataset"))
        self.rate_hz = rospy.get_param("~rate_hz", 2.0)
        self.max_imgs = rospy.get_param("~max_imgs", 2000)
        self.raw = os.path.join(self.out, "raw")
        os.makedirs(self.raw, exist_ok=True)
        self.csv_path = os.path.join(self.raw, "poses.csv")
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, "w", newline="") as f:
                csv.writer(f).writerow(["seq", "t", "x", "y", "z", "qx", "qy", "qz", "qw"])
        self.seq = self._next_seq()
        self.pose = None
        self.bridge = cv_bridge.CvBridge()
        self.last = rospy.Time(0)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.pose_cb)
        sl = message_filters.Subscriber("/airsim_node/drone_1/front_left/Scene", Image)
        sr = message_filters.Subscriber("/airsim_node/drone_1/front_right/Scene", Image)
        message_filters.ApproximateTimeSynchronizer([sl, sr], queue_size=5, slop=0.03) \
            .registerCallback(self.cb)
        rospy.loginfo("DatasetCapture -> %s (rate %.1f Hz)", self.raw, self.rate_hz)

    def _next_seq(self):
        n = 0
        for fn in os.listdir(self.raw) if os.path.isdir(self.raw) else []:
            if fn.endswith("_L.png"):
                try:
                    n = max(n, int(fn.split("_")[0]) + 1)
                except ValueError:
                    pass
        return n

    def pose_cb(self, m):
        self.pose = m.pose

    def cb(self, ml, mr):
        if self.pose is None or self.seq >= self.max_imgs:
            return
        now = rospy.Time.now()
        if (now - self.last).to_sec() < 1.0 / self.rate_hz:
            return
        self.last = now
        L = self.bridge.imgmsg_to_cv2(ml, "bgr8")
        R = self.bridge.imgmsg_to_cv2(mr, "bgr8")
        cv2.imwrite(os.path.join(self.raw, "%06d_L.png" % self.seq), L)
        cv2.imwrite(os.path.join(self.raw, "%06d_R.png" % self.seq), R)
        p = self.pose.position
        q = self.pose.orientation
        with open(self.csv_path, "a", newline="") as f:
            csv.writer(f).writerow([self.seq, "%.4f" % now.to_sec(),
                                    "%.4f" % p.x, "%.4f" % p.y, "%.4f" % p.z,
                                    "%.5f" % q.x, "%.5f" % q.y, "%.5f" % q.z, "%.5f" % q.w])
        self.seq += 1
        rospy.loginfo_throttle(5.0, "saved %d images", self.seq)


if __name__ == "__main__":
    rospy.init_node("gate_dataset_capture")
    DatasetCapture()
    rospy.spin()
