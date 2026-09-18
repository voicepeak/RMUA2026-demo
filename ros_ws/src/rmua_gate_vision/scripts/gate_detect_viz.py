#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双目 + OpenCV Gate 检测可视化节点。

订阅 front_left / front_right (同步), 在左右图跑 OpenCV Gate 检测,
发布:
  /rmua/gate_detection/left    (左图 + 检测框)
  /rmua/gate_detection/right   (右图 + 检测框)
  /rmua/gate_detection/stereo  (左右拼接)
用于 rqt_image_view / image_view 直接查看。
"""

import os
import sys

import cv2
import cv_bridge
import message_filters
import rospy
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gate_detector_opencv import detect_gates, draw  # noqa: E402


class GateDetectViz(object):

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.params = {
            "white_s_max": rospy.get_param("~white_s_max", 50),
            "white_v_min": rospy.get_param("~white_v_min", 140),
            "min_area": rospy.get_param("~min_area", 800),
        }
        self.frame_id = rospy.get_param("~frame_id", "drone_1")
        self.pub_l = rospy.Publisher("/rmua/gate_detection/left", Image, queue_size=2)
        self.pub_r = rospy.Publisher("/rmua/gate_detection/right", Image, queue_size=2)
        self.pub_s = rospy.Publisher("/rmua/gate_detection/stereo", Image, queue_size=2)
        sl = message_filters.Subscriber("/airsim_node/drone_1/front_left/Scene", Image)
        sr = message_filters.Subscriber("/airsim_node/drone_1/front_right/Scene", Image)
        message_filters.ApproximateTimeSynchronizer([sl, sr], queue_size=5, slop=0.03) \
            .registerCallback(self.cb)
        rospy.loginfo("GateDetectViz ready (OpenCV gate detector)")

    def cb(self, ml, mr):
        left = self.bridge.imgmsg_to_cv2(ml, "bgr8")
        right = self.bridge.imgmsg_to_cv2(mr, "bgr8")
        gl = detect_gates(left, self.params)
        gr = detect_gates(right, self.params)
        vl = draw(left, gl)
        vr = draw(right, gr)
        st = cv2.hconcat([vl, vr])
        cv2.putText(st, "LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.putText(st, "RIGHT", (left.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 255, 255), 2)
        now = ml.header.stamp
        for pub, img in ((self.pub_l, vl), (self.pub_r, vr), (self.pub_s, st)):
            m = self.to_imgmsg(img, now)
            pub.publish(m)
        rospy.loginfo_throttle(2.0, "gates L=%d R=%d", len(gl), len(gr))

    def to_imgmsg(self, bgr, stamp):
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.frame_id
        img.height, img.width = bgr.shape[:2]
        img.encoding = "bgr8"
        img.is_bigendian = 0
        img.step = bgr.shape[1] * 3
        img.data = bgr.tobytes()
        return img


if __name__ == "__main__":
    rospy.init_node("gate_detect_viz")
    GateDetectViz()
    rospy.spin()
