#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双目 OpenCV Gate 识别 -> 世界系 XYZ 采集节点。

订阅: front_left/Scene, front_right/Scene (同步), pose_gt
发布: /rmua/gate_observations  (std_msgs/String, JSON)
服务: ~save (保存稳定 Gate 到 YAML), ~clear

多帧稳定: 世界系观测贪心聚类, 支持帧数 >= min_support 才保存 (取中位数)。
"""

import json
import math
import os
import sys

import cv2
import cv_bridge
import message_filters
import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gate_detector_opencv import detect_gates, gate_frame_mask, draw  # noqa: E402
import gate_stereo as gs  # noqa: E402
from gate_map import GateMap  # noqa: E402


class GateVision(object):

    def __init__(self):
        cfg = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "config"))
        self.route_file = rospy.get_param("~route_file",
                                          os.path.join(cfg, "route_1_3.yaml"))
        self.route_name = rospy.get_param("~route_name", "route_1_3")
        self.out_file = rospy.get_param("~out_file", os.path.join(cfg, "gates_vision_1_3.yaml"))
        self.cluster_radius = rospy.get_param("~cluster_radius", 3.0)
        self.min_support = rospy.get_param("~min_support", 4)
        self.max_d_cross = rospy.get_param("~max_d_cross", 5.0)
        self.save_images = bool(rospy.get_param("~save_images", False))
        self.img_dir = rospy.get_param("~img_dir", "/tmp/opencode/gate_imgs")

        self.detector_params = {
            "white_s_max": rospy.get_param("~white_s_max", 50),
            "white_v_min": rospy.get_param("~white_v_min", 140),
            "min_area": rospy.get_param("~min_area", 800),
        }
        self.pose = None
        self.bridge = cv_bridge.CvBridge()
        self.sgbm = gs.build_sgbm()
        self.pub = rospy.Publisher("/rmua/gate_observations", String, queue_size=5)
        self.pub_map = rospy.Publisher("/rmua/gate_map", String, queue_size=2)

        self.route = self.load_route(self.route_file, self.route_name)
        self.build_route_s()
        self.gate_map = GateMap(self.route, assoc_radius=self.cluster_radius)

        rospy.Service("~save", Trigger, self.save)
        rospy.Service("~clear", Trigger, self.clear)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.pose_cb)
        sl = message_filters.Subscriber("/airsim_node/drone_1/front_left/Scene", Image)
        sr = message_filters.Subscriber("/airsim_node/drone_1/front_right/Scene", Image)
        self.sync = message_filters.ApproximateTimeSynchronizer([sl, sr], queue_size=5, slop=0.03)
        self.sync.registerCallback(self.pair_cb)
        if self.save_images:
            os.makedirs(self.img_dir, exist_ok=True)
        rospy.Timer(rospy.Duration(0.25), self.publish_map)
        rospy.loginfo("GateVision ready: route=%s out=%s", self.route_name, self.out_file)

    # ---- route ----
    @staticmethod
    def load_route(path, name):
        with open(path) as f:
            pts = yaml.safe_load(f)["routes"][name]
        return np.array(pts, dtype=np.float64)

    def build_route_s(self):
        r = self.route
        self.seg_len = np.hypot(np.diff(r[:, 0]), np.diff(r[:, 1]))
        self.seg_s = np.concatenate([[0.0], np.cumsum(self.seg_len)])

    # ---- callbacks ----
    def pose_cb(self, m):
        self.pose = m.pose

    def pair_cb(self, ml, mr):
        if self.pose is None:
            return
        left = self.bridge.imgmsg_to_cv2(ml, "bgr8")
        right = self.bridge.imgmsg_to_cv2(mr, "bgr8")
        cl = detect_gates(left, self.detector_params)
        if not cl:
            return
        disp = gs.compute_disparity(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY),
                                    cv2.cvtColor(right, cv2.COLOR_BGR2GRAY), self.sgbm)
        fmask = gate_frame_mask(left, self.detector_params)
        px = self.pose.position
        q = self.pose.orientation
        pos = np.array([px.x, px.y, px.z])
        quat = (q.x, q.y, q.z, q.w)
        now_stamp = ml.header.stamp.to_sec()
        obs_json = []
        for cand in cl:
            res = gs.gate_center_from_frame(cand, disp, fmask)
            if res is None:
                continue
            center_cam, depth, npx = res
            if depth < 1.0 or depth > 200.0:
                continue
            world = gs.body_to_world(gs.camera_to_body(center_cam), pos, quat)
            if not np.all(np.isfinite(world)):
                continue
            t = self.gate_map.observe(world, now_stamp, depth)
            obs_json.append({"world": world.tolist(), "depth": float(depth),
                             "npix": npx, "conf": float(cand["confidence"]),
                             "track_id": int(t.id), "sigma_z": float(t.sigma()[2])})
        self.pub.publish(String(data=json.dumps(obs_json)))
        if self.save_images and obs_json:
            cv2.imwrite(os.path.join(self.img_dir, "L_%d.jpg" % ml.header.stamp.to_nsec()),
                        draw(left, cl))
        self.gate_map.prune(now_stamp)
        rospy.loginfo_throttle(
            2.0, "obs=%d tracks=%d map=%s", len(obs_json),
            len(self.gate_map.tracker.tracks),
            [(g["id"], round(g["s"], 1), round(g["z"], 2)) for g in self.gate_map.gates(2)])

    # ---- save ----
    def publish_map(self, _e=None):
        self.pub_map.publish(String(data=self.gate_map.to_json(min_support=self.min_support)))

    def save(self, _req):
        n = self.gate_map.save_yaml(self.out_file, min_support=self.min_support,
                                    source="opencv_stereo", max_d_cross=self.max_d_cross)
        rospy.loginfo("saved %d gates to %s", n, self.out_file)
        return TriggerResponse(success=True, message="saved %d gates" % n)

    def clear(self, _req):
        n = len(self.gate_map.tracker.tracks)
        self.gate_map = GateMap(self.route, assoc_radius=self.cluster_radius)
        return TriggerResponse(success=True, message="cleared %d tracks" % n)


if __name__ == "__main__":
    rospy.init_node("gate_vision")
    GateVision()
    rospy.spin()
