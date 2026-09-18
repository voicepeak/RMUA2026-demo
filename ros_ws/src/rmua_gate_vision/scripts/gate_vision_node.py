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


class GateVision(object):

    def __init__(self):
        cfg = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "config"))
        self.route_file = rospy.get_param("~route_file",
                                          os.path.join(cfg, "route_1_3.yaml"))
        self.route_name = rospy.get_param("~route_name", "route_1_3")
        self.out_file = rospy.get_param("~out_file", os.path.join(cfg, "gates_vision_1_3.yaml"))
        self.cluster_radius = rospy.get_param("~cluster_radius", 3.0)
        self.min_support = rospy.get_param("~min_support", 8)
        self.max_d_cross = rospy.get_param("~max_d_cross", 5.0)
        self.save_images = bool(rospy.get_param("~save_images", False))
        self.img_dir = rospy.get_param("~img_dir", "/tmp/opencode/gate_imgs")

        self.detector_params = {
            "white_s_max": rospy.get_param("~white_s_max", 50),
            "white_v_min": rospy.get_param("~white_v_min", 140),
            "min_area": rospy.get_param("~min_area", 800),
        }
        self.pose = None
        self.obs = []            # list of world(3) np
        self.bridge = cv_bridge.CvBridge()
        self.sgbm = gs.build_sgbm()
        self.pub = rospy.Publisher("/rmua/gate_observations", String, queue_size=5)

        self.route = self.load_route(self.route_file, self.route_name)
        self.build_route_s()

        rospy.Service("~save", Trigger, self.save)
        rospy.Service("~clear", Trigger, self.clear)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.pose_cb)
        sl = message_filters.Subscriber("/airsim_node/drone_1/front_left/Scene", Image)
        sr = message_filters.Subscriber("/airsim_node/drone_1/front_right/Scene", Image)
        self.sync = message_filters.ApproximateTimeSynchronizer([sl, sr], queue_size=5, slop=0.03)
        self.sync.registerCallback(self.pair_cb)
        if self.save_images:
            os.makedirs(self.img_dir, exist_ok=True)
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

    def project_route(self, xy):
        best_d, best_s = 1e9, 0.0
        for i in range(len(self.route) - 1):
            a, b = self.route[i], self.route[i + 1]
            d = b[:2] - a[:2]
            L2 = d @ d
            t = 0.0 if L2 < 1e-9 else float(np.clip((xy - a[:2]) @ d / L2, 0, 1))
            c = a[:2] + t * d
            dd = float(np.linalg.norm(xy - c))
            if dd < best_d:
                best_d, best_s = dd, self.seg_s[i] + t * self.seg_len[i]
        return best_s, best_d

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
            self.obs.append(world.copy())
            obs_json.append({"world": world.tolist(), "depth": float(depth),
                             "npix": npx, "conf": float(cand["confidence"])})
        self.pub.publish(String(data=json.dumps(obs_json)))
        if self.save_images and obs_json:
            cv2.imwrite(os.path.join(self.img_dir, "L_%d.jpg" % ml.header.stamp.to_nsec()),
                        draw(left, cl))
        rospy.loginfo_throttle(2.0, "obs total=%d, this=%d", len(self.obs), len(obs_json))

    def route_tangent(self, s):
        for i in range(len(self.route) - 1):
            if self.seg_s[i] <= s <= self.seg_s[i + 1]:
                d = self.route[i + 1][:2] - self.route[i][:2]
                L = np.linalg.norm(d)
                if L > 1e-9:
                    d = d / L
                return np.array([d[0], d[1], 0.0])
        return np.array([1.0, 0.0, 0.0])

    # ---- save ----
    def cluster(self):
        clusters = []
        for w in self.obs:
            for c in clusters:
                if np.linalg.norm(w - c["sum"] / c["n"]) < self.cluster_radius:
                    c["sum"] += w
                    c["n"] += 1
                    c["ws"].append(w)
                    break
            else:
                clusters.append({"sum": w.copy(), "n": 1, "ws": [w]})
        return [c for c in clusters if c["n"] >= self.min_support]

    def save(self, _req):
        clusters = self.cluster()
        gates = []
        for c in clusters:
            w = np.median(np.array(c["ws"]), axis=0)
            s, dcross = self.project_route(w[:2])
            if dcross > self.max_d_cross:
                continue
            n = self.route_tangent(s)
            gates.append({"x": float(w[0]), "y": float(w[1]), "z": float(w[2]),
                          "nx": float(n[0]), "ny": float(n[1]), "nz": float(n[2]),
                          "s": float(s), "d_cross": float(dcross),
                          "support": int(c["n"]), "valid": True,
                          "source": "opencv_stereo"})
        gates.sort(key=lambda g: g["s"])
        for i, g in enumerate(gates):
            g["id"] = i
        with open(self.out_file, "w") as f:
            yaml.safe_dump({"gates": gates}, f, default_flow_style=False, sort_keys=False)
        rospy.loginfo("saved %d gates to %s", len(gates), self.out_file)
        return TriggerResponse(success=True, message="saved %d gates" % len(gates))

    def clear(self, _req):
        n = len(self.obs)
        self.obs = []
        return TriggerResponse(success=True, message="cleared %d observations" % n)


if __name__ == "__main__":
    rospy.init_node("gate_vision")
    GateVision()
    rospy.spin()
