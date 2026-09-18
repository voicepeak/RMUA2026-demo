#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO Gate 推理节点 (替换 OpenCV Detector)。

用 xal 环境运行 (含 torch/ultralytics):
  PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages \
  /home/huang/miniconda3/envs/xal/bin/python gate_yolo_node.py _model:=.../best.pt

订阅: front_left/Scene, front_right/Scene (同步), pose_gt
发布: /rmua/gate_detection/left, /right, /stereo  (可视化)
      /rmua/gate_observations (JSON)
服务: ~save (保存稳定 Gate 到 YAML), ~clear

门中心深度用门框像素稠密视差估计 (gate_stereo), 四角交给下游三角化。
"""

import json
import math
import os
import sys

import cv2
import message_filters
import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gate_detector_opencv import gate_frame_mask  # noqa: E402
import gate_stereo as gs  # noqa: E402


def imgmsg_to_bgr(m):
    buf = np.frombuffer(m.data, dtype=np.uint8)
    if m.encoding in ("bgr8", "8UC3"):
        return buf.reshape(m.height, m.width, 3)
    if m.encoding == "rgb8":
        return buf.reshape(m.height, m.width, 3)[:, :, ::-1].copy()
    if m.encoding in ("mono8", "8UC1"):
        g = buf.reshape(m.height, m.width)
        return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    return buf.reshape(m.height, m.width, -1)[:, :, :3]


def bgr_to_imgmsg(bgr, stamp, frame_id):
    m = Image()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.height, m.width = bgr.shape[:2]
    m.encoding = "bgr8"
    m.is_bigendian = 0
    m.step = bgr.shape[1] * 3
    m.data = bgr.tobytes()
    return m


class GateYolo(object):

    def __init__(self):
        cfg = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "config"))
        self.model_path = rospy.get_param("~model")
        self.conf = rospy.get_param("~conf", 0.35)
        self.imgsz = rospy.get_param("~imgsz", 960)
        self.route_file = rospy.get_param("~route_file", os.path.join(cfg, "route_1_3.yaml"))
        self.route_name = rospy.get_param("~route_name", "route_1_3")
        self.out_file = rospy.get_param("~out_file", os.path.join(cfg, "gates_yolo_1_3.yaml"))
        self.cluster_radius = rospy.get_param("~cluster_radius", 9.0)
        self.min_support = rospy.get_param("~min_support", 6)
        self.max_d_cross = rospy.get_param("~max_d_cross", 6.0)

        from ultralytics import YOLO
        self.model = YOLO(self.model_path)
        rospy.loginfo("YOLO loaded: %s", self.model_path)

        self.sgbm = gs.build_sgbm()
        self.pose = None
        self.obs = []
        self.pub_l = rospy.Publisher("/rmua/gate_detection/left", Image, queue_size=2)
        self.pub_r = rospy.Publisher("/rmua/gate_detection/right", Image, queue_size=2)
        self.pub_s = rospy.Publisher("/rmua/gate_detection/stereo", Image, queue_size=2)
        self.pub_o = rospy.Publisher("/rmua/gate_observations", String, queue_size=5)
        rospy.Service("~save", Trigger, self.save)
        rospy.Service("~clear", Trigger, self.clear)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.pose_cb)
        sl = message_filters.Subscriber("/airsim_node/drone_1/front_left/Scene", Image)
        sr = message_filters.Subscriber("/airsim_node/drone_1/front_right/Scene", Image)
        message_filters.ApproximateTimeSynchronizer([sl, sr], queue_size=5, slop=0.03) \
            .registerCallback(self.cb)
        self.route = self.load_route()

    def load_route(self):
        with open(self.route_file) as f:
            pts = yaml.safe_load(f)["routes"][self.route_name]
        r = np.array(pts, dtype=np.float64)
        self.seg_len = np.hypot(np.diff(r[:, 0]), np.diff(r[:, 1]))
        self.seg_s = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        return r

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

    def route_tangent(self, s):
        for i in range(len(self.route) - 1):
            if self.seg_s[i] <= s <= self.seg_s[i + 1]:
                d = self.route[i + 1][:2] - self.route[i][:2]
                L = np.linalg.norm(d)
                d = d / L if L > 1e-9 else d
                return np.array([d[0], d[1], 0.0])
        return np.array([1.0, 0.0, 0.0])

    def pose_cb(self, m):
        self.pose = m.pose

    def detect(self, bgr):
        res = self.model.predict(bgr, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]
        cands = []
        if res.boxes is not None and len(res.boxes) > 0:
            xyxy = res.boxes.xyxy.cpu().numpy()
            cf = res.boxes.conf.cpu().numpy()
            for (x0, y0, x1, y1), c in zip(xyxy, cf):
                x0, y0 = max(0, int(x0)), max(0, int(y0))
                x1, y1 = min(bgr.shape[1] - 1, int(x1)), min(bgr.shape[0] - 1, int(y1))
                if x1 <= x0 or y1 <= y0:
                    continue
                cands.append({"bbox": (x0, y0, x1 - x0, y1 - y0),
                              "center": np.array([(x0 + x1) / 2.0, (y0 + y1) / 2.0]),
                              "conf": float(c),
                              "corners": np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], np.float32)})
        return cands

    def draw(self, bgr, cands):
        out = bgr.copy()
        for c in cands:
            x, y, w, h = c["bbox"]
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(out, "gate %.2f" % c["conf"], (x, max(15, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        return out

    def cb(self, ml, mr):
        if self.pose is None:
            return
        left = imgmsg_to_bgr(ml)
        right = imgmsg_to_bgr(mr)
        cl = self.detect(left)
        cr = self.detect(right)
        disp = gs.compute_disparity(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY),
                                    cv2.cvtColor(right, cv2.COLOR_BGR2GRAY), self.sgbm)
        fmask = gate_frame_mask(left, {})
        px = self.pose.position
        q = self.pose.orientation
        pos = np.array([px.x, px.y, px.z])
        quat = (q.x, q.y, q.z, q.w)
        obs = []
        for c in cl:
            r = gs.gate_center_from_frame(c, disp, fmask)
            if r is None:
                continue
            cc, depth, npx = r
            if depth < 1.0 or depth > 200.0:
                continue
            world = gs.body_to_world(gs.camera_to_body(cc), pos, quat)
            if np.all(np.isfinite(world)):
                self.obs.append(world.copy())
                obs.append({"world": world.tolist(), "depth": float(depth), "conf": c["conf"]})

        vl, vr = self.draw(left, cl), self.draw(right, cr)
        st = cv2.hconcat([vl, vr])
        cv2.putText(st, "YOLO LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.putText(st, "YOLO RIGHT", (left.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 255, 255), 2)
        now = ml.header.stamp
        self.pub_l.publish(bgr_to_imgmsg(vl, now, "drone_1"))
        self.pub_r.publish(bgr_to_imgmsg(vr, now, "drone_1"))
        self.pub_s.publish(bgr_to_imgmsg(st, now, "drone_1"))
        self.pub_o.publish(String(data=json.dumps(obs)))
        rospy.loginfo_throttle(2.0, "YOLO gates L=%d R=%d, obs=%d", len(cl), len(cr), len(self.obs))

    def cluster(self):
        cl = []
        for w in self.obs:
            for c in cl:
                if np.linalg.norm(w - c["sum"] / c["n"]) < self.cluster_radius:
                    c["sum"] += w
                    c["n"] += 1
                    c["ws"].append(w)
                    break
            else:
                cl.append({"sum": w.copy(), "n": 1, "ws": [w]})
        return [c for c in cl if c["n"] >= self.min_support]

    def save(self, _req):
        gates = []
        for c in self.cluster():
            w = np.median(np.array(c["ws"]), axis=0)
            s, dc = self.project_route(w[:2])
            if dc > self.max_d_cross:
                continue
            n = self.route_tangent(s)
            gates.append({"x": float(w[0]), "y": float(w[1]), "z": float(w[2]),
                          "nx": float(n[0]), "ny": float(n[1]), "nz": 0.0,
                          "s": float(s), "support": int(c["n"]), "valid": True,
                          "source": "yolo"})
        gates.sort(key=lambda g: g["s"])
        for i, g in enumerate(gates):
            g["id"] = i
        with open(self.out_file, "w") as f:
            yaml.safe_dump({"gates": gates}, f, default_flow_style=False, sort_keys=False)
        return TriggerResponse(success=True, message="saved %d gates (yolo)" % len(gates))

    def clear(self, _req):
        n = len(self.obs)
        self.obs = []
        return TriggerResponse(success=True, message="cleared %d obs" % n)


if __name__ == "__main__":
    rospy.init_node("gate_yolo")
    GateYolo()
    rospy.spin()
