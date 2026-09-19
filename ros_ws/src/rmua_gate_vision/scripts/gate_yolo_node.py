#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO Gate 推理节点: 四角关键点 + 双目逐角三角化 + 时间同步 + 反投影 (Stage11)。

用 xal 环境运行 (含 torch/ultralytics):
  PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages \
  /home/huang/miniconda3/envs/xal/bin/python gate_yolo_node.py _model:=.../best.pt

订阅: front_left/Scene, front_right/Scene (同步), pose_gt, [imu/imu]
发布: /rmua/gate_detection/{left,right,stereo}, /rmua/gate_observations, /rmua/gate_map
服务: ~save, ~clear

Stage11 管线 (替代 "bbox 中心 + 稠密视差"):
  YOLO(Pose/四角) -> 左右门级匹配 -> 逐角三角化 -> GateGeometry 质量检查
  -> TimestampSynced Pose -> 完整 quaternion World 变换 -> Persistent GateMap
  -> 反投影关联 (下一帧)。

若模型只有 bbox, 则用 bbox 四角参与三角化 (仍避免门洞背景深度), 稠密视差仅作兜底。
"""

import json
import os
import sys

import cv2
import message_filters
import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gate_detector_opencv import gate_frame_mask  # noqa: E402
import gate_stereo as gs  # noqa: E402
from gate_map import GateMap  # noqa: E402
from gate_geometry import GateGeometry  # noqa: E402
from timestamp_sync import PoseBuffer  # noqa: E402
import stereo_keypoint_matcher as skm  # noqa: E402
from gate_reprojection import reproject_gate  # noqa: E402


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


def _stamp_to_sec(st):
    return st.to_sec() if st is not None else 0.0


class GateYolo(object):

    def __init__(self):
        cfg = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "config"))
        self.model_path = rospy.get_param("~model")
        self.conf = rospy.get_param("~conf", 0.35)
        self.imgsz = rospy.get_param("~imgsz", 960)
        self.route_file = rospy.get_param("~route_file", os.path.join(cfg, "route_1_3.yaml"))
        self.route_name = rospy.get_param("~route_name", "route_1_3")
        self.out_file = rospy.get_param("~out_file", os.path.join(cfg, "gates_yolo_1_3.yaml"))
        self.assoc_radius = rospy.get_param("~assoc_radius", 4.0)
        self.min_support = rospy.get_param("~min_support", 4)
        self.max_d_cross = rospy.get_param("~max_d_cross", 6.0)
        self.max_age = rospy.get_param("~max_age", 3.0)
        self.use_imu = bool(rospy.get_param("~use_imu", False))
        self.use_keypoints = bool(rospy.get_param("~use_keypoints", True))
        self.corner_only = bool(rospy.get_param("~corner_only", False))
        self.sync_max_age = rospy.get_param("~sync_max_age", 1.0)
        self.fallback_center = bool(rospy.get_param("~fallback_dense_center", True))

        from ultralytics import YOLO
        self.model = YOLO(self.model_path)
        rospy.loginfo("YOLO loaded: %s", self.model_path)

        self.geom = GateGeometry(
            min_width=rospy.get_param("~min_width", 0.5),
            max_width=rospy.get_param("~max_width", 8.0),
            min_height=rospy.get_param("~min_height", 0.5),
            max_height=rospy.get_param("~max_height", 8.0),
            plane_rmse_max=rospy.get_param("~plane_rmse_max", 0.25),
            side_mismatch_max=rospy.get_param("~side_mismatch_max", 0.5))

        self.sgbm = gs.build_sgbm()
        self.pose = None
        self.imu_quat = None
        self.pose_buf = PoseBuffer(max_age=self.sync_max_age)
        self.pub_l = rospy.Publisher("/rmua/gate_detection/left", Image, queue_size=2)
        self.pub_r = rospy.Publisher("/rmua/gate_detection/right", Image, queue_size=2)
        self.pub_s = rospy.Publisher("/rmua/gate_detection/stereo", Image, queue_size=2)
        self.pub_o = rospy.Publisher("/rmua/gate_observations", String, queue_size=5)
        self.pub_map = rospy.Publisher("/rmua/gate_map", String, queue_size=2)
        rospy.Service("~save", Trigger, self.save)
        rospy.Service("~clear", Trigger, self.clear)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.pose_cb)
        if self.use_imu:
            rospy.Subscriber("/airsim_node/drone_1/imu/imu", Imu, self.imu_cb)
        sl = message_filters.Subscriber("/airsim_node/drone_1/front_left/Scene", Image)
        sr = message_filters.Subscriber("/airsim_node/drone_1/front_right/Scene", Image)
        message_filters.ApproximateTimeSynchronizer([sl, sr], queue_size=5, slop=0.03) \
            .registerCallback(self.cb)
        self.route = self.load_route()
        self.gate_map = GateMap(self.route, assoc_radius=self.assoc_radius,
                                max_age=self.max_age)
        self.last_prune = rospy.Time.now()
        rospy.Timer(rospy.Duration(0.25), self.publish_map)

    def load_route(self):
        with open(self.route_file) as f:
            pts = yaml.safe_load(f)["routes"][self.route_name]
        r = np.array(pts, dtype=np.float64)
        self.seg_len = np.hypot(np.diff(r[:, 0]), np.diff(r[:, 1]))
        self.seg_s = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        return r

    def pose_cb(self, m):
        self.pose = m.pose
        p = m.pose.position
        q = m.pose.orientation
        t = _stamp_to_sec(m.header.stamp)
        if t <= 0.0:
            t = rospy.Time.now().to_sec()
        self.pose_buf.add(t, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def imu_cb(self, m):
        q = m.orientation
        self.imu_quat = (q.x, q.y, q.z, q.w)

    def current_pose(self, t_img):
        """返回 (pos, quat) —— 优先时间同步插值, 否则退回最近位姿。"""
        pos = quat = None
        if self.pose_buf.ready():
            pos, quat, _ = self.pose_buf.sample(t_img)
        if pos is None:
            px = self.pose.position
            q = self.pose.orientation
            pos = (px.x, px.y, px.z)
            quat = (q.x, q.y, q.z, q.w)
        if self.use_imu and self.imu_quat is not None and not self.pose_buf.ready():
            quat = self.imu_quat
        return np.asarray(pos, dtype=float), quat

    def detect(self, bgr):
        res = self.model.predict(bgr, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]
        cands = []
        if res.boxes is None or len(res.boxes) == 0:
            return cands, False
        xyxy = res.boxes.xyxy.cpu().numpy()
        cf = res.boxes.conf.cpu().numpy()
        kxy = None
        if self.use_keypoints and getattr(res, "keypoints", None) is not None \
                and res.keypoints is not None and len(res.keypoints) > 0:
            try:
                kxy = res.keypoints.xy.cpu().numpy()      # (n, K, 2)
            except Exception:
                kxy = None
        used_kp = kxy is not None and len(kxy) == len(xyxy) and kxy.shape[1] >= 4
        for i, ((x0, y0, x1, y1), c) in enumerate(zip(xyxy, cf)):
            x0, y0 = max(0, int(x0)), max(0, int(y0))
            x1, y1 = min(bgr.shape[1] - 1, int(x1)), min(bgr.shape[0] - 1, int(y1))
            if x1 <= x0 or y1 <= y0:
                continue
            if used_kp:
                corners = np.asarray(kxy[i][:4], dtype=np.float32).reshape(4, 2)
            else:
                corners = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], np.float32)
            cands.append({
                "bbox": (x0, y0, x1 - x0, y1 - y0),
                "center": corners.mean(axis=0),
                "conf": float(c),
                "area": float((x1 - x0) * (y1 - y0)),
                "corners": corners,
                "keypoints": bool(used_kp),
            })
        return cands, used_kp

    def draw(self, bgr, cands):
        out = bgr.copy()
        for c in cands:
            x, y, w, h = c["bbox"]
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 1)
            cpts = np.asarray(c["corners"], dtype=int)
            cv2.polylines(out, [cpts], True, (0, 255, 0), 2)
            for pt in cpts:
                cv2.circle(out, tuple(pt), 3, (0, 255, 255), -1)
            tag = "gate %.2f%s" % (c["conf"], " K" if c.get("keypoints") else "")
            if c.get("track_id") is not None:
                tag += " #%d" % c["track_id"]
            cv2.putText(out, tag, (x, max(15, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 2)
        return out

    def _observe_corner_pair(self, gl, gr, pos, quat, t_img, obs):
        X = skm.triangulate_corners(gl["corners"], gr["corners"])
        geom = self.geom.compute(X, camera_origin=np.zeros(3))
        if not geom["geometry_valid"]:
            return False
        world = gs.body_to_world(gs.camera_to_body(geom["center"]), pos, quat)
        if not np.all(np.isfinite(world)):
            return False
        uv = np.asarray(gl["center"], dtype=float)
        t = self.gate_map.observe(world, t_img, depth=geom["depth_mean"],
                                  geom=geom, uv=uv, conf=gl["conf"])
        gl["track_id"] = int(t.id)
        obs.append({"world": world.tolist(), "depth": float(geom["depth_mean"]),
                    "conf": gl["conf"], "track_id": int(t.id),
                    "sigma_z": float(t.sigma()[2]),
                    "plane_rmse": float(geom["plane_rmse"]),
                    "width": float(geom["width"]), "height": float(geom["height"]),
                    "geometry_valid": bool(geom["geometry_valid"]),
                    "u": float(uv[0]), "v": float(uv[1]), "keypoints": gl.get("keypoints", False)})
        return True

    def cb(self, ml, mr):
        if self.pose is None:
            return
        left = imgmsg_to_bgr(ml)
        right = imgmsg_to_bgr(mr)
        cl, kp_l = self.detect(left)
        cr, _ = self.detect(right)
        tl = _stamp_to_sec(ml.header.stamp)
        tr = _stamp_to_sec(mr.header.stamp)
        t_img = 0.5 * (tl + tr) if (tl > 0 and tr > 0) else rospy.Time.now().to_sec()
        pos, quat = self.current_pose(t_img)

        # 用当前位姿反投影旧 track, 供本帧关联 (方案 27/28)
        self.gate_map.predict(pos, quat)

        obs = []
        if not self.corner_only:
            for gl, gr, _ in skm.match_gate_corners(cl, cr):
                self._observe_corner_pair(gl, gr, pos, quat, t_img, obs)

        # 兜底: 未匹配上的左目检测 -> 稠密视差 (旧路径)
        if self.fallback_center:
            unmatched = [c for c in cl if c.get("track_id") is None]
            if unmatched:
                disp = gs.compute_disparity(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY),
                                            cv2.cvtColor(right, cv2.COLOR_BGR2GRAY), self.sgbm)
                fmask = gate_frame_mask(left, {})
                for c in unmatched:
                    r = gs.gate_center_from_frame(c, disp, fmask)
                    if r is None:
                        continue
                    cc, depth, _ = r
                    if depth < 1.0 or depth > 200.0:
                        continue
                    world = gs.body_to_world(gs.camera_to_body(cc), pos, quat)
                    if not np.all(np.isfinite(world)):
                        continue
                    t = self.gate_map.observe(world, t_img, depth=depth,
                                              uv=np.asarray(c["center"], float), conf=c["conf"])
                    c["track_id"] = int(t.id)
                    obs.append({"world": world.tolist(), "depth": float(depth),
                                "conf": c["conf"], "track_id": int(t.id),
                                "sigma_z": float(t.sigma()[2]), "geometry_valid": False,
                                "keypoints": c.get("keypoints", False)})

        vl, vr = self.draw(left, cl), self.draw(right, cr)
        st = cv2.hconcat([vl, vr])
        cv2.putText(st, "YOLO LEFT%s" % (" KP" if kp_l else " bbox"),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.putText(st, "YOLO RIGHT", (left.shape[1] + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        now = ml.header.stamp
        self.pub_l.publish(bgr_to_imgmsg(vl, now, "drone_1"))
        self.pub_r.publish(bgr_to_imgmsg(vr, now, "drone_1"))
        self.pub_s.publish(bgr_to_imgmsg(st, now, "drone_1"))
        self.pub_o.publish(String(data=json.dumps(obs)))

        stamp = rospy.Time.now().to_sec()
        if stamp - self.last_prune > self.max_age:
            self.gate_map.prune(stamp)
            self.last_prune = stamp
        tracks = len(self.gate_map.tracker.tracks)
        rospy.loginfo_throttle(
            2.0, "YOLO L=%d R=%d obs=%d kp=%s tracks=%d map=%s",
            len(cl), len(cr), len(obs), kp_l, tracks,
            [(g["id"], round(g["s"], 1), round(g["z"], 2), round(g["sigma_z"], 2),
              g["hard_anchor"], g["geometry_valid"]) for g in self.gate_map.gates(2)])

    def publish_map(self, _e=None):
        self.pub_map.publish(String(data=self.gate_map.to_json(min_support=self.min_support)))

    def save(self, _req):
        n = self.gate_map.save_yaml(self.out_file, min_support=self.min_support,
                                    max_d_cross=self.max_d_cross)
        return TriggerResponse(success=True, message="saved %d gates (yolo)" % n)

    def clear(self, _req):
        n = len(self.gate_map.tracker.tracks)
        self.gate_map = GateMap(self.route, assoc_radius=self.assoc_radius,
                                max_age=self.max_age)
        return TriggerResponse(success=True, message="cleared %d tracks" % n)


if __name__ == "__main__":
    rospy.init_node("gate_yolo")
    GateYolo()
    rospy.spin()
