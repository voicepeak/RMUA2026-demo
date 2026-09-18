#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Waypoint 采集器 (第二阶段文档 ①)。

手动/低速把飞机停在道路中心, 依次调用:
  rosservice call /route_recorder/add      # 记录当前 pose_gt
  rosservice call /route_recorder/save     # 写入 YAML
  rosservice call /route_recorder/clear    # 清空
"""

import os

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse


class RouteRecorder(object):

    def __init__(self):
        self.out_file = rospy.get_param(
            "~out_file",
            os.path.join(os.path.dirname(__file__), "..", "config", "recorded_route.yaml"))
        self.route_name = rospy.get_param("~route_name", "recorded")
        self.points = []
        self.gates = []
        self.pose = None
        self.yaw = 0.0
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.cb)
        rospy.Service("~add", Trigger, self.add)
        rospy.Service("~save", Trigger, self.save)
        rospy.Service("~clear", Trigger, self.clear)
        rospy.Service("~add_gate", Trigger, self.add_gate)
        rospy.Service("~save_gates", Trigger, self.save_gates)
        rospy.loginfo("RouteRecorder ready, out_file=%s", self.out_file)

    def cb(self, msg):
        self.pose = msg.pose.position
        q = msg.pose.orientation
        import tf.transformations as tft
        _, _, self.yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def add(self, _req):
        if self.pose is None:
            return TriggerResponse(success=False, message="no pose_gt yet")
        p = self.pose
        self.points.append([round(p.x, 4), round(p.y, 4), round(p.z, 4)])
        return TriggerResponse(success=True, message="recorded #%d (%.2f, %.2f, %.2f)"
                               % (len(self.points), p.x, p.y, p.z))

    def save(self, _req):
        data = {"routes": {self.route_name: self.points}}
        with open(self.out_file, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=True)
        return TriggerResponse(success=True, message="saved %d points to %s"
                               % (len(self.points), self.out_file))

    def clear(self, _req):
        n = len(self.points)
        self.points = []
        return TriggerResponse(success=True, message="cleared %d points" % n)

    def add_gate(self, _req):
        """把当前位姿记录为一个 Gate 中心, 法向取当前机头水平朝向。"""
        if self.pose is None:
            return TriggerResponse(success=False, message="no pose_gt yet")
        import math
        p = self.pose
        g = {
            "id": len(self.gates),
            "x": round(p.x, 3), "y": round(p.y, 3), "z": round(p.z, 3),
            "nx": round(math.cos(self.yaw), 4),
            "ny": round(math.sin(self.yaw), 4),
            "nz": 0.0,
            "approach_distance": 4.0,
            "speed": 1.5,
        }
        self.gates.append(g)
        return TriggerResponse(success=True, message="gate #%d (%.2f, %.2f, %.2f)"
                               % (g["id"], p.x, p.y, p.z))

    def save_gates(self, _req):
        path = rospy.get_param("~gates_out_file",
                               os.path.join(os.path.dirname(self.out_file), "gates_recorded.yaml"))
        with open(path, "w") as f:
            yaml.safe_dump({"gates": self.gates}, f, default_flow_style=False, sort_keys=False)
        return TriggerResponse(success=True, message="saved %d gates to %s"
                               % (len(self.gates), path))


if __name__ == "__main__":
    rospy.init_node("route_recorder")
    RouteRecorder()
    rospy.spin()
