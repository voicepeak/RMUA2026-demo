#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RMUA 2026 Stage 0: 最基础 Start-to-Goal 运动闭环。

订阅:
  /airsim_node/drone_1/debug/pose_gt   (geometry_msgs/PoseStamped)  当前位姿(真值)
  /airsim_node/end_goal                (geometry_msgs/PoseStamped)  官方目标点
发布:
  /airsim_node/drone_1/vel_body_cmd    (airsim_ros/VelCmd)          速度控制(机体系)

坐标约定: AirSim NED (x 前/北, y 右/东, z 下)。
"""

import math

import rospy
import tf.transformations as tft
from geometry_msgs.msg import Point, PoseStamped

from airsim_ros.msg import VelCmd


class StartToGoal(object):

    def __init__(self):
        self.kp_xy = rospy.get_param("~kp_xy", 0.3)
        self.kp_z = rospy.get_param("~kp_z", 0.3)
        self.vmax_xy = rospy.get_param("~vmax_xy", 2.0)
        self.vmax_z = rospy.get_param("~vmax_z", 1.0)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 1.0)
        self.accel = int(rospy.get_param("~accel", 8))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))

        self.use_end_goal = bool(rospy.get_param("~use_end_goal", True))
        self.fixed_goal = rospy.get_param("~fixed_goal", [10.0, 0.0, 0.0])

        self.do_takeoff = bool(rospy.get_param("~do_takeoff", False))
        self.takeoff_height = float(rospy.get_param("~takeoff_height", 1.0))

        self.pose = None
        self.goal = None
        self.reached = False
        self.start_z = None
        self.takeoff_done = not self.do_takeoff

        self.cmd_pub = rospy.Publisher(
            "/airsim_node/drone_1/vel_body_cmd", VelCmd, queue_size=1)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt",
                         PoseStamped, self.pose_cb)
        rospy.Subscriber("/airsim_node/end_goal",
                         PoseStamped, self.goal_cb)
        rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_loop)

        rospy.loginfo("StartToGoal ready: use_end_goal=%s fixed_goal=%s",
                      self.use_end_goal, self.fixed_goal)

    def pose_cb(self, msg):
        self.pose = msg.pose
        if self.start_z is None:
            self.start_z = msg.pose.position.z

    def goal_cb(self, msg):
        if self.use_end_goal:
            self.goal = msg.pose.position

    def publish_cmd(self, vx, vy, vz, yaw_rate=0.0):
        cmd = VelCmd()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "drone_1"
        cmd.vx = vx
        cmd.vy = vy
        cmd.vz = vz
        cmd.yawRate = yaw_rate
        cmd.va = self.accel
        cmd.stop = 0
        self.cmd_pub.publish(cmd)

    def hover(self):
        self.publish_cmd(0.0, 0.0, 0.0, 0.0)

    def control_loop(self, _event):
        if self.pose is None:
            return
        p = self.pose.position
        q = self.pose.orientation

        if not self.takeoff_done:
            altitude = self.start_z - p.z
            if altitude >= self.takeoff_height:
                self.takeoff_done = True
                rospy.loginfo("Takeoff done, altitude=%.2f m", altitude)
                return
            self.publish_cmd(0.0, 0.0, -self.vmax_z, 0.0)
            return

        if self.use_end_goal:
            if self.goal is None:
                rospy.logwarn_throttle(2.0, "waiting for /airsim_node/end_goal ...")
                self.hover()
                return
            goal = self.goal
        else:
            goal = Point(self.fixed_goal[0], self.fixed_goal[1],
                         self.fixed_goal[2])

        ex = goal.x - p.x
        ey = goal.y - p.y
        ez = goal.z - p.z
        distance = math.sqrt(ex * ex + ey * ey + ez * ez)

        if distance < self.goal_tolerance:
            self.hover()
            if not self.reached:
                self.reached = True
                rospy.loginfo("GOAL_REACHED distance=%.3f m", distance)
            return
        self.reached = False

        vx_w = self.kp_xy * ex
        vy_w = self.kp_xy * ey
        # 模拟器机体 z 轴向上为正，而世界系 z 向下为正(NED)，因此垂直方向取反
        vz = -self.kp_z * ez

        speed_xy = math.hypot(vx_w, vy_w)
        if speed_xy > self.vmax_xy and speed_xy > 1e-6:
            scale = self.vmax_xy / speed_xy
            vx_w *= scale
            vy_w *= scale

        vz = max(-self.vmax_z, min(self.vmax_z, vz))

        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        vx_b = cos_y * vx_w + sin_y * vy_w
        vy_b = -sin_y * vx_w + cos_y * vy_w

        self.publish_cmd(vx_b, vy_b, vz, 0.0)
        rospy.loginfo_throttle(
            1.0,
            "pos(%.2f,%.2f,%.2f) goal(%.2f,%.2f,%.2f) err(%.2f,%.2f,%.2f) "
            "d=%.2f yaw=%.1f cmd_body(%.2f,%.2f,%.2f)",
            p.x, p.y, p.z, goal.x, goal.y, goal.z,
            ex, ey, ez, distance, math.degrees(yaw), vx_b, vy_b, vz)


if __name__ == "__main__":
    rospy.init_node("start_to_goal")
    StartToGoal()
    rospy.spin()
