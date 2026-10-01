#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RMUA 2026 v8: Control Authority Cleanup + Preview Feasibility Speed Planning.

本版本按 "坡道拉升、穿门卡顿与控制链冲突完整解决方案" 重构:

  - Z 轴正常 TRACK 只有一套 authority: vz = FF + FB (ZController);
  - 删除 gate time-to-gate -> vz override 作为主控制 (k_anticipate 仅留 A/B, 默认 0);
  - Z_LAG 不再日常调速, 降级为 SafetySupervisor 严重掉队 recovery (方案 6.1);
  - 水平速度唯一 authority = SpeedScheduler, 统一 hard_cap:
      v_curve_preview (curve braking envelope) + v_climb_preview (climb feasibility)
      + v_map (RECON/MAP_HORIZON) + tracking soft cap;
  - RECON 独立 mode, 不与 TRACK 的 FF+FB 叠加 (方案 10, 38.2);
  - Gate 账本 resolved/passed/missed/skipped 分离 (方案 15);
  - 过门判定优先真实 Gate plane 相交 (方案 16);
  - 使用 pose timestamp 计算真实 dt + pose timeout (方案 22);
  - AltitudeProfile 不再重复融合 gate; 在线地图刷新用 ProfileBlender 平滑切换 (方案 12).

坐标: AirSim NED; vel_body_cmd 的 vz 向上为正。
"""

import json
import math
import os
import sys
import threading
import copy
import time

import numpy as np
import rospy
import tf.transformations as tft
import yaml
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from airsim_ros.msg import VelCmd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from altitude_profile import ProfileBlender                                      # noqa: E402
from avoidance_interface import AvoidanceCommand                                 # noqa: E402
from climb_feasibility import ClimbFeasibility                                   # noqa: E402
from command_arbiter import CommandArbiter                                       # noqa: E402
from curve_preview import CurveBrakingEnvelope                                   # noqa: E402
from gate_task_state import GateTaskState                                        # noqa: E402
from mission_state import MissionState, RECON, HOLD, ABORT                       # noqa: E402
from online_gate_cache import OnlineGateCache                                    # noqa: E402
from reference_planner import RouteGeometry, ReferencePlanner                    # noqa: E402
from safety_supervisor import SafetySupervisor                                   # noqa: E402
from speed_scheduler import SpeedScheduler                                       # noqa: E402
from lidar_clearance import LidarClearance
from persistent_guides import PersistentGuides
from xy_tracker import XYTracker                                                 # noqa: E402
from motion_estimator import MotionEstimator
from terrain_speed import TerrainSpeedEnvelope
from predictive_avoidance import PredictiveAvoidance, CarTracks, JoinedDetour
from execution_guard import ExecutionGuard
from point_index import measured_car_faces
from lattice_detour import LatticeDetour
from pose_history import PoseHistory
from lidar_points import valid_points
from async_planner import AsyncPlanner
from yaw_controller import YawController                                         # noqa: E402
from z_capability import load_capability                                         # noqa: E402
from z_controller import ZController                                             # noqa: E402


class RouteFollower(object):

    def __init__(self):
        cfg = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "config"))
        self.route_file = rospy.get_param("~route_file", os.path.join(cfg, "route_1_3.yaml"))
        self.route_name = rospy.get_param("~route_name", "route_1_3")
        self.gates_file = rospy.get_param("~gates_file", os.path.join(cfg, "gates_vision_1_3.yaml"))
        self.guides_file = rospy.get_param("~guides_file", os.path.join(cfg, "none.yaml"))
        self.gate_z_uses_offset = bool(rospy.get_param("~gate_z_uses_offset", True))
        self.start_anchor_z = rospy.get_param("~start_anchor_z", -999.0)
        self.start_anchor_z = None if self.start_anchor_z == -999.0 else self.start_anchor_z

        # speed
        self.cruise_speed = rospy.get_param("~cruise_speed", 8.0)
        self.max_speed = rospy.get_param("~max_speed", 12.0)
        self.normal_speed_floor = rospy.get_param("~normal_speed_floor", 4.0)
        self.adaptive_speed=bool(rospy.get_param('~adaptive_speed',False))
        self.lidar_range=float(rospy.get_param('~lidar_range',30.))
        self.lidar_braking=float(rospy.get_param('~lidar_braking',8. if self.adaptive_speed else 4.))
        self.sensor_reaction=float(rospy.get_param('~sensor_reaction',.35))
        self.a_lat_max = rospy.get_param("~a_lat_max", 6.0)
        self.a_up = rospy.get_param("~a_up", 4.0)
        self.a_down = rospy.get_param("~a_down", 5.0)
        self.lookahead_base = rospy.get_param("~lookahead_base", 8.0)
        self.lookahead_kv = rospy.get_param("~lookahead_kv", 0.7)
        self.k_pursuit = rospy.get_param("~k_pursuit", 1.2)

        # z
        self.k_z = rospy.get_param("~k_z", 1.0)
        self.z_rate_max = rospy.get_param("~z_rate_max", 4.0)
        self.terminal_hover_height = float(rospy.get_param('~terminal_hover_height',1.5))
        self.departure_hover_z=float(rospy.get_param('~departure_hover_z',-999.))
        self.corridor_half = rospy.get_param("~corridor_half", 1.5)
        self.gate_blend_start = rospy.get_param("~gate_blend_start", 25.0)
        self.gate_blend_full = rospy.get_param("~gate_blend_full", 8.0)

        # gate chain / pass
        self.aperture_xy_tol = rospy.get_param("~aperture_xy_tol", 1.2)
        self.aperture_z_tol = rospy.get_param("~aperture_z_tol", 1.0)
        self.gate_pass_half_width = rospy.get_param("~gate_pass_half_width", 1.5)
        self.gate_pass_half_height = rospy.get_param("~gate_pass_half_height", 1.5)
        self.gate_miss_margin = rospy.get_param("~gate_miss_margin", 3.0)
        self.gate_miss_radius = rospy.get_param("~gate_miss_radius", 6.0)
        self.gate_skip_s = rospy.get_param("~gate_skip_s", 5.0)
        self.startup_z_jump_limit = rospy.get_param("~startup_z_jump_limit", 1.0)
        self.ff_gate_margin = rospy.get_param("~ff_gate_margin", 4.0)
        self.trace_gate = int(rospy.get_param("~trace_gate", -1))
        self.trace_file = rospy.get_param("~trace_file", "/tmp/opencode/gate_trace.csv")
        self.trace_pre = rospy.get_param("~trace_pre", 30.0)
        self.trace_post = rospy.get_param("~trace_post", 5.0)
        self.trace_fh = None
        self.slope_factor = rospy.get_param("~slope_factor", 3.0)
        self.slope_abs_max = rospy.get_param("~slope_abs_max", 0.6)
        self.xy_converge = rospy.get_param("~xy_converge", 0.5)
        self.snap_gate_to_route = bool(rospy.get_param("~snap_gate_to_route", False))
        self.gate_exit_blend_distance = rospy.get_param("~gate_exit_blend_distance", 4.0)

        # Z 前视/前馈
        self.z_response_time = float(rospy.get_param("~z_response_time", 0.25))
        self.vz_up_safe = rospy.get_param("~vz_up_safe", 4.0)
        self.vz_up_limit = rospy.get_param("~vz_up_limit", 4.0)
        self.vz_down_limit = rospy.get_param("~vz_down_limit", 3.0)
        self.vz_accel_limit = rospy.get_param("~vz_accel_limit", 6.0)
        self.k_ff_z = rospy.get_param("~k_ff_z", 1.0)
        self.slope_eta = rospy.get_param("~slope_eta", 0.8)
        self.pred_time = rospy.get_param("~pred_time", 1.5)
        self.vz_capability_file = rospy.get_param(
            "~vz_capability_file", os.path.join(cfg, "vz_capability.yaml"))

        # tracking 软限速
        self.z_slow1 = rospy.get_param("~z_slow1", 0.3)
        self.z_slow2 = rospy.get_param("~z_slow2", 0.6)
        self.z_slow3 = rospy.get_param("~z_slow3", 1.0)
        self.z_slow4 = rospy.get_param("~z_slow4", 1.5)
        self.z_f2 = rospy.get_param("~z_f2", 0.9)
        self.z_f3 = rospy.get_param("~z_f3", 0.75)
        self.z_f4 = rospy.get_param("~z_f4", 0.6)
        self.z_f5 = rospy.get_param("~z_f5", 0.5)
        self.gate_f1 = rospy.get_param("~gate_f1", 0.8)
        self.gate_f2 = rospy.get_param("~gate_f2", 0.6)
        self.gate_f3 = rospy.get_param("~gate_f3", 0.45)
        self.yaw_slow1 = rospy.get_param("~yaw_slow1", 10.0)
        self.yaw_slow2 = rospy.get_param("~yaw_slow2", 20.0)
        self.yaw_slow3 = rospy.get_param("~yaw_slow3", 30.0)
        self.yaw_f1 = rospy.get_param("~yaw_f1", 0.9)
        self.yaw_f2 = rospy.get_param("~yaw_f2", 0.8)
        self.yaw_f3 = rospy.get_param("~yaw_f3", 0.6)

        # Yaw path following
        self.yaw_control = bool(rospy.get_param("~yaw_control", True))
        self.yaw_lookahead = rospy.get_param("~yaw_lookahead", 25.0)
        self.k_yaw = rospy.get_param("~k_yaw", 1.2)
        self.yaw_rate_max = rospy.get_param("~yaw_rate_max", 1.0)
        self.yaw_accel_limit = rospy.get_param("~yaw_accel_limit", 2.0)
        self.k_vision = rospy.get_param("~k_vision", 0.15)
        self.fov_n_gates = int(rospy.get_param("~fov_n_gates", 3))

        # Z 外推 Horizon
        self.z_horizon_time = rospy.get_param("~z_horizon_time", 2.0)
        self.z_horizon_min = rospy.get_param("~z_horizon_min", 10.0)
        self.z_horizon_max = rospy.get_param("~z_horizon_max", 20.0)

        # Gate Map
        self.use_gate_map = bool(rospy.get_param("~use_gate_map", True))
        self.gate_map_topic = rospy.get_param("~gate_map_topic", "/rmua/gate_map")
        self.gate_map_min_support = int(rospy.get_param("~gate_map_min_support", 2))
        self.sigma_z_max = rospy.get_param("~sigma_z_max", 0.6)

        # Preview feasibility (方案 4, 8, 26)
        self.climb_preview_time = rospy.get_param("~climb_preview_time", 3.5)
        self.climb_preview_min = rospy.get_param("~climb_preview_min", 20.0)
        self.climb_preview_max = rospy.get_param("~climb_preview_max", 50.0)
        self.climb_preview_step = rospy.get_param("~climb_preview_step", 5.0)
        self.climb_response_time = rospy.get_param("~climb_response_time", 0.4)
        self.curve_preview_time = rospy.get_param("~curve_preview_time", 3.0)
        self.curve_preview_min = rospy.get_param("~curve_preview_min", 20.0)
        self.curve_preview_max = rospy.get_param("~curve_preview_max", 40.0)
        self.curve_preview_step = rospy.get_param("~curve_preview_step", 3.0)
        self.curve_brake_a = rospy.get_param("~curve_brake_a", 4.0)

        # Recon / map horizon (方案 38.2)
        self.recon_speed = float(rospy.get_param("~recon_speed", 2.0))
        self.recon_climb = float(rospy.get_param("~recon_climb", 0.6))
        self.recon_max_dist = float(rospy.get_param("~recon_max_dist", 30.0))
        self.recon_max_climb = float(rospy.get_param("~recon_max_climb", 4.0))
        self.map_brake_a = float(rospy.get_param("~map_brake_a", 1.5))

        # Safety recovery (原 Z_LAG 降级, 方案 6.1)
        self.safety_z_error = float(rospy.get_param("~safety_z_error", 2.0))
        self.safety_z_gain = float(rospy.get_param("~safety_z_gain", 5.0))
        self.stuck_speed = float(rospy.get_param("~stuck_speed", 0.4))
        self.stuck_time = float(rospy.get_param("~stuck_time", 2.5))
        self.pose_timeout = float(rospy.get_param("~pose_timeout", 0.3))

        # 无门区坡度外推
        self.z_extrap_m = float(rospy.get_param("~z_extrap_m", 50.0))
        self.z_extrap_slope_max = float(rospy.get_param("~z_extrap_slope_max", 0.5))
        self.z_soft_guide_max = float(rospy.get_param("~z_soft_guide_max", 120.0))
        self.z_soft_slope_max = float(rospy.get_param("~z_soft_slope_max", 0.5))
        self.gate_center_pull_max = float(rospy.get_param("~gate_center_pull_max", 2.0))

        # 上升预测仅保留 A/B 能力, 默认关闭 (方案 2.1/27)
        self.z_anticipate_dist = float(rospy.get_param("~z_anticipate_dist", 25.0))
        self.k_anticipate = float(rospy.get_param("~k_anticipate", 0.0))

        # 在线地图更新 / profile 切换 (方案 13, 26)
        self.map_update_xy_threshold = float(rospy.get_param("~map_update_xy_threshold", 0.15))
        self.map_update_z_threshold = float(rospy.get_param("~map_update_z_threshold", 0.10))
        self.map_update_s_threshold = float(rospy.get_param("~map_update_s_threshold", 0.20))
        self.map_commit_stable_frames = int(rospy.get_param("~map_commit_stable_frames", 1))
        self.static_correction_max = float(rospy.get_param("~static_correction_max", 1.5))
        self.profile_switch_time = float(rospy.get_param("~profile_switch_time", 0.4))

        self.v_cmd_prev = 0.0
        self.z_prev = None
        self.start_gate = int(rospy.get_param("~start_gate", 0))
        self.end_gate = int(rospy.get_param("~end_gate", -1))

        self.accel = int(rospy.get_param("~accel", 8))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.dt = 1.0 / self.control_rate

        self.pose = None
        self.pose_stamp = None
        self.lidar_points = None
        self.lidar_stamp = None
        self.pose_history=PoseHistory()
        self.pending_cloud=None
        self.cloud_history=[]
        self.raw_cloud=None
        self.debug_cloud_dir=rospy.get_param('~debug_cloud_dir','')
        self.debug_cloud_stamp=None
        if self.debug_cloud_dir:os.makedirs(self.debug_cloud_dir,exist_ok=True)
        self.car_tracks=CarTracks()
        self.predictive=PredictiveAvoidance(margin=1.15,braking=self.lidar_braking,budget=1.5,vertical_limit=1.5)
        self.planning=AsyncPlanner()
        self.executing_plan=None
        self.execution_guard=ExecutionGuard(margin=1.15,braking=min(self.lidar_braking,float(self.accel),self.a_down,self.curve_brake_a),
                                             reaction=self.sensor_reaction,horizon=self.lidar_range)
        self.planner_info={}
        self.plan_sequence=0
        self.shift_target=None
        self.shift_retreat=False
        self.retreat_pending=False
        self.shift_checked=None
        self.terrain=TerrainSpeedEnvelope(braking=4.,response=.4)
        self.clearance = LidarClearance(margin=1.15 if self.adaptive_speed else 1.,braking=self.lidar_braking)
        self.clearance_z = 0.
        self.clearance_y = 0.
        self.clearance_checked = None
        self.clearance_info = dict(cap=0. if self.adaptive_speed else None,active=False,source='PENDING')
        self.pose_arrival = None
        self.event_sequence = 0
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.seg = None
        self.z_offset = 0.0
        self.reached = False
        self.stopping = False
        self.prev_s = None
        self.prev_pose = None
        self.chain = None
        self.profile = None
        self.soft_horizon_s = None
        self.soft_horizon_stamp = None
        self.soft_guides = []
        self.guide_cache = PersistentGuides()

        self.avoidance = AvoidanceCommand()
        self.arbiter = CommandArbiter(self.avoidance)
        self.yaw_ctrl = YawController(lookahead=self.yaw_lookahead, k=self.k_yaw,
                                      rate_max=self.yaw_rate_max)
        self.vz_cap = load_capability(self.vz_capability_file)
        self.speed_sched = SpeedScheduler(
            cruise_speed=self.cruise_speed, max_speed=self.max_speed,
            normal_speed_floor=self.normal_speed_floor, a_lat_max=self.a_lat_max,
            slope_eta=self.slope_eta, vz_up_safe=self.vz_up_safe,
            a_up=self.a_up, a_down=self.a_down, vz_capability=self.vz_cap,
            z_slow1=self.z_slow1, z_slow2=self.z_slow2, z_slow3=self.z_slow3,
            z_slow4=self.z_slow4, z_f2=self.z_f2, z_f3=self.z_f3, z_f4=self.z_f4,
            z_f5=self.z_f5, gate_f1=self.gate_f1, gate_f2=self.gate_f2,
            gate_f3=self.gate_f3, yaw_slow1=self.yaw_slow1, yaw_slow2=self.yaw_slow2,
            yaw_slow3=self.yaw_slow3, yaw_f1=self.yaw_f1, yaw_f2=self.yaw_f2,
            yaw_f3=self.yaw_f3,adaptive_speed=self.adaptive_speed,
            braking_accel=self.lidar_braking,reaction_time=self.sensor_reaction,
            jerk_limit=20. if self.adaptive_speed else 0.)
        self.motion=MotionEstimator()
        self.xy_tracker = XYTracker(
            lookahead_base=self.lookahead_base, lookahead_kv=self.lookahead_kv,
            k_pursuit=self.k_pursuit, xy_converge=self.xy_converge,
            gate_blend_start=self.gate_blend_start, gate_blend_full=self.gate_blend_full,
            exit_blend_distance=self.gate_exit_blend_distance,
            half_width=self.gate_pass_half_width,acceleration=self.accel)
        self.z_ctrl = ZController(
            k_z=self.k_z, k_ff_z=self.k_ff_z, z_rate_max=self.z_rate_max,
            vz_up_limit=self.vz_up_limit, vz_down_limit=self.vz_down_limit,
            vz_accel_limit=self.vz_accel_limit,
            k_anticipate=self.k_anticipate, z_anticipate_dist=self.z_anticipate_dist)
        self.safety = SafetySupervisor(
            pose_timeout=self.pose_timeout, z_recovery_error=self.safety_z_error,
            z_recovery_gain=self.safety_z_gain, stuck_speed=self.stuck_speed,
            stuck_time=self.stuck_time, max_speed=self.max_speed,
            yaw_rate_max=self.yaw_rate_max)
        self.mission = MissionState(recon_speed=self.recon_speed,
                                    recon_max_dist=self.recon_max_dist,
                                    map_brake_a=self.map_brake_a)
        self.task_state = GateTaskState(
            half_width=self.gate_pass_half_width,
            half_height=self.gate_pass_half_height,
            miss_margin=self.gate_miss_margin, skip_s=self.gate_skip_s,
            miss_radius=self.gate_miss_radius)
        self.online_cache = OnlineGateCache(
            stable_frames=self.map_commit_stable_frames,
            xy_threshold=self.map_update_xy_threshold,
            z_threshold=self.map_update_z_threshold,
            s_threshold=self.map_update_s_threshold,
            static_correction_max=self.static_correction_max)
        self.blender = ProfileBlender(self.profile_switch_time)
        self.climb_feas = ClimbFeasibility(
            preview_time=self.climb_preview_time, preview_min=self.climb_preview_min,
            preview_max=self.climb_preview_max, preview_step=self.climb_preview_step,
            response_time=self.climb_response_time, eta=self.slope_eta)
        self.curve_env = CurveBrakingEnvelope(
            preview_time=self.curve_preview_time, preview_min=self.curve_preview_min,
            preview_max=self.curve_preview_max, preview_step=self.curve_preview_step,
            a_brake=self.curve_brake_a)

        self._lock = threading.RLock()
        self.start_z0 = None
        self.no_future_gate = True
        self.yaw_rate_prev = 0.0
        self.z_err_prev = None
        self.speed_info = {}
        self.v_target = 0.0
        self.last_pose_stamp = None
        self.last_stale_event = None
        self.race_goal = None
        self.race_goal_changed = False
        self.departure_aligned = False
        self.departure_turn_announced = False
        self.blocked_since = None
        self.last_perception_event = None

        self.route = self.load_route(self.route_file, self.route_name)
        self.gates = self.load_list(self.gates_file, "gates")
        self.static_gates = [dict(g) for g in self.gates]
        self.verified_ids = set(g.get("id") for g in self.static_gates)
        self.guides = self.load_list(self.guides_file, "altitude_guides")
        self.yaml_gate_z = {g.get("id"): g.get("z") for g in self.gates}
        self.planner = ReferencePlanner(
            self.route, start_gate=self.start_gate,
            gate_z_uses_offset=self.gate_z_uses_offset,
            snap_gate_to_route=self.snap_gate_to_route,
            gate_center_pull_max=self.gate_center_pull_max,
            slope_factor=self.slope_factor, slope_abs_max=self.slope_abs_max,
            sigma_z_max=self.sigma_z_max,
            startup_z_jump_limit=self.startup_z_jump_limit,
            corridor_half=self.corridor_half,
            gate_blend_start=self.gate_blend_start,
            gate_blend_full=self.gate_blend_full, z_rate_max=self.z_rate_max,
            z_extrap_m=self.z_extrap_m, z_extrap_slope_max=self.z_extrap_slope_max,
            z_soft_guide_max=self.z_soft_guide_max,
            z_soft_slope_max=self.z_soft_slope_max,corridor_guidance=self.adaptive_speed)

        rospy.loginfo("route '%s': %d pts, %.0f m; gates=%d",
                      self.route_name, len(self.route.points), self.route.total_s,
                      len(self.gates))
        self.cmd_pub = rospy.Publisher("/airsim_node/drone_1/vel_body_cmd", VelCmd, queue_size=1)
        self.event_pub = rospy.Publisher("/rmua/controller/events", String, queue_size=100, latch=True)
        self.telemetry_pub = rospy.Publisher("/rmua/controller/telemetry", String, queue_size=100)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.pose_cb,
                         queue_size=1,buff_size=1024*1024)
        rospy.Subscriber("/airsim_node/drone_1/lidar", PointCloud2,self.lidar_cb,queue_size=1,buff_size=4*1024*1024)
        rospy.Subscriber('/rmua/car_observations',String,self.car_cb,queue_size=1)
        rospy.Subscriber('/airsim_node/end_goal',PoseStamped,self.race_goal_cb,queue_size=1)
        if self.use_gate_map:
            rospy.Subscriber(self.gate_map_topic, String, self.gate_map_cb,queue_size=1)
            rospy.loginfo("[GateMap] ENABLED topic=%s min_support=%d (静态 gates 作为初始/兜底)",
                          self.gate_map_topic, self.gate_map_min_support)
        else:
            rospy.logwarn("[WARN] Persistent Gate Map disabled -> 使用静态 gates_file: %s",
                          self.gates_file)
        rospy.loginfo("[Z] z_rate_max=%.1f vz_up_limit=%.1f vz_accel_limit=%.1f Kz=%.2f "
                      "Kff_z=%.2f vz_capability=%s", self.z_rate_max, self.vz_up_limit,
                      self.vz_accel_limit, self.k_z, self.k_ff_z, self.vz_cap.as_list())
        rospy.loginfo("[PREVIEW] climb T=%.1f H=[%.0f,%.0f] step=%.1f resp=%.2f | "
                      "curve T=%.1f H=[%.0f,%.0f] a_brake=%.1f",
                      self.climb_preview_time, self.climb_preview_min,
                      self.climb_preview_max, self.climb_preview_step,
                      self.climb_response_time, self.curve_preview_time,
                      self.curve_preview_min, self.curve_preview_max, self.curve_brake_a)
        rospy.Timer(rospy.Duration(self.dt), self.control_loop)

    # ---------- load ----------
    @staticmethod
    def load_route(path, name):
        with open(path) as f:
            pts = yaml.safe_load(f)["routes"][name]
        return RouteGeometry(pts)

    @staticmethod
    def load_list(path, key):
        if not path or not os.path.exists(path):
            return []
        with open(path) as f:
            d = yaml.safe_load(f)
        return d.get(key, []) if d else []

    # ---------- init / rebuild ----------
    def init_once(self, p):
        i, t, d, c = self.route.project((p.x, p.y, p.z))
        self.seg = i
        self.z_offset = p.z - c[2]
        self.start_z0 = p.z
        s_now = self.route.seg_s[i] + t * self.route.seg_len[i]
        with self._lock:
            chain, profile = self.planner.build(
                self.static_gates, self.guides, self.soft_guides, s_now, p.z,
                z_offset=self.z_offset,
                corrections=self.online_cache.static_corrections,
                verified_ids=self.verified_ids, start_anchor_z=self.start_anchor_z)
            self.chain, self.profile = chain, profile
            self.blender.set_initial(profile, self.pose_stamp)
            self.xy_tracker.configure(self.route, chain.gates, s_now,stamp=self.pose_stamp)
            self.gate_idx = self.planner.initial_index(chain, s_now)
            for g in chain.gates[:self.gate_idx]:
                self.task_state.resolved_gate_ids.add(g.get("id"))
        rospy.loginfo("init seg=%d z_offset=%.2f gates=%d suspects=%s start_gate=%d",
                      i, self.z_offset, len(chain.gates), chain.suspect_ids(), self.gate_idx)
        zp = profile.center(s_now)
        rospy.loginfo("[STARTUP] s=%.1f current_z=%.2f profile_z(s)=%.2f jump=%.2f",
                      s_now, p.z, zp, abs(zp - p.z))
        for g in chain.gates:
            rospy.loginfo("[GATE] id=%s s=%.1f z=%.2f src=%s class=%s trusted=%s "
                          "sigma_z=%.2f support=%s z_suspect=%s",
                          g.get("id"), g["s"], g["z"], g.get("source", "static_yaml"),
                          g.get("anchor_class", "-"), g.get("trusted", True),
                          g.get("sigma_z", 0.0), g.get("support", "-"),
                          g.get("z_suspect", False))
        if self.trace_gate >= 0:
            try:
                self.trace_fh = open(self.trace_file, "w")
                self.trace_fh.write(
                    "t,s,gate_id,used_z,yaml_z,source,hard_anchor,sigma_z,support,"
                    "z_ref_raw,z_ref,z_actual,dzds,dzds_prev,vz_ff,vz_fb,vz_target,"
                    "vz_cmd,roll,pitch,yaw,prev_anchor_s,prev_anchor_z,next_anchor_s,"
                    "next_anchor_z,mode,v_curve_preview,v_climb_preview,dt\n")
                rospy.loginfo("[TRACE] gate %d -> %s", self.trace_gate, self.trace_file)
            except IOError as e:
                rospy.logwarn("[TRACE] cannot open %s: %s", self.trace_file, e)

    def gate_map_cb(self, msg):
        """持久 Gate Map 更新 (方案 7 节: Perception/Planning 解耦)。"""
        try:
            gates = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(gates, list):
            return
        # Dense bbox tracks carry a tangent from the vision node's route.
        # That route can have the opposite direction on the next race leg.
        # Only measured plane geometry may keep its own normal.
        gates = [dict(g) for g in gates]
        for g in gates:
            if not g.get('geometry_valid', False):
                for key in ('nx', 'ny', 'nz'):
                    g.pop(key, None)
        with self._lock:
            if self.pose is None or self.chain is None:
                return
            p = self.pose.position
            i, t, d, c = self.route.project((p.x, p.y, p.z))
            s_now = self.route.seg_s[i] + t * self.route.seg_len[i]
            now = self.pose_stamp if self.pose_stamp is not None else 0.0
            soft_changed = self.guide_cache.update(
                gates, now, s_now, lambda g: self.route.project_gate(g["x"],g["y"]),
                min_support=4 if self.planner.height_prior.valid else 6)
            self.soft_guides = self.guide_cache.guides
            self.soft_horizon_s = self.guide_cache.horizon
            self.soft_horizon_stamp = now
            static_s = [self.route.project_gate(g["x"], g["y"]) for g in self.static_gates]
            hard_changed = self.online_cache.ingest(
                    gates, static_s, s_now, now,
                    lambda g: self.route.project_gate(g["x"], g["y"]),
                    self.task_state.resolved_gate_ids,
                    static_gates=self.static_gates)
            if not hard_changed and not soft_changed:
                return
            merged_gates = self.static_gates + self.online_cache.gates
            p0z = self.start_z0 if self.start_z0 is not None else p.z
            chain, profile = self.planner.build(
                merged_gates, self.guides, self.soft_guides, s_now, p0z,
                z_offset=self.z_offset,
                corrections=self.online_cache.static_corrections,
                verified_ids=self.verified_ids, start_anchor_z=self.start_anchor_z)
            idx = self.planner.resolve_index(chain, self.task_state.resolved_gate_ids)
            self.chain, self.profile, self.gate_idx = chain, profile, idx
            self.blender.switch(profile, self.pose_stamp)
            self.xy_tracker.configure(self.route, chain.gates, s_now,stamp=self.pose_stamp)
        rospy.loginfo_throttle(
            2.0, "GATE_MAP update: static=%d +online=%d -> %d gates, next_idx=%d, "
            "static_corrections=%d", len(self.static_gates),
            len(self.online_cache.gates), len(merged_gates), self.gate_idx,
            len(self.online_cache.static_corrections))

    # ---------- callbacks ----------
    def race_goal_cb(self,msg):
        p=msg.pose.position
        goal=np.array([p.x,p.y,p.z])
        with self._lock:
            if self.race_goal is None:
                # A later leg must match its own published endpoint.
                if np.linalg.norm(goal[:2]-np.array(self.route.points[-1][:2]))<35.:
                    self.race_goal=goal
            elif np.linalg.norm(goal-self.race_goal)>20.:
                self.race_goal_changed=True

    def lidar_cb(self,msg):
        fields={f.name:f for f in msg.fields}
        if any(k not in fields or fields[k].datatype!=7 for k in ('x','y','z')):return
        endian='>' if msg.is_bigendian else '<'
        dtype=np.dtype(dict(names=['x','y','z'],formats=[endian+'f4']*3,
                            offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
        raw=np.frombuffer(msg.data,dtype=dtype)
        points=np.column_stack([raw[k] for k in ('x','y','z')])
        points=valid_points(points)
        stamp=msg.header.stamp.to_sec()
        with self._lock:
            self.pending_cloud=(stamp,points)
            self._transform_cloud()

    def _transform_cloud(self):
        if self.pending_cloud is None:return
        stamp,points=self.pending_cloud
        pose=self.pose_history.sample(stamp)
        if pose is None:return
        p,q=pose;R=tft.quaternion_matrix(q)[:3,:3]
        points=points.copy();points[:,2]-=.05
        world=points@R.T+p
        self.raw_cloud=points
        self.cloud_history=[(t,c) for t,c in self.cloud_history if 0.<=stamp-t<.15]
        self.cloud_history.append((stamp,world))
        self.lidar_points=np.concatenate([c for t,c in self.cloud_history],axis=0)
        self.lidar_stamp=stamp
        self.pending_cloud=None

    def car_cb(self,msg):
        try:
            data=json.loads(msg.data)
            with self._lock:self.car_tracks.update(float(data['stamp']),data.get('detections',[]))
        except (ValueError,TypeError,KeyError):
            rospy.logwarn_throttle(5.,'Invalid car observation')

    def pose_cb(self, m):
        arrived=rospy.Time.now().to_sec()
        with self._lock:
            # 模拟器 pose 时间戳可能落后 ros::Time (仿真时基), dt 用 header stamp,
            # 但 timeout 必须用回调到达的 wall clock。
            self.pose_stamp = m.header.stamp.to_sec()
            self.pose_arrival = arrived
            self.pose = m.pose
            q = m.pose.orientation
            p=m.pose.position
            self.pose_history.add(self.pose_stamp,(p.x,p.y,p.z),(q.x,q.y,q.z,q.w))
            self._transform_cloud()
            self.roll, self.pitch, self.yaw = tft.euler_from_quaternion(
                [q.x, q.y, q.z, q.w])

    def _on_teleport(self, s_now):
        """瞬移/reset 后清空任务账本与在线地图, 否则会把旧门当已通过。"""
        rospy.logwarn("[TELEPORT] s %.1f -> %.1f; 重置任务账本与在线地图",
                      self.prev_s, s_now)
        self.task_state.reset()
        self.online_cache = OnlineGateCache(
            stable_frames=self.map_commit_stable_frames,
            xy_threshold=self.map_update_xy_threshold,
            z_threshold=self.map_update_z_threshold,
            s_threshold=self.map_update_s_threshold,
            static_correction_max=self.static_correction_max)
        self.soft_horizon_s = None
        self.soft_horizon_stamp = None
        self.guide_cache = PersistentGuides()
        self.soft_guides = []
        self.mission.reset()
        self.safety.reset()
        self.z_ctrl.reset()
        self.motion.reset()
        self.speed_sched.reset()
        self.xy_tracker.reset()
        self.lidar_points = None
        self.pending_cloud=None
        self.cloud_history=[]
        self.car_tracks=CarTracks()
        self.planning.reset()
        self.predictive=PredictiveAvoidance(margin=1.15,braking=self.lidar_braking,budget=1.5,vertical_limit=1.5)
        self.executing_plan=None
        self.execution_guard=ExecutionGuard(margin=1.15,braking=min(self.lidar_braking,float(self.accel),self.a_down,self.curve_brake_a),
                                             reaction=self.sensor_reaction,horizon=self.lidar_range)
        self.planner_info={}
        self.plan_sequence=0
        self.shift_target=None
        self.shift_retreat=False
        self.retreat_pending=False
        self.shift_checked=None
        self.clearance_z = self.clearance_y = 0.
        self.clearance_checked = None
        self.chain = None
        self.profile = None
        self.gate_idx = 0
        self.seg = None
        self.prev_s = None
        self.prev_pose = None
        self.reached = False
        self.start_z0 = None
        self.v_cmd_prev = 0.0
        self.last_pose_stamp = None
        self.pose_arrival = None

    def event(self, kind, **fields):
        """Controller diagnostics only; never independent race scoring."""
        self.event_sequence += 1
        payload = dict(schema_version=1, source="controller", sequence=self.event_sequence,
                       stamp=rospy.Time.now().to_sec(), pose_stamp=self.pose_stamp, kind=kind)
        payload.update(fields)
        self.event_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def shutdown(self):
        self.stopping = True
        self.publish(0.0, 0.0, 0.0, 0.0)
        self.event("TERMINATION", reason="ROS_SHUTDOWN", controller_reached=self.reached)
        self.planning.close()
        if self.trace_fh is not None:
            self.trace_fh.close()

    def publish(self, vx, vy, vz, yaw_rate=0.0):
        if self.stopping:
            vx = vy = vz = yaw_rate = 0.0
        c = VelCmd()
        c.header.stamp = rospy.Time.now()
        c.header.frame_id = "drone_1"
        c.vx, c.vy, c.vz = vx, vy, vz
        # 模拟器 VelCmd.yawRate 单位是 度/秒 (官方示例注释 "yaw, deg");
        # 控制器内部用 rad/s, 这里统一换算后下发。
        c.yawRate = math.degrees(yaw_rate)
        c.va = self.accel
        c.stop = 0
        self.cmd_pub.publish(c)

    def _gate_metrics(self, ng, pn):
        """门平面几何: (d_g, lat, vert, e_n); ng=None 时返回 d_g=1e9。"""
        if ng is None:
            return 1e9, 0.0, 0.0, 0.0
        G = np.array([ng["x"], ng["y"], ng["z"]])
        n = np.array([ng.get("nx", 1.0), ng.get("ny", 0.0), ng.get("nz", 0.0)])
        n = n / (np.linalg.norm(n) + 1e-9)
        r = pn - G
        e_n = float(r @ n)
        vert = float(r[2] - e_n * n[2])
        lat = float(math.hypot(r[0] - e_n * n[0], r[1] - e_n * n[1]))
        return float(np.linalg.norm(r)), lat, vert, e_n

    def _gate_half_extents(self, ng):
        """门洞半宽/半高: 优先用 Gate Map 的四角尺寸, 否则退回容差参数。"""
        hw = self.gate_pass_half_width
        hh = self.gate_pass_half_height
        if hw <= 0.0:
            w = ng.get("width")
            hw = 0.5 * float(w) if w else self.aperture_xy_tol
        if hh <= 0.0:
            h = ng.get("height")
            hh = 0.5 * float(h) if h else self.aperture_z_tol
        return hw, hh

    def _predict_gate_miss(self, ng, pn, dt):
        """用实际世界速度预测到门平面时的穿门偏差比 max(lat/hw, |vert|/hh)。
        返回 None 表示不接近/无有效预测 (不因门限速)。(方案 3~5)"""
        if ng is None or self.prev_pose is None or dt <= 0:
            return None
        v_world = (pn - np.asarray(self.prev_pose, dtype=float)) / dt
        G = np.array([ng["x"], ng["y"], ng["z"]])
        n = np.array([ng.get("nx", 1.0), ng.get("ny", 0.0), ng.get("nz", 0.0)])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            return None
        n = n / nn
        e_n = float((pn - G) @ n)
        closing = float(v_world @ n)
        if closing <= 0.2:
            return None
        tt = -e_n / closing
        # Only a near-plane collision guard. Multi-second ballistic projection
        # ignores the commanded curve/climb and brakes after every task-gate switch.
        if tt < 0.0 or tt > 0.6:
            return None
        pp = pn + v_world * tt
        r = pp - G
        rn = float(r @ n)
        vert_p = float(r[2] - rn * n[2])
        lat_p = float(math.hypot(r[0] - rn * n[0], r[1] - rn * n[1]))
        hw, hh = self._gate_half_extents(ng)
        return max(lat_p / max(hw, 1e-3), abs(vert_p) / max(hh, 1e-3))

    def _trace_z(self, s_now, ng, z_raw, z_ref, z_act, z_err, dzds, dzds_prev,
                 vz_ff, vz_fb, vz_tgt, vz_clamp, vz_cmd, z_limit, a_prev, a_next,
                 mode, v_curve_preview, v_climb_preview, dt):
        """输出单个 Gate 的完整 Z 链路 (方案 30/31)。"""
        gid = ng.get("id")
        yaml_z = self.yaml_gate_z.get(gid)
        gid_v = gid if gid is not None else -1
        src = ng.get("source", "static_yaml")
        hard = ng.get("hard_anchor", True)
        sigz = ng.get("sigma_z", 0.0)
        if self.trace_fh is not None:
            try:
                self.trace_fh.write("%.4f,%.2f,%s,%.3f,%s,%s,%s,%.3f,%s,%.3f,%.3f,"
                                    "%.3f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,"
                                    "%.2f,%.3f,%.2f,%.3f,%s,%.2f,%.2f,%.3f\n"
                                    % (rospy.Time.now().to_sec(), s_now, gid_v, ng.get("z", 0.0),
                                       ("%.3f" % yaml_z) if yaml_z is not None else "-", src,
                                       hard, sigz, str(ng.get("support", "-")), z_raw, z_ref,
                                       z_act, dzds, dzds_prev, vz_ff, vz_fb, vz_tgt, vz_cmd,
                                       self.roll, self.pitch, self.yaw,
                                       a_prev[0], a_prev[1], a_next[0], a_next[1],
                                       mode, v_curve_preview, v_climb_preview, dt))
                self.trace_fh.flush()
            except Exception:
                pass
        rospy.loginfo_throttle(
            0.2,
            "[G%s] s=%.1f used_z=%.2f yaml_z=%s src=%s hard=%s sigz=%.2f dzds=%.4f "
            "dzds_prev=%.4f | z_raw=%.2f z_ref=%.2f z=%.2f err=%.2f | ff=%.2f fb=%.2f "
            "tgt=%.2f clamp=%.2f cmd=%.2f LIMIT=%s | anchor %.1f->%.1f",
            gid_v, s_now, ng.get("z", 0.0),
            ("%.2f" % yaml_z) if yaml_z is not None else "-", src, hard, sigz,
            dzds, dzds_prev, z_raw, z_ref, z_act, z_err, vz_ff, vz_fb, vz_tgt,
            vz_clamp, vz_cmd, z_limit, a_prev[0], a_next[0])

    # ---------- main ----------
    def control_loop(self, _e):
        with self._lock:
            return self._control_loop(_e)

    def _plan_avoidance(self,planner,s,p,v,xy,center,route,points,cars,stamp,speed,gates,distance,offset):
        initial=float(np.min(np.linalg.norm(points-p,axis=1))) if len(points) else float('inf')
        # Do not spend a full route search attempting to depart from inside
        # an existing buffer. First certify a short non-decreasing escape.
        if initial<self.clearance.margin:
            info=dict(active=True,feasible=False,cap=0.,source='PREDICTIVE',initial_clearance=initial)
            distance=min(distance,12.)
        else:info=planner.evaluate(s,p,v,xy.base_point_at,center,route,points,cars,stamp,speed,gates)
        if not info.get('feasible',False):
            reference=[]
            for ahead in np.linspace(0.,distance,max(3,int(distance/3.)+1)):
                ex,ey,_=xy.base_point_at(s+ahead)
                reference.append((ex-p[0],ey-p[1],center(s+ahead)-p[2]))
            fallback=self.clearance.evaluate_path(points-p,reference,offset)
            if fallback.get('active',False) and fallback.get('feasible',False):
                a,b=np.asarray(xy.base_point_at(s-.3)[:2]),np.asarray(xy.base_point_at(s+.3)[:2])
                forward=(b-a)/max(1e-6,np.linalg.norm(b-a))
                target=np.array([*xy.base_point_at(s)[:2],center(s)])
                target[:2]+=fallback['lateral']*np.array([-forward[1],forward[0]])
                target[2]+=fallback['vertical']
                info=dict(fallback,source='LIDAR_RECOVERY',recovery_target=target.tolist());planner.plan=None
        if (info.get('active') and self.debug_cloud_dir and
                (self.debug_cloud_stamp is None or stamp-self.debug_cloud_stamp>1.)):
            self.debug_cloud_stamp=stamp
            reference=np.array([[*xy.base_point_at(s+d)[:2],center(s+d)] for d in np.arange(0.,66.,1.)])
            name=os.path.join(self.debug_cloud_dir,'%.3f_s%.1f'%(stamp,s))
            np.savez_compressed(name+'.npz',points=points,position=p,reference=reference)
            with open(name+'.json','w') as file:
                json.dump(dict(clearance=info,s=s,pose_stamp=stamp,cars=[dict(world=t['world'].tolist(),
                          stamp=t['stamp'],uncertainty=t['uncertainty'],half_width=t['half_width'],
                          half_height=t['half_height']) for t in cars],
                          plan=(dict(stations=(s+np.arange(66.)).tolist(),
                                     offsets=planner.plan.offsets(s+np.arange(66.)).tolist(),
                                     start_slope=planner.plan.slope(s).tolist(),end=planner.plan.end)
                                if planner.plan is not None else None)),file)
        return info,planner.plan

    def _control_loop(self, _e):
        now = rospy.Time.now().to_sec()
        with self._lock:
            if self.pose is None or self.stopping:
                return
            p, pose_stamp = self.pose.position, self.pose_stamp
            # Keep pose and cloud from one callback snapshot. Reading a newer
            # cloud after releasing the lock creates a negative cloud age.
            lidar_points, lidar_stamp = self.lidar_points, self.lidar_stamp
            car_tracks=self.car_tracks.snapshot(pose_stamp)
        if self.reached or self.safety.aborted or self.mission.mode == ABORT:
            self.publish(0.0, 0.0, 0.0, 0.0)
            return
        if self.race_goal_changed:
            self.publish(0.0,0.0,0.0,0.0)
            return

        # ---- pose timeout: 立即输出零速度 (方案 22.2) ----
        if pose_stamp is None or self.safety.pose_stale(now, self.pose_arrival):
            self.publish(0.0, 0.0, 0.0, 0.0)
            if self.last_stale_event is None or now - self.last_stale_event > 1.0:
                self.last_stale_event = now
                self.event("POSE_STALE", pose_age=(now - self.pose_arrival)
                           if self.pose_arrival is not None else None)
                rospy.logwarn_throttle(1.0, "POSE_STALE: no fresh pose for > %.2f s",
                                       self.pose_timeout)
            return

        # ---- 真实 dt: 用 pose timestamp, 不用固定 1/rate (方案 22) ----
        if pose_stamp is not None and self.last_pose_stamp is not None:
            if pose_stamp == self.last_pose_stamp:
                return
            motion_dt=max(.001,pose_stamp-self.last_pose_stamp)
            dt = min(.2,motion_dt)
        else:
            dt = self.dt
            motion_dt=dt
        if pose_stamp is not None:
            self.last_pose_stamp = pose_stamp
        self.execution_guard.set_faces(measured_car_faces(car_tracks,self.route,pose_stamp))

        # Establish flight height before constructing the departure profile.
        # The initial spawn origin may settle below the rendered road surface;
        # treating that floor as an arbitrary obstacle prevents takeoff.
        # This applies only at the official origin, never at an in-course car.
        if (self.seg is None and math.hypot(*self.route.points[0][:2])<3.
                and math.hypot(p.x,p.y)<3. and p.z> -1.65):
            takeoff=self.z_ctrl.recon(p.z,0.,1.8,min(1.5,max(.4,2.*(p.z+1.8))),dt)
            self.publish(0.,0.,takeoff['vz_cmd'],0.)
            return

        if self.seg is None:
            self.z_ctrl.reset()
            self.init_once(p)
        pn = np.array([p.x, p.y, p.z])

        i, t, d_cross, c = self.route.project((p.x, p.y, p.z))
        self.seg = i
        s_now = self.route.seg_s[i] + t * self.route.seg_len[i]

        # 瞬移/reset 检测: 清空已完成账本, 支持途中复位后重飞
        if self.prev_s is not None and s_now < self.prev_s - 20.0:
            self._on_teleport(s_now)
            return

        chain, profile = self.chain, self.profile
        p_prev = np.asarray(self.prev_pose, dtype=float) if self.prev_pose is not None else pn
        measured_progress,measured_up=self.motion.update(p_prev,pn,motion_dt,self.route.tangent(s_now))
        self.xy_tracker.stamp=pose_stamp

        # ---- Gate Task State: 真实 Gate plane crossing + 账本分离 (方案 15, 16) ----
        gate_idx, gate_events = self.task_state.step(chain.gates, self.gate_idx, p_prev, pn, s_now)
        self.gate_idx = gate_idx
        for ev in gate_events:
            rospy.loginfo("GATE %s %s (lat=%s vert=%s basis=%s)",
                          ev.get("gate_id"), "PASSED" if ev["status"] == "PASS"
                          else ("MISSED" if ev["status"] == "MISS" else "SKIPPED"),
                          ("%.2f" % ev["lateral_error"]) if ev["lateral_error"] is not None else "-",
                          ("%.2f" % ev["vertical_error"]) if ev["vertical_error"] is not None else "-",
                          ev.get("basis"))
            if ev.get("basis") == "gate_behind_route_progress":
                rospy.logwarn("%s gate %s (s=%.1f, s_now=%.1f)",
                              "SKIP" if ev["status"] == "SKIP" else "MISSED",
                              ev.get("gate_id"), ev.get("gate_s", -1.0), s_now)
            self.event("GATE", pose_stamp=pose_stamp, s=s_now, **ev)

        ng = chain.gates[self.gate_idx] if self.gate_idx < len(chain.gates) else None
        d_g, lat, vert, e_n = self._gate_metrics(ng, pn)

        # ---- 未来门判定 + NO_FUTURE_GATE 保护 ----
        v_s = max(self.v_cmd_prev, 0.5)
        future_gates = [g for g in chain.gates[self.gate_idx:] if g["s"] > s_now + 1.0]
        self.no_future_gate = (ng is None) or (len(future_gates) == 0)

        # ---- Z 前视 Horizon ----
        # Only compensate short actuator response; the climb profile already anticipates gates.
        Lz = min(2.0, self.z_response_time * (measured_progress if self.adaptive_speed else v_s))
        z_horizon = max(self.z_horizon_min, min(self.z_horizon_max, self.z_horizon_time*v_s))
        s_ff = s_now + Lz
        kz_prev = self.blender.dz_ds(s_ff, stamp=pose_stamp)

        # ---- Yaw Path Following: 提前看向未来赛道 (方案 18) ----
        yaw_calc = 0.0
        yaw_target = self.yaw
        yaw_err = 0.0
        e_img = 0.0
        yaw_source = "HOLD"
        if self.yaw_control:
            yaw_source = "CONTINUOUS_PATH"
            rx, ry, _ = self.xy_tracker.point_at(s_now + self.yaw_lookahead)
            tgt = self.yaw_ctrl.target_from_route((p.x,p.y), (rx,ry))
            yaw_calc, yaw_target, yaw_err = self.yaw_ctrl.rate(
                (p.x, p.y), self.yaw, tgt)
            if self.k_vision != 0.0 and future_gates:
                R_wb = tft.euler_matrix(self.roll, self.pitch, self.yaw)[:3, :3]
                gw = [(g["x"], g["y"], g["z"]) for g in future_gates[:self.fov_n_gates]]
                e_img = self.yaw_ctrl.vision_fov_error(
                    gw, np.array([p.x, p.y, p.z]), R_wb, self.fov_n_gates)
        yaw_rate_target = max(-self.yaw_rate_max,
                              min(self.yaw_rate_max, yaw_calc + self.k_vision * e_img))
        dyr = self.yaw_accel_limit * dt
        yaw_rate = max(self.yaw_rate_prev - dyr,
                       min(self.yaw_rate_prev + dyr, yaw_rate_target))
        self.yaw_rate_prev = yaw_rate

        # A return leg begins facing away from the road. Rotate before moving,
        # so the forward cameras observe the new obstacles before departure.
        if not self.departure_aligned:
            departure_z=self.departure_hover_z if self.departure_hover_z> -900. else self.start_z0
            settle=(self.departure_hover_z> -900. and abs(p.z-departure_z)>.25)
            if (self.yaw_control and abs(yaw_err) > math.radians(25.)) or settle:
                if not self.departure_turn_announced:
                    self.event('TURN_AROUND', pose_stamp=pose_stamp,
                               yaw_error_deg=math.degrees(yaw_err))
                    self.departure_turn_announced = True
                hold=self.z_ctrl.track(departure_z,p.z,0.,0.,dt)
                command=np.array([0.,0.,-hold['vz_cmd']])
                if self.adaptive_speed:
                    command,_=self.execution_guard.filter_command(pn,self.motion.velocity,command,
                        lidar_points,lidar_stamp,pose_stamp)
                    self.z_ctrl.prev_vz=-float(command[2])
                self.prev_pose=(p.x,p.y,p.z)
                self.publish(0., 0.,-float(command[2]), yaw_rate)
                return
            self.departure_aligned = True
            if self.departure_turn_announced:
                self.event('TURN_AROUND_COMPLETE', pose_stamp=pose_stamp)
            if self.departure_hover_z> -900.:
                # Rebuild the departure anchor at the settled height. Reusing
                # the old anchor would pull the aircraft back into the sign.
                self.seg=None;self.z_prev=None;self.prev_pose=None
                self.motion.reset();self.z_ctrl.reset()
                self.publish(0.,0.,0.,yaw_rate)
                return

        # ---- 高度误差 / 预测 tracking ----
        base_center = lambda s: self.blender.center(s, pose_stamp)
        rx,ry,_=self.xy_tracker.base_point_at(s_now)
        a,b=self.xy_tracker.base_point_at(s_now-.3),self.xy_tracker.base_point_at(s_now+.3)
        length=max(1e-6,math.hypot(b[0]-a[0],b[1]-a[1]));fx,fy=(b[0]-a[0])/length,(b[1]-a[1])/length
        current_lateral=-(p.x-rx)*fy+(p.y-ry)*fx
        current_vertical=p.z-base_center(s_now)
        cloud_age=pose_stamp-lidar_stamp if lidar_stamp is not None else None
        cloud_fresh=lidar_points is not None and 0. <= cloud_age < .5
        measured_vxy=float(np.linalg.norm((pn-p_prev)[:2]))/motion_dt
        available_cruise=self.speed_sched.set_visibility(
            self.lidar_range if cloud_fresh else 8.,cloud_age if cloud_fresh else .5)
        if self.adaptive_speed:
            # Finish the currently certified discrete move. Applying a
            # concurrent spatial plan here repeatedly moved its goal away.
            completed=self.planning.poll() if self.shift_target is None else None
            if completed is not None:
                info,candidate=completed
                self.planner_info=info
                if info.get('feasible') and info.get('source')=='PREDICTIVE':
                    old=self.executing_plan
                    if candidate is not old:
                        start=old.offset(s_now) if old is not None else np.array([current_lateral,current_vertical])
                        if old is not None:
                            slope=old.slope(s_now)
                            accel=(old.slope(s_now+.1)-old.slope(s_now-.1))/.2
                        else:
                            progress=max(3.,float(self.motion.velocity[:2]@np.array([fx,fy])))
                            slope=np.array([self.motion.velocity[:2]@np.array([-fy,fx])/progress,
                                            self.motion.velocity[2]/progress-(base_center(s_now+.5)-base_center(s_now-.5))])
                            accel=np.zeros(2)
                        needs_join=candidate is not None or np.linalg.norm(start)>.02
                        lengths=[max(6.,min(12.,measured_vxy*.6)),4.,2.,1.]
                        accepted=None;join_safe=False
                        for length in (lengths if needs_join else [0.]):
                            joined=(JoinedDetour(s_now,start,slope,accel,candidate,length) if needs_join else None)
                            if self.departure_hover_z> -900. and s_now<20.:
                                stations=np.arange(s_now,20.1,.2)
                                entry=PredictiveAvoidance.positions(stations,self.xy_tracker.base_point_at,base_center,joined)
                                floor_limit=np.array([self.departure_hover_z+.25+base_center(t)-base_center(0.) for t in stations])
                                if self.execution_guard.index is not None:
                                    measured_floor=self.execution_guard.index.floor_limit(entry,self.execution_guard.margin+.1)
                                    supported=np.isfinite(measured_floor)
                                    floor_limit[supported]=measured_floor[supported]
                                if np.any(entry[:,2]>floor_limit):continue
                            verified=self.execution_guard.evaluate(s_now,pn,self.motion.velocity,
                                self.xy_tracker.base_point_at,base_center,joined,lidar_points,lidar_stamp,pose_stamp)
                            if (verified.get('execution_verified') and
                                    (verified.get('execution_clearance') or 0.)>=self.execution_guard.margin+.1):
                                accepted=joined;join_safe=True;break
                        if join_safe:
                            self.executing_plan=accepted
                            self.shift_target=None
                            self.shift_retreat=False
                            self.plan_sequence+=1
                            self.planner_info=dict(info,applied_pose_stamp=pose_stamp,applied_monotonic=time.monotonic(),join_accepted=True)
                        else:self.planner_info=dict(info,join_accepted=False)
                    else:self.planner_info=dict(info,join_accepted=True)
        if cloud_fresh:
            if self.clearance_checked is None or pose_stamp-self.clearance_checked >= .2:
                submitted=True
                distance=min(self.lidar_range,self.clearance.preview_distance(
                    max(v_s,self.v_cmd_prev,measured_vxy),braking=self.lidar_braking,latency=self.sensor_reaction))
                if self.adaptive_speed:
                    xy=copy.copy(self.xy_tracker);xy.offset_blender=copy.copy(self.xy_tracker.offset_blender)
                    blender=copy.copy(self.blender)
                    center=lambda s,b=blender,t=pose_stamp:b.center(s,t)
                    planner=copy.copy(self.predictive);planner.plan=self.executing_plan
                    job=lambda s=s_now,p=pn.copy(),v=self.motion.velocity.copy(),x=xy,b=center,r=self.route,cloud=lidar_points,c=car_tracks,t=pose_stamp,sp=max(v_s,measured_vxy),g=list(future_gates),d=distance,offset=(current_lateral,current_vertical),planner=planner:self._plan_avoidance(planner,s,p,v,x,b,r,cloud,c,t,sp,g,d,offset)
                    submitted=self.planning.submit(pose_stamp,job)
                else:
                    reference=[]
                    for ahead in np.linspace(0.,distance,max(3,int(distance/3.)+1)):
                        ex,ey,_=self.xy_tracker.base_point_at(s_now+ahead)
                        reference.append((ex-p.x,ey-p.y,base_center(s_now+ahead)-p.z))
                    self.clearance_info=self.clearance.evaluate_path(lidar_points-pn,reference,(current_lateral,current_vertical))
                if submitted:self.clearance_checked=pose_stamp
        elif self.clearance_info.get('active', False):
            # A delayed cloud cannot authorize acceleration into a surface
            # that the last fresh cloud said was blocking the path.
            self.clearance_info=dict(self.clearance_info,cap=0.,stale=True)
        else:
            self.clearance_info=dict(cap=None,active=False)
        if self.adaptive_speed:
            guard=self.execution_guard.evaluate(s_now,pn,self.motion.velocity,
                self.xy_tracker.base_point_at,base_center,self.executing_plan,
                lidar_points,lidar_stamp,pose_stamp)
            planned=self.planner_info.get('planned_stamp')
            age=None if planned is None else pose_stamp-planned
            cap=self.execution_guard.route_cap(guard,self.planner_info)
            if self.retreat_pending:
                if self.execution_guard.resume_after_retreat(guard,self.planner_info):
                    self.retreat_pending=False
                else:cap=0.
            # Plan age diagnoses search lag. Latest geometry determines the
            # braking cap; a slow future never drops an executing reference.
            self.clearance_info=dict(self.planner_info,**guard)
            self.clearance_info.update(cap=cap,source='PREDICTIVE',feasible=guard['execution_verified'],
                active=self.executing_plan is not None or guard['guard_reason']!='EXECUTION_CLEAR',
                stale=guard['guard_reason']=='LIDAR_STALE',plan_stale=age is None or age>.7,
                plan_age=age,plan_id=self.plan_sequence,planner_busy=self.planning.future is not None,
                planner_feasible=self.planner_info.get('feasible'),
                lateral=float(self.executing_plan.offset(s_now)[0]) if self.executing_plan is not None else 0.,
                vertical=float(self.executing_plan.offset(s_now)[1]) if self.executing_plan is not None else 0.)
            if self.executing_plan is not None:
                plan=self.executing_plan
                self.clearance_info.update(executing_start=plan.s,executing_end=plan.end,
                    executing_slope=plan.slope(s_now).tolist(),
                    terminal_offset=plan.offset(plan.end).tolist(),terminal_slope=plan.slope(plan.end).tolist())
        active=self.clearance_info.get('active',False)
        predictive_path=(self.adaptive_speed or (self.clearance_info.get('source')=='PREDICTIVE' and
                         self.clearance_info.get('feasible',False)))
        self.xy_tracker.detour=self.executing_plan if predictive_path else None
        if active and not self.clearance_info.get('feasible',False):
            if self.blocked_since is None:self.blocked_since=pose_stamp
            if (pose_stamp-self.blocked_since>3. and
                    (self.last_perception_event is None or pose_stamp-self.last_perception_event>15.)):
                self.event('PERCEPTION_REQUIRED',pose_stamp=pose_stamp,s=s_now,
                           position=[p.x,p.y,p.z],reason=self.clearance_info.get('guard_reason','NO_SAFE_LIDAR_CORRIDOR'))
                self.last_perception_event=pose_stamp
        else:
            self.blocked_since=None
        blend=1.-math.exp(-dt/(.25 if active else 1.5))
        target_z=self.clearance_info.get('vertical',0.)
        target_y=self.clearance_info.get('lateral',0.)
        if active and (not self.clearance_info.get('feasible',False) or self.clearance_info.get('stale',False)):
            target_z=current_vertical
            target_y=current_lateral
        self.clearance_z+=blend*(target_z-self.clearance_z)
        self.clearance_y+=blend*(target_y-self.clearance_y)
        if predictive_path:
            plan=self.executing_plan
            self.clearance_y,self.clearance_z=(plan.offset(s_now) if plan is not None else (0.,0.))
            center_fn=lambda s:base_center(s)+(float(plan.offset(s)[1]) if plan is not None else 0.)
        else:center_fn = lambda s: base_center(s)+self.clearance_z
        terminal = self.race_goal is not None and s_now > self.route.total_s-25.
        if self.race_goal is not None and s_now > self.route.total_s-45.:
            old_center=center_fn
            def center_fn(s):
                blend=max(0.,min(1.,(s-self.route.total_s+45.)/25.))
                # Published marker locations are at road level, like the
                # initial pose. Enter the 5m-high trigger above that surface.
                target=min(self.race_goal[2]-self.terminal_hover_height,old_center(s)+1.)
                return (1.-blend)*old_center(s)+blend*target
        kz_prev=center_fn(s_ff+.5)-center_fn(s_ff-.5)
        e_z0 = p.z - center_fn(s_now)
        z_dot = 0.0
        if self.z_prev is not None and dt > 0:
            z_dot = (p.z - self.z_prev) / dt
        if self.adaptive_speed:z_dot=-measured_up
        self.z_prev = p.z
        s_pred = profile.horizon_s(s_now + max(2.0, v_s * self.pred_time),
                                   s_now, z_horizon)
        e_pred = (p.z + z_dot * self.pred_time) - center_fn(s_pred)
        z_worsening = (self.z_err_prev is not None
                       and abs(e_z0) > abs(self.z_err_prev) + 1e-4)
        pred_worse = abs(e_pred) > abs(e_z0) + 1e-3
        self.z_err_prev = e_z0
        miss_ratio = self._predict_gate_miss(ng, pn, dt)
        slope_trusted = (ng is None or ng.get("trusted", True))

        # ---- Preview feasibility: climb + curve (方案 4, 8) ----
        # 预览距离按 "恢复到巡航后" 的最坏情况估计, 否则低速时 preview 自身
        # 会被当作上限, 形成越慢越上不去的死锁。
        feedback_speed = self.v_cmd_prev if self.v_cmd_prev > 0.5 else available_cruise
        preview_speed = max(feedback_speed, available_cruise)
        climb = (self.terrain.evaluate(s_now,center_fn,self.vz_cap,available_cruise) if self.adaptive_speed
                 else self.climb_feas.evaluate(s_now,p.z,center_fn,self.vz_cap,preview_speed,velocity_up=-z_dot))
        curve = self.curve_env.evaluate(s_now, self.xy_tracker.curvature, self.a_lat_max,
                                        preview_speed, cruise=available_cruise)

        # ---- Mission mode: TRACK / RECON / HOLD (方案 10, 38.2) ----
        horizon_s = max((g["s"] for g in chain.gates), default=None)
        if self.planner.trend_horizon is not None:
            horizon_s=max(horizon_s or 0.,self.planner.trend_horizon)
        if self.race_goal is not None and self.route.total_s-s_now<30.:
            horizon_s=max(horizon_s or 0.,self.route.total_s+10.)
        soft = self.planner.evidence_horizon
        if soft is not None and self.soft_horizon_stamp is not None \
                and pose_stamp is not None \
                and 0.0 <= pose_stamp - self.soft_horizon_stamp <= 8.0 \
                and soft > s_now + 2.0:
            horizon_s = soft if horizon_s is None else max(horizon_s, soft)
        mode, v_map, transition = self.mission.update(
            self.use_gate_map, horizon_s, s_now, p.z)
        if transition is not None:
            rospy.logwarn("[MODE] %s -> %s at s=%.1f", transition["from"],
                          transition["to"], s_now)
            self.event("MODE", pose_stamp=pose_stamp, **transition)

        cross_track_error=current_lateral-self.clearance_y
        cross_cap=None
        if self.adaptive_speed:
            tracking_error=max(abs(cross_track_error),abs(e_z0))
            cross_cap=self.speed_sched.cruise*max(.35,min(1.,.6/max(.6,tracking_error)))
        # ---- SpeedScheduler: 唯一水平速度 authority (方案 5) ----
        v_target, self.speed_info = self.speed_sched.target(
            self.xy_tracker.curvature(s_now), kz_prev, slope_trusted, miss_ratio, e_z0,
            z_worsening, pred_worse, math.degrees(yaw_err), vxy=v_s,
            v_curve_preview=curve["v_curve_preview"],
            v_climb_preview=climb["v_climb_preview"], v_map=v_map,v_tracking_cap=cross_cap)
        self.speed_info["mode"] = mode
        self.speed_info["v_curve_envelope"] = curve["curve_worst_v"]
        self.speed_info["v_climb_worst_dz"] = climb["climb_worst_dz"]
        if self.adaptive_speed:
            if cross_cap==v_target:
                self.speed_info['reason']='CROSS_TRACK'
            self.speed_info['v_cross_track']=cross_cap
            self.speed_info['tracking_error']=tracking_error
        self.speed_info['cross_track_error']=cross_track_error

        obstacle_cap=self.clearance_info.get('cap')
        if active and not predictive_path:
            obstacle_cap,aligned=self.clearance.progress_cap(
                self.clearance_info,current_lateral,current_vertical,
                self.clearance_info.get('lateral',0.),self.clearance_info.get('vertical',0.))
            self.clearance_info=dict(self.clearance_info,aligned=aligned)
        if obstacle_cap is not None:
            v_target=min(v_target,obstacle_cap)
            self.speed_info['hard_cap']=min(self.speed_info['hard_cap'],obstacle_cap)
            self.speed_info['v_obstacle']=obstacle_cap
            if v_target==obstacle_cap:self.speed_info['reason']=self.clearance_info.get('guard_reason','OBSTACLE')

        # ---- Safety Recovery: 严重 Z 掉队才接管 (原 Z_LAG 降级, 方案 6.1) ----
        v_recovery, recovery_active = self.safety.z_recovery(
            v_target, e_z0, z_worsening, pred_worse)
        if recovery_active:
            v_target = v_recovery
            self.speed_info["reason"] = "SAFETY_Z"
            self.speed_info["v_recovery"] = v_recovery
        v = self.speed_sched.step(self.v_cmd_prev, v_target, dt,
                                  hard_cap=self.speed_info.get("hard_cap"))
        self.v_cmd_prev = v
        self.v_target = v_target

        # ---- STUCK 检测: 真实 pose dt (方案 23) ----
        if self.prev_pose is not None and self.safety.stuck_step(
                dt, v, self.prev_pose, (p.x, p.y, p.z)):
            rospy.logerr("STUCK: cmd=%.2f -> abort", v)
            self.event("TERMINATION", reason="STUCK", s=s_now, v_cmd=v)
            self.mission.abort()
            self.publish(0.0, 0.0, 0.0, 0.0)
            return

        # ---- XY: continuous corridor tangent plus cross-track correction ----
        tx, ty = self.xy_tracker.target(
            (p.x, p.y), s_now, v, ng, d_g, self.task_state.last_resolved,
            self.route.point_at)
        v_route = self.xy_tracker.velocity(
            (p.x, p.y), (tx, ty), v, arbiter=self.arbiter, dt=dt,
            lateral_offset=0. if predictive_path else self.clearance_y,
            terminal_position=self.race_goal[:2] if terminal else None,
            lateral_speed_limit=(1.5 if active and not predictive_path and
                                 self.clearance_info.get('feasible', False) and
                                 not self.clearance_info.get('stale', False) else None))

        # ---- Z: 正常 TRACK 只有 FF+FB; RECON/HOLD 独立 (方案 9, 10) ----
        z_raw = profile.center(s_now)
        a_prev, a_next = profile.anchor_pair(s_now)
        if mode == HOLD:
            zres = self.z_ctrl.hold(dt)
        else:
            # Search changes horizontal speed, not the known spatial climbing trajectory.
            z_ref, z_rate_limited = self.z_ctrl.reference(center_fn, s_ff, dt)
            v_feed_forward=measured_progress if self.adaptive_speed else v
            zres = self.z_ctrl.track(z_ref, p.z, kz_prev, v_feed_forward, dt,
                                     next_gate=ng, d_gate=d_g)
            zres["z_ref"] = z_ref
            if z_rate_limited and zres["z_limit"] == "NONE":
                zres["z_limit"] = "Z_RATE_MAX"
        z_ref = zres["z_ref"]
        z_err = zres["z_err"]
        vz_ff = zres["vz_ff"]
        vz_fb = zres["vz_fb"]
        vz_target = zres["vz_target"]
        vz_clamped = zres["vz_clamped"]
        vz = zres["vz_cmd"]
        z_limit = zres["z_limit"]
        z_actual_dot = z_dot

        if self.adaptive_speed:
            # ZController commands positive UP, while measured motion/clouds
            # use world NED. Certify all three final axes in the same frame.
            desired=np.array([v_route[0],v_route[1],-vz])
            command,checked=self.execution_guard.filter_command(pn,self.motion.velocity,
                desired,lidar_points,lidar_stamp,pose_stamp)
            shifting=self.shift_target is not None
            shift_completed=False
            if (shifting and np.linalg.norm(self.shift_target-pn)<.1 and
                    np.linalg.norm(self.motion.velocity)<.1 and
                    self.execution_guard.index.distance([pn])[0]>=self.execution_guard.margin+.2):
                offset=np.array([current_lateral,current_vertical])
                self.executing_plan=LatticeDetour([s_now,s_now+30.],np.array([offset,offset]))
                self.plan_sequence+=1
                self.shift_target=None
                self.retreat_pending=self.retreat_pending or self.shift_retreat
                self.shift_retreat=False
                self.planner_info={}
                shifting=False
                shift_completed=True
            target=(self.shift_target if shifting
                    else self.planner_info.get('recovery_target'))
            # The official departure marker supplies a floor datum even when
            # the upward-looking lidar cannot see the floor immediately below.
            max_shift_z=(self.departure_hover_z+.25+base_center(s_now)-base_center(0.)
                         if self.departure_hover_z> -900. and s_now<20. else None)
            if self.execution_guard.index is not None:
                measured_floor=self.execution_guard.index.floor_limit([pn],self.execution_guard.margin+.3)[0]
                if np.isfinite(measured_floor):max_shift_z=measured_floor
            if target is not None and max_shift_z is not None and target[2]>max_shift_z:
                target=None
            needs_shift=(not shift_completed and
                         (self.retreat_pending or guard['cap']<.1 or checked['command_scale']==0. or
                          (self.executing_plan is None and guard.get('verified_distance',30.)<8.)) and
                         np.linalg.norm(self.motion.velocity)<=.8)
            if not shifting and not needs_shift:
                target=None
            if shift_completed:
                command=np.zeros(3)
                checked=dict(command_reason='SHIFT_COMPLETE',command_scale=0.)
            if (target is not None or needs_shift) and checked['command_reason']!='COUNTER_BRAKE':
                escape=(self.execution_guard.escape(pn,self.motion.velocity,np.asarray(target),
                    lidar_points,lidar_stamp,pose_stamp) if target is not None else None)
                if (escape is None and needs_shift and
                        (self.shift_checked is None or pose_stamp-self.shift_checked>=.25)):
                    self.shift_checked=pose_stamp
                    shifted=self.execution_guard.find_shift(pn,self.motion.velocity,
                        np.array([rx,ry,base_center(s_now)]),np.array([-fy,fx,0.]),
                        lidar_points,lidar_stamp,pose_stamp,
                        forward_path=PredictiveAvoidance.positions(s_now+np.arange(0.,4.1,.15),
                            self.xy_tracker.base_point_at,base_center,None),max_z=max_shift_z)
                    if shifted is not None:target,escape=shifted
                if escape is not None:
                    if not shifting:self.shift_retreat=float((np.asarray(target)-pn)[:2]@np.array([fx,fy]))<-.25
                    self.shift_target=np.asarray(target)
                    command=escape
                    checked=dict(command_reason='CERTIFIED_SHIFT',command_scale=None,
                                 recovery_target=np.asarray(target).tolist())
                    # Replan from the measured shifted position, rather than
                    # trying to join the obsolete reference through the car.
                    self.executing_plan=None
                    self.xy_tracker.detour=None
                elif target is not None or needs_shift:
                    command=np.zeros(3)
                    checked=dict(command_reason='SHIFT_WAIT',command_scale=0.)
            v_route=command[:2];vz=-float(command[2])
            v=float(np.linalg.norm(v_route))
            self.v_cmd_prev=v
            self.xy_tracker.previous_velocity=tuple(v_route)
            self.z_ctrl.prev_vz=vz
            self.clearance_info.update(checked)
            self.clearance_info['retreat_pending']=self.retreat_pending
            if checked['command_reason'] in ('COMMAND_BLOCKED','LIDAR_STALE','LIDAR_EMPTY'):
                self.z_ctrl.prev_z_ref=p.z
            if checked['command_reason']!='COMMAND_CLEAR':
                self.speed_info['reason']=checked['command_reason']

        # Gate 完整链路 Trace
        if (self.trace_gate >= 0 and ng is not None and ng.get("id") == self.trace_gate
                and (ng["s"] - self.trace_pre) <= s_now <= (ng["s"] + self.trace_post)):
            self._trace_z(s_now, ng, z_raw, z_ref, p.z, z_err, profile.dz_ds(s_now), kz_prev,
                          vz_ff, vz_fb, vz_target, vz_clamped, vz, z_limit,
                          a_prev, a_next, mode, curve["v_curve_preview"],
                          climb["v_climb_preview"], dt)

        self.prev_pose = (p.x, p.y, p.z)
        self.prev_s = s_now

        ledgers = self.task_state.summary()
        self.telemetry_pub.publish(String(data=json.dumps(dict(
            schema_version=1, stamp=rospy.Time.now().to_sec(), pose_stamp=pose_stamp,
            dt=dt, mode=mode, s=s_now, gate_index=self.gate_idx,
            next_gate_id=ng.get("id") if ng else None,
            no_future_gate=self.no_future_gate, z=p.z, z_ref_raw=z_raw, z_ref=z_ref,
            dzds=kz_prev, vz_ff=vz_ff, vz_fb=vz_fb, vz_target=vz_target,
            vz_command=vz, z_limit=z_limit, speed=v, speed_target=v_target,
            velocity_world=[float(v_route[0]),float(v_route[1])],
            measured_velocity_world=self.motion.velocity.tolist(),
            measured_progress_speed=measured_progress,lidar_age=cloud_age,
            pose_callback_age=max(0.,now-self.pose_arrival),
            attitude_rad=[self.roll,self.pitch,self.yaw],
            path_xy=list(self.xy_tracker.point_at(s_now)[:2]),
            speed_limits=self.speed_info, yaw_target=yaw_target,
            yaw_rate_rad_s=yaw_rate, gate_ledger=ledgers,
            climb_preview=climb["v_climb_preview"], curve_preview=curve["v_curve_preview"],
            clearance=self.clearance_info, clearance_z=self.clearance_z,
            clearance_y=self.clearance_y,
            height_prior_valid=self.planner.height_prior.valid,
            height_prior_error=self.planner.height_prior.error,
            planning_horizon=self.planner.trend_horizon,
            v_map=v_map), allow_nan=False)))

        # ---- 到达 ----
        if self.end_gate >= 0 and self.gate_idx > self.end_gate:
            self.publish(0.0, 0.0, 0.0, 0.0)
            if not self.reached:
                self.event("TERMINATION", reason="END_GATE_LIMIT", s=s_now)
            self.reached = True
            return
        if self.gate_idx >= len(chain.gates) and s_now > self.route.total_s - 5.0 \
                and (self.race_goal is None or
                     (np.linalg.norm(pn[:2]-self.race_goal[:2])<1. \
                      and abs(p.z-(self.race_goal[2]-self.terminal_hover_height))<.5)):
            self.publish(0.0, 0.0, 0.0, 0.0)
            if not self.reached:
                rospy.loginfo("GOAL_REACHED")
                self.event("TERMINATION", reason="ROUTE_END", s=s_now,
                           official_result="UNKNOWN")
            self.reached = True
            return

        final_cmd = self.arbiter.finalize(v_route, vz, yaw_rate, self.yaw,
                                          safety=self.safety, mode=mode)
        self.publish(final_cmd.vx, final_cmd.vy, final_cmd.vz, final_cmd.yaw_rate)

        si = self.speed_info
        g_id = ng.get("id") if ng is not None else None
        g_s = ng["s"] if ng is not None else -1.0
        g_src = ng.get("source", "static_yaml") if ng is not None else "-"
        g_hard = ng.get("hard_anchor", True) if ng is not None else "-"
        g_class = ng.get("anchor_class", "-") if ng is not None else "-"
        g_sigz = ng.get("sigma_z", 0.0) if ng is not None else 0.0
        rospy.loginfo_throttle(
            2.0,
            "PATH s=%.0f v=%.2f/%.2f CAP=%s(vC=%.1f vCv=%.1f vS=%.1f vCl=%.1f vT=%.1f "
            "vM=%s vzAv=%.2f) noFG=%s idx=%d next=%s@%.0f src=%s class=%s hard=%s sigz=%.2f "
            "dG=%.1f lat=%.2f vert=%.2f miss=%s mode=%s | "
            "yaw_src=%s yaw=%.1f yawT=%.1f yawErr=%.1f yawSentDeg=%.1f eImg=%.3f | "
            "Z s=%.0f zRef=%.2f z=%.2f err=%.2f dzds=%.3f ff=%.2f fb=%.2f tgt=%.2f "
            "clamp=%.2f cmd=%.2f act=%.2f LIMIT=%s",
            s_now, v, v_target, si.get("reason", ""),
            si.get("v_curve", 0.0), si.get("v_curve_preview") or si.get("v_curve", 0.0),
            si.get("v_slope", 0.0), si.get("v_climb", 0.0),
            si.get("v_tracking", 0.0),
            ("%.1f" % si["v_map"]) if si.get("v_map") is not None else "-",
            si.get("vz_available", 0.0), self.no_future_gate, self.gate_idx,
            str(g_id), g_s, g_src, g_class, str(g_hard), g_sigz, d_g, lat, vert,
            ("%.2f" % miss_ratio) if miss_ratio is not None else "-", mode,
            yaw_source, math.degrees(self.yaw), math.degrees(yaw_target),
            math.degrees(yaw_err), math.degrees(yaw_rate), e_img,
            s_now, z_ref, p.z, z_err, kz_prev, vz_ff, vz_fb, vz_target,
            vz_clamped, vz, -z_actual_dot, z_limit)


if __name__ == "__main__":
    rospy.init_node("route_follower")
    RouteFollower()
    rospy.spin()
