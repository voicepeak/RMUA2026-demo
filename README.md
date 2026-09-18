# RMUA 2026 自主无人机竞速 —— 开发记录

本仓库按迭代顺序记录 RMUA 2026 模拟器（RoboMaster IntelligentUAVChampionshipSimulator, 分支 RMUA2026-01）
上从 0 到 1 的控制程序开发过程。每个阶段包含：**设计方案（docs/）** + **对应代码（ros_ws/）**。

## 迭代顺序

| 阶段 | 方案文档 | 代码 | 目标 |
| --- | --- | --- | --- |
| Stage 1 | `docs/01_stage1_start_to_goal.md` | `ros_ws/src/airsim_ros`, `ros_ws/src/start_to_goal` | 最小 Start→Goal 闭环 |
| Stage 2 | `docs/02_stage2_track_path_planning.md` | `ros_ws/src/route_follower` | 赛道约束下的参考路径跟踪 |
| Stage 3 | `docs/03_stage3_z_axis_and_gates.md` | `route_follower`（Gate Manager） | 3D 路径 + 检测门穿越 |
| Stage 4 | `docs/04_stage4_z_safety_and_gate_collection.md` | `route_follower`（Z Safety）+ `z_probe.py` | Z 合法高度包络 + 真实 Gate 采集 |
| Stage 5 | `docs/05_stage5_xy_gate_anchors_z_planner.md` | `altitude_planner.py` + `route_follower` | Gate 锚点生成 z_ref(s)（Smoothstep/混合/限速/异常保护） |
| Stage 6 | `docs/06_stage6_stereo_opencv_gate_vision.md` | `ros_ws/src/rmua_gate_vision` | 双目 OpenCV Gate 识别 → 三角化 → 真实 Gate XYZ |
| Stage 7 | `docs/07_stage7_full_gate_crossing_v2.md` | `route_follower`（模块化重构） | Command Arbiter + Gate 状态机 + 动态 Z 走廊 + 避障预留 |
| Stage 7b | `docs/08_stage7_v3_multi_gate_chain.md` | `gate_chain.py` + `route_follower` | 多 Gate 前视/顺序链 + Look-ahead 参考 + 动态 Z + 提速 |
| Stage 8 | `docs/09_stage8_z_preview_feedforward_highspeed.md` | `altitude_profile` + `route_follower` | Z 前视+前馈、坡度限速、掉高保护、速度斜坡，支持 10 m/s |

## 系统与接口
- 平台：Ubuntu 20.04 + ROS Noetic + 官方模拟器（NED 世界系）。
- 关键话题：
  - 位姿：`/airsim_node/drone_1/debug/pose_gt` (`geometry_msgs/PoseStamped`)
  - 终点：`/airsim_node/end_goal` (`geometry_msgs/PoseStamped`)
  - 速度控制：`/airsim_node/drone_1/vel_body_cmd` (`airsim_ros/VelCmd`)
- `airsim_ros/VelCmd` 定义与模拟器一致（否则 md5 不匹配）：
  `std_msgs/Header header; float64 vx, vy, vz, yawRate; uint8 va, stop`
- 实测约定：世界系 NED（z 向下为正），而 `vel_body_cmd` 的 `vz` **向上为正**。

## 编译与运行
```bash
# 依赖 ROS Noetic
cp -r ros_ws ~/rmua_ws          # 或直接把 ros_ws/src 放进你的 catkin 工作空间
cd ~/rmua_ws && catkin_make
source /opt/ros/noetic/setup.bash && source devel/setup.bash

# Stage 1
roslaunch start_to_goal start_to_goal.launch

# Stage 2/3/4
roslaunch route_follower route_follower.launch

# Stage 6: 双目 OpenCV Gate 识别/采集 (边飞边采, 最后 save)
roslaunch rmua_gate_vision gate_vision.launch
rosservice call /gate_vision/save
```

> 运行前需先启动官方模拟器（见其 README）。
