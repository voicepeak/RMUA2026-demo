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
```

> 运行前需先启动官方模拟器（见其 README）。
