# RMUA 2026 自主无人机竞速 —— 开发与交接

本仓库记录在 RMUA 2026 官方模拟器（RoboMaster IntelligentUAVChampionshipSimulator,
分支 `RMUA2026-01`）上，从 0 到 1 开发无人机自主飞行的完整过程：
**每个阶段 = 一份设计方案（docs/）+ 对应代码（ros_ws/、yolo/）**。

---

## 目录结构（分三级）

```text
RMUA2026-demo/
├── README.md                # 本文件：总览 / 环境 / 运行 / 交接说明
├── docs/                    # 一级：设计方案（按迭代顺序 01..10）
│   ├── 01_stage1_start_to_goal.md
│   ├── 02_stage2_track_path_planning.md
│   ├── 03_stage3_z_axis_and_gates.md
│   ├── 04_stage4_z_safety_and_gate_collection.md
│   ├── 05_stage5_xy_gate_anchors_z_planner.md
│   ├── 06_stage6_stereo_opencv_gate_vision.md
│   ├── 07_stage7_full_gate_crossing_v2.md
│   ├── 08_stage7_v3_multi_gate_chain.md
│   ├── 09_stage8_z_preview_feedforward_highspeed.md
│   └── 10_stage9_yolo_gate_detection.md
├── ros_ws/                  # 一级：ROS 工作空间源码
│   └── src/
│       ├── airsim_ros/          # 与模拟器匹配的 VelCmd/Takeoff/Land/Reset 消息服务
│       ├── start_to_goal/       # Stage1: 最小 Start→Goal
│       ├── route_follower/      # Stage2-8: 路径跟踪/动态Z/Gate/仲裁/避障预留
│       └── rmua_gate_vision/    # Stage6/9: OpenCV 与 YOLO 双目 Gate 识别
└── yolo/                    # 一级：YOLO 视觉（采集/标注/训练/部署）
    ├── README.md
    ├── weights/best.pt
    ├── tools/xany_to_yolo.py
    ├── samples/                 # 标注格式样例
    ├── runs/                    # 训练曲线
    └── data.yaml.example
```

---

## 迭代与方案索引

| 阶段 | 方案 | 代码 | 目标 |
| --- | --- | --- | --- |
| Stage 1 | `docs/01` | `airsim_ros`, `start_to_goal` | 最小 Start→Goal 闭环 |
| Stage 2 | `docs/02` | `route_follower` | 赛道约束参考路径跟踪（Look-ahead/横向纠偏/Boundary Guard） |
| Stage 3 | `docs/03` | `route_follower` | 3D 路径 + 检测门穿越（Gate Manager/三点模型） |
| Stage 4 | `docs/04` | `z_probe.py`, `route_follower` | Z 合法高度包络 + Z Safety Clamp |
| Stage 5 | `docs/05` | `altitude_planner.py` | Gate 锚点生成 z_ref(s) |
| Stage 6 | `docs/06` | `rmua_gate_vision` | 双目 OpenCV 识别 → 三角化 → 真实 Gate XYZ |
| Stage 7 | `docs/07` | `route_follower` 模块化 | Command Arbiter + Gate 状态机 + 动态 Z 走廊 + 避障预留 |
| Stage 7b | `docs/08` | `gate_chain.py` | 多 Gate 前视/顺序链 + 动态 Look-ahead + 提速 |
| Stage 8 | `docs/09` | `altitude_profile.py` | Z 前视+前馈、坡度限速、掉高保护、速度斜坡（10 m/s） |
| Stage 9 | `docs/10` | `yolo/`, `gate_yolo_node.py` | YOLO Gate 识别（采集/标注/训练/部署） |

---

## 环境要求

```text
Ubuntu 20.04 + ROS Noetic + 官方模拟器 (NED 世界系)
仿真控制: 系统 python3 (rospy)
YOLO:     conda 环境 xal (torch cu128 + ultralytics); 无 GPU 时 CPU 亦可
```

关键接口 / 约定：
- 位姿 `/airsim_node/drone_1/debug/pose_gt` (`geometry_msgs/PoseStamped`)
- 终点 `/airsim_node/end_goal`、速度 `/airsim_node/drone_1/vel_body_cmd` (`airsim_ros/VelCmd`)
- `VelCmd` 定义必须与模拟器一致（否则 md5 不匹配）：
  `std_msgs/Header header; float64 vx, vy, vz, yawRate; uint8 va, stop`
- **世界系 NED（z 向下为正），而 `vel_body_cmd` 的 `vz` 向上为正**。

---

## 运行

```bash
# 0) 启动官方模拟器
cd /path/to/simulator_12.0.0.5 && ./run_simulator.sh 123      # 渲染窗口
# 或 ./run_simulator_offscreen.sh 123                          # 后台(更稳)

# 1) 编译本仓库源码
cp -r ros_ws ~/rmua_ws && cd ~/rmua_ws && catkin_make
source /opt/ros/noetic/setup.bash && source devel/setup.bash

# 2) Stage1 最小闭环
roslaunch start_to_goal start_to_goal.launch

# 3) Stage2-8 主控制器（真实 Gate + 动态Z + v4 前馈/掉高保护）
roslaunch route_follower route_follower.launch \
  gates_file:=$(rospack find route_follower)/config/gates_vision_1_3.yaml \
  gate_z_uses_offset:=false start_anchor_z:=-4.0

# 4) 视觉: OpenCV 检测/可视化
rosrun rmua_gate_vision gate_detect_viz.py     # 发布 /rmua/gate_detection/stereo
rosrun image_view image_view image:=/rmua/gate_detection/stereo

# 5) 视觉: YOLO 检测 (xal 环境)
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
/home/huang/miniconda3/envs/xal/bin/python \
  ~/rmua_ws/src/rmua_gate_vision/scripts/gate_yolo_node.py \
  _model:=<repo>/yolo/weights/best.pt \
  _route_file:=<ws>/src/route_follower/config/route_1_3.yaml
```

---

## 交接要点 / 已知问题

- **控制链路已跑通**：模拟器实测 **10/10 真实 Gate 顺序通过**（Stage7/8，横向误差 ≤0.2 m，巡航 4–6 m/s）。
- **视觉两种 Detector 可互换**：OpenCV（`gate_detector_opencv.py`）与 YOLO（`gate_yolo_node.py`），
  输出统一 `GateObservation`，下游不变。
- **YOLO 目前只训了 ~40 张**（mAP50≈0.7，Recall≈0.5）；要上比赛需按 `yolo/README.md`
  标注 **500+ 图** 重训。
- **模拟器渲染在这台机器上会周期性 Xid 崩**（老 UE4 Vulkan + 新驱动）；用 offscreen 更稳，
  渲染用于观看时可能几分钟崩一次。
- **避障仅预留接口**（`avoidance_interface.py`，默认 `active=False`），尚未实现。
- **Z 能力标定**：`vz_up_safe/vz_up_limit` 目前为估值 3.0，建议用 `z_probe.py`/垂速测试实测后写回。
