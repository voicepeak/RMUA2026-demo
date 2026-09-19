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
│   ├── 10_stage9_yolo_gate_detection.md
│   ├── 11_stage10_yaw_persistent_map_continuous3d.md
│   ├── 12_stage11_gate_keypoints_timesync_reprojection.md
│   └── 13_stage12_speed_yaw_fix.md
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
| Stage 5 | `docs/05` | `altitude_profile.py` | Gate 锚点生成 z_ref(s)（原 altitude_planner 已并入） |
| Stage 6 | `docs/06` | `rmua_gate_vision` | 双目 OpenCV 识别 → 三角化 → 真实 Gate XYZ |
| Stage 7 | `docs/07` | `route_follower` 模块化 | Command Arbiter + Gate 状态机 + 动态 Z 走廊 + 避障预留 |
| Stage 7b | `docs/08` | `gate_chain.py` | 多 Gate 前视/顺序链 + 动态 Look-ahead + 提速 |
| Stage 8 | `docs/09` | `altitude_profile.py` | Z 前视+前馈、坡度限速、掉高保护、速度斜坡（10 m/s） |
| Stage 9 | `docs/10` | `yolo/`, `gate_yolo_node.py` | YOLO Gate 识别（采集/标注/训练/部署） |
| Stage 10 | `docs/11` | `yaw_controller.py`, `speed_scheduler.py`, `gate_tracker.py`, `gate_map.py` | 转弯丢 Gate / Z 下坠 / 连续三维跟踪修复：Yaw 前视、持久 Gate Map、连续 Z 与 Horizon |
| Stage 11 | `docs/12` | `stereo_keypoint_matcher.py`, `gate_geometry.py`, `timestamp_sync.py`, `gate_reprojection.py` | 四角关键点 + 逐角三角化 + 时间同步 + 反投影关联 + Yaw FOV correction |
| Stage 12 | `docs/13` | `speed_scheduler.py`, `route_follower.py` | 转弯/爬升骤降速 & Yaw 不转向修复：物理限速 + 预测穿门 + 软限速 floor + Yaw 斜坡 |

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

# 3) Stage2-12 主控制器（v6: 物理限速 + 预测穿门 + Yaw 斜坡 + 连续三维）
roslaunch route_follower route_follower.launch \
  gates_file:=$(rospack find route_follower)/config/gates_vision_1_3.yaml \
  gate_z_uses_offset:=false start_anchor_z:=-4.0 \
  cruise_speed:=10.0 normal_speed_floor:=4.0 \
  yaw_control:=true yaw_lookahead:=25.0 k_yaw:=1.2 yaw_accel_limit:=2.0

# 3b) 可选：订阅视觉端运行时持久 Gate Map（边飞边更新 3D 锚点）
#     先启动视觉节点（见 4/5），再加 use_gate_map:=true

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
- **持久 Gate Map（Stage10）**：`gate_tracker.py`/`gate_map.py` 把逐帧观测融合为稳定 track
  （EMA + sigma，远小近大），发布 `/rmua/gate_map`；`route_follower` 可用
  `use_gate_map:=true` 运行时更新 3D 锚点，YOLO 短时丢失不再影响规划。
- **Yaw 前视（Stage10）**：`yaw_controller.py` 用未来 Gate Chain 折线 / Route 切线取
  `yaw_lookahead≈22 m` 处作为 yaw_target，`yawRate = K·wrap(target−yaw)`（限幅 0.8 rad/s），
  转弯前提前转头，前视相机更久锁定未来 Gate。
- **连续 Z（Stage10）**：`z_horizon≈10~20 m` 外 `dz/ds→0`，避免无未来门时无限下探。
- **速度调度 v6（Stage12, docs/13）**：只保留 `v_curve=sqrt(a_lat_max/κ)`（横向加速度物理限）、
  `v_slope`（垂速能力，且 Soft 门坡度不得压低到 floor 以下）、`v_tracking`（**预测**穿门误差 /
  Z 掉队恶化 / Yaw 误差）；`hard=min(cruise,curve,slope)`，`soft=max(normal_speed_floor,tracking)`，
  `v=min(hard,soft)`。删除 `gate_bad_near` 瞬时误差 →1.5 m/s 与 extra_cap，软限速下限
  `normal_speed_floor=4`。日志含 `CAP=REASON` 与各分量 `vC/vS/vT`。
- **Yaw（Stage12）**：`yaw_lookahead=25 m`，`K_yaw=1.2`，`yaw_rate_max=1.0`，
  新增 `yaw_accel_limit=2.0` 斜坡；日志含 `yaw_src=GATE_CHAIN/ROUTE`、`yawCalc`、`yawSentDeg`。
- **【关键】`VelCmd.yawRate` 单位是 度/秒, 不是 rad/s**（官方 `basic_dev` 注释 `yaw, deg`，
  本机实测：`yawRate=30` 持续 2 s → 约 49°，`yawRate=1` 且 vx=0 → 0°，且**必须有前向速度才生效**）。
  控制器内部用 rad/s，`publish()` 处统一 `math.degrees()` 转换后下发；此前直接发 rad/s 导致机头
  转速慢了约 57 倍、看似"不转向"、侧向 Gate 进不了 FOV。
- **YOLO 目前只训了 ~40 张**（mAP50≈0.7，Recall≈0.5）；要上比赛需按 `yolo/README.md`
  标注 **500+ 图** 重训。
- **坐标变换已用完整 quaternion**（`gate_stereo.body_to_world`）；视觉节点可 `_use_imu:=true`
  改用 `/airsim_node/drone_1/imu/imu` 的姿态（正式应做 pitch/roll 下 Gate World Z 稳定性验证，见 docs/11 Test 2）。
- **Stage11 四角几何链**：`stereo_keypoint_matcher.py` 对左右门按 TL/TR/BR/BL 逐角三角化，
  `gate_geometry.py` 做平面 RMSE / 尺寸 / 长宽比检查，`timestamp_sync.py` 把位姿插值到图像时刻，
  `gate_reprojection.py` 反投影用于 track 关联与 `k_vision` FOV correction。
- **模型**：当前 `weights/best.pt` 是 **检测(bbox)** 模型，四角会退化为 bbox 四角（仍避免门洞背景深度）；
  若训练成 YOLOv8-pose（4 关键点），设置 `_use_keypoints:=true` 即自动使用真实关键点。训练仍需按
  `yolo/README.md` 标注 500+ 图；`_use_keypoints` 会自动检测模型是否带 keypoints。
- **模拟器渲染在这台机器上会周期性 Xid 崩**（老 UE4 Vulkan + 新驱动）；用 offscreen 更稳，
  渲染用于观看时可能几分钟崩一次。
- **避障仅预留接口**（`avoidance_interface.py`，默认 `active=False`），尚未实现。
- **Z 能力标定**：`vz_up_safe/vz_up_limit` 目前为估值 3.0，建议用 `z_probe.py`/垂速测试实测后写回。
