# RMUA 2026 自主无人机竞速 —— 开发与交接

本仓库记录在 RMUA 2026 官方模拟器（RoboMaster IntelligentUAVChampionshipSimulator,
分支 `RMUA2026-01`）上，从 0 到 1 开发无人机自主飞行的完整过程：
**每次迭代 = 设计方案（docs/）+ 代码（ros_ws/、yolo/）+ 回归测试 + README 更新**。

当前状态（2026-09-29）：控制链路可稳定飞到 s≈500–800 m（8 m/s 档最好 s≈797 m），
测试 120 项全过；**尚未完成全程验收**，主要缺口见 [§5 已知限制](#5-已知限制未解决)。

---

## 1. 目录结构

```text
RMUA2026-demo/
├── README.md                # 本文件：迭代路线 / 环境 / 运行 / 交接
├── docs/                    # 设计方案（按迭代顺序 01..17）
├── ros_ws/src/
│   ├── airsim_ros/          # 与模拟器匹配的 VelCmd/Takeoff/Land/Reset 消息服务
│   ├── start_to_goal/       # Stage1: 最小 Start→Goal
│   ├── route_follower/      # Stage2-14 + 模块化控制（详见 §2）
│   └── rmua_gate_vision/    # 双目视觉：OpenCV / YOLO / 四角几何 / 持久门图
├── yolo/                    # 采集 / 标注 / 训练 / 部署（weights/best.pt）
└── tools/                   # 运行归档、独立评估、离线审计、回归测试
```

---

## 2. 迭代路线（分层）

> 每一层都建立在上一层之上；层内按提交顺序列出。完整提交序列见 [§3](#3-提交索引)。

### L0 基础闭环（Stage 1）
| 提交 | 内容 |
| --- | --- |
| `8c2cd6a` | 最小 Start→Goal：`airsim_ros`（VelCmd/Takeoff/Land/Reset）+ `start_to_goal` 节点 |

- 方案：`docs/01`；目标：打通位姿订阅 → 速度指令 → 到达终点。

### L1 路线跟踪与高度（Stage 2–5）
| 提交 | 内容 |
| --- | --- |
| `7d7d93a` | Stage2 赛道约束路径跟踪（Look-ahead / 横向纠偏 / Boundary Guard） |
| `5619df6` | Stage3 3D 路径 + 检测门穿越（Gate Manager / 三点模型） |
| `ec1cf8b` | Stage4 Z 合法高度包络 + Z Safety Clamp + `z_probe.py` |
| `90833f9` | Stage5 Gate 锚点生成 `z_ref(s)`（AltitudeProfile） |
| `43759b7` `fb1bb56` | Stage5b Altitude Guides、闭环多门验证、`start_anchor_z` 覆盖 |

- 方案：`docs/02`–`05`；目标：让无人机沿赛道飞、高度有依据、不会随便掉高。

### L2 视觉与真实穿门（Stage 6–9）
| 提交 | 内容 |
| --- | --- |
| `d9810cf` `3983c60` | Stage6 双目 OpenCV 检测 → 三角化 → 真实 Gate XYZ |
| `a6a55b1` | Stage7 全穿门 + 强对齐 + Command Arbiter + 避障接口预留 |
| `47eeab0` | Stage7-v3 多门链 + 动态 Z + 速度调度 |
| `b04152b` | Stage7-v3b **门 XY 吸附到路线中心 + 温和收敛 + skip-behind** |
| `7aefb22` | Stage8 Z 前视 + 前馈 + 坡度限速 + 掉高保护（10 m/s 档） |
| `a769f22` `be55ea3` `9b964de` | 检测可视化、YOLO 检测器、仓库分层整理 |

- 方案：`docs/06`–`10`；目标：用视觉拿到门的世界坐标并完成穿越。
- ⚠️ `b04152b` 引入的"门 XY 吸附到路线"是后来横向偏差问题的根源（见 §4 关键结论）。

### L3 高速稳定性（Stage 10–14）
| 提交 | 内容 |
| --- | --- |
| `a779cec` | Stage10/11 持久 Gate Map + 四角关键点 + 时间同步 + 反投影关联 |
| `ab6e8f3` | Stage10/12 Yaw 前视、连续三维、物理限速调度、**deg/s yawRate 修正** |
| `20de2e0` | Stage13/14 Z 通道解限 + 垂直能力标定 + 启动高度保护 + Gate6 前馈修复 + hard/soft 优先级 |
| `716ec2f` | YOLO prune 时间戳类型修复 |

- 方案：`docs/11`–`15`；目标：10 m/s、转弯丢门、Z 下坠、Gate6 偏高这些具体问题逐项修复。

### L4 全程审查与独立评估（工作包 1）
| 提交 | 内容 |
| --- | --- |
| `063ed1c` | 全程算法审查（`docs/16`）+ 运行归档/独立计分方案（`docs/17`） |
| `654da48` | `tools/`：`run_experiment`（快照+录制）、`evaluate_run`（独立门洞几何评估）、`export_run`、`audit_course` + 回归测试 |

- 目标：固定场景/源码/配置；PASS/MISS/SKIP/UNKNOWN 与终止原因分开记录；官方成绩默认为 `UNKNOWN`。
- 关键工具链：
  - `python3 tools/run_experiment.py --execute`：录全话题 + 启控制
  - `python3 tools/evaluate_run.py --gates <核验门.json> --poses poses.csv`：按门平面几何判穿越
  - `python3 tools/audit_course.py --log <controller.log>`：路线/门覆盖离线审计

### L5 在线地图与安全保护（工作包 3）
| 提交 | 内容 |
| --- | --- |
| `d6b1149` | `online_gate_cache`（track 身份匹配 + 静态门去重 + completed 账本）、视觉深度修复、控制器 `events/telemetry`、软视野 + RECON 搜扫、Z 掉队限速、卡死检测、reset 账本清理、无门区坡度外推 |
| `fc09a34` | 预测爬升前馈（修正符号）、路线/门心混合引导（`gate_center_pull_max` 限幅）、**门平面/门心过门判定** |

- 方案：`docs/17`（WP3 章节）；目标：门锚点跨 ID/丢检保持、地图断档不悬停、过门判定不再自欺。

### L6 模块化与高度先验（当前最新）
| 提交 | 内容 |
| --- | --- |
| `ba20d1d` | 模块化控制管线：`reference_planner` / `xy_tracker` / `z_controller` / `safety_supervisor` / `mission_state` / `gate_task_state` / `curve_preview` / `climb_feasibility` |
| `b5c7b8e` | `route_height_prior`（赛道 spline Z 先验，用门测量拟合 datum/tilt、残差大则拒绝）、`persistent_guides`（与 hard/可见性解耦的高度证据）、`spatial_curve`（C1 Hermite 保形插值）、`lidar_clearance`（激光净空，待实飞验证） |
| `c57c2ec` | 视觉异步 latest-frame 推理循环 + gate_map 更新（防 GPU 慢导致位姿过期丢帧） |

- 新默认（`route_follower.launch`）：`cruise_speed=8.0`、`use_gate_map=true`、
  `snap_gate_to_route=false`、`gate_center_pull_max=2.0`、`k_ff_z=1.0`、
  `z_soft_guide_max=120`、`z_extrap_m=50`、`z_extrap_slope_max=0.5`。
- 实测：`c6` s≈505 m / 15 门；`c7` s≈690 m（探针 15 门通过、1 MISS；
  ROS 日志跨会话可见 23 个门 id）后 STUCK；
  `d1` s≈797 m / 15 门（7 MISS，横向 1.56–4.64 m）后异常复位。

---

## 3. 提交索引

| # | 提交 | 层级 | 一句话 |
| --- | --- | --- | --- |
| 1 | `8c2cd6a` | L0 | Stage1 最小闭环 |
| 2 | `7d7d93a` | L1 | Stage2 赛道约束跟踪 |
| 3 | `5619df6` | L1 | Stage3 3D 路径 + Gate Manager |
| 4 | `ec1cf8b` | L1 | Stage4 Z 包络 + Safety Clamp |
| 5 | `f2f7a7a` | L1 | 添加 .gitignore |
| 6 | `90833f9` | L1 | Stage5 Gate 锚点高度规划 |
| 7 | `43759b7` | L1 | Stage5b Altitude Guides + 多门验证 |
| 8 | `fb1bb56` | L1 | Stage5b `start_anchor_z` 覆盖 |
| 9 | `d9810cf` | L2 | Stage6 双目 OpenCV → Gate XYZ |
| 10 | `3983c60` | L2 | Stage6b 真门集成修复 |
| 11 | `a6a55b1` | L2 | Stage7 全穿门 + Arbiter |
| 12 | `47eeab0` | L2 | Stage7-v3 多门链 + 动态 Z + 限速 |
| 13 | `b04152b` | L2 | Stage7-v3b 门 XY 吸附路线（问题根源） |
| 14 | `7aefb22` | L2 | Stage8 Z 前视/前馈/坡度限速 |
| 15 | `a769f22` | L2 | 检测可视化工具 |
| 16 | `be55ea3` | L2 | Stage9 YOLO 检测节点 |
| 17 | `9b964de` | L2 | 仓库分层整理交接 |
| 18 | `a0ec34b` | L3 | docs stage10-12 |
| 19 | `a779cec` | L3 | Stage10/11 持久门图 + 关键点/时间同步 |
| 20 | `ab6e8f3` | L3 | Stage10/12 Yaw/连续三维/限速/deg-yawRate |
| 21 | `b795731` | L3 | 清理死代码 |
| 22 | `d406913` | L3 | docs stage13-14 |
| 23 | `716ec2f` | L3 | yolo prune 类型修复 |
| 24 | `20de2e0` | L3 | Stage13/14 Z 解限 + 启动保护 + Gate6 FF |
| 25 | `063ed1c` | L4 | 全程审查 + 运行归档方案 |
| 26 | `654da48` | L4 | tools 运行归档/独立评估/审计 + 测试 |
| 27 | `d6b1149` | L5 | 在线门缓存 + 视觉修复 + 安全保护 |
| 28 | `fe47024` | L5 | README（WP1/WP3） |
| 29 | `fc09a34` | L5 | 预测爬升 + 混合引导 + 门心判定 |
| 30 | `ba20d1d` | L6 | 模块化控制管线 |
| 31 | `b5c7b8e` | L6 | 高度先验 + 持久引导 + C1 曲线 + lidar |
| 32 | `c57c2ec` | L6 | 视觉异步 latest-frame |
| 33 | `<本轮>` | L6 | README 分层迭代路线（本文件） |

---

## 4. 关键结论（用数据换来的）

1. **"过门 PASS" 曾经是假的**：`b04152b` 起门 XY 被吸附到路线中心，过门判定也按路线中心算横向误差，
   所以飞机从门旁边过也记 PASS。用仓库自带静态门平面做几何核验：
   **9/10 个静态门实际从门洞外侧 1.9–5.2 m 穿过**（`docs/16` 表格）。
   `fc09a34` 改为门平面/门心判定后，MISS 才真实暴露。
2. **路线与门心不一致**：静态门相对路线中心偏移 0.1–5.2 m；YOLO 在线测量也测到 2–7 m 偏移，两套独立
   数据互相印证。当前 `gate_center_pull_max=2.0` 只能拉 ±2 m，偏移更大的门必然 MISS（`d1`：7 MISS）。
3. **无门区不能平飞**：早期剖面在最后一个门之后被压平，坡顶直接撞地；现在用
   "最后坡度限距外推 + soft 门限幅高度引导 + route height prior" 兜底。
4. **"上升预测"符号错过两次**：预测项必须用"当前位置到门的高度差"且只叠加爬升、绝不覆盖下降；
   否则要么在下降门前反向上飘，要么所有下降被清零越飞越高（h1–h3 的教训，`fc09a34` 修正）。
5. **地图断档会永久悬停**：`MAP_HORIZON` 视界只看 hard 锚点时，最后锚点用尽就停死；
   现在软视野 + RECON 搜扫（2 m/s、限 30 m）+ 卡死检测兜底。
6. **GPU 会被打挂**：UE4 Vulkan 反复崩后驱动报
   `uvm global fatal error 0x60 / Node Reboot Required`，必须重启主机；优先用 offscreen 跑。

---

## 5. 已知限制（未解决）

| # | 问题 | 证据 | 方向 |
| --- | --- | --- | --- |
| A | 路线 ≠ 门心，偏移门必然 MISS | d1 7 MISS，横向 1.56–4.64 m | planner 做路线→门心的局部修正（门间直线/偏移样条），或明确信任门心 |
| B | 末端 STUCK（撞结构/地形） | s≈396–433、c7≈690、d1≈797 后卡停 `cmd≈8 act=0` | `lidar_clearance` 实飞验证 + 窄段减速/偏置/绕行 |
| C | 感知连续性 | 坡顶 L=0/R=1、右目 ~2 Hz、hard 锚点断续、后段无门事件 | 单目/单帧兜底、相机帧率、锚点延迟压缩 |
| D | 门账本/ID 一致性 | c7 出现 `100005` 在 s=630 才通过；探针与 ROS 日志统计不一致 | 复核 GateTaskState 与锚点 s 更新/ID 复用 |
| E | `reset` 语义未知 | 飞行中 `/airsim_node/reset` 返回 `success:false` | 确认官方 reset/成绩接口；`race_success` 保持 UNKNOWN |
| F | 高度先验边界 | 已飞到 z≈-98 m（约 98 m 高）、s≈797 m | datum/tilt 只在有门段标定，外推区间需验证 |
| G | 速度-感知矛盾 | >6 m/s 门锚点易滞后；8 m/s 能飞远但 MISS/卡滞 | 感知提速后再做全程速度规划 |

---

## 6. 环境

```text
Ubuntu 20.04 容器 + ROS Noetic（宿主 Ubuntu 22.04 + Docker，镜像 rmua/noetic-xal:latest）
仿真控制: 系统 python3 (rospy)
YOLO:     conda 环境 xal（torch cu128 + ultralytics）; 无 GPU 时 CPU 亦可（很慢，不推荐）
GPU:      NVIDIA RTX 3060 Laptop（UE4 Vulkan 易 Xid；off 屏更稳，崩后需重启主机）
```

关键接口 / 约定：
- 位姿 `/airsim_node/drone_1/debug/pose_gt` (`geometry_msgs/PoseStamped`)
- 终点 `/airsim_node/end_goal`、速度 `/airsim_node/drone_1/vel_body_cmd` (`airsim_ros/VelCmd`)
- `VelCmd` 定义必须与模拟器一致（否则 md5 不匹配）：
  `std_msgs/Header header; float64 vx, vy, vz, yawRate; uint8 va, stop`
- **世界系 NED（z 向下为正），而 `vel_body_cmd` 的 `vz` 向上为正**。
- **`VelCmd.yawRate` 单位是 度/秒**（控制器内部 rad/s，`publish()` 处统一换算）。

---

## 7. 运行

```bash
# 0) 启动官方模拟器（容器内运行；会自动清理旧实例）
cd /home/tianbot/RUMA-by-helinjun
./run_sim.sh 123 render      # 渲染窗口（易崩）
./run_sim.sh 123 offscreen   # 后台模式（稳，推荐长跑）

# 1) 视觉（另开终端；等 ~20s 模型加载）
./run_yolo.sh                # 发布 /rmua/gate_map 等

# 2a) 最新默认（8 m/s，codex L6 参数；直接 roslaunch 即用 launch 默认值）
./enter_rmua.sh
roslaunch route_follower route_follower.launch

# 2b) 保守 5 m/s 档（含混合引导参数）
./run_route_vision.sh        # = cruise 5 / floor 3 + 混合参数

# 2c) 纯静态模式（只认 gates_vision_1_3.yaml 的 10 个门）
./run_route.sh
```

- 观测：`rostopic echo /rmua/controller/events`（latch，门事件/终止原因）、
  `/rmua/controller/telemetry`（20Hz JSON：s/z_ref/vz/限速/yaw）。
- 数据采集：`docker exec rmua_noetic python3 /workspace/frames/flight_probe.py
  --out /workspace/frames/<名字> --seconds 300`。
- 回归：`python3 -m unittest discover -s tools/tests`（当前 **120 项全过**）。
- 停止：`docker exec rmua_noetic pkill -f route_follower.py`；
  关模拟器：`docker exec rmua_noetic bash -lc 'pkill -f RMUA; pkill -f rosmaster'`。

---

## 8. 迭代工作流（约定）

1. 改动源码在 `repo/ros_ws/src`（唯一源头），同步到运行空间：
   `rsync -a --exclude __pycache__ repo/ros_ws/src/ rmua_ws/src/ &&
    docker exec rmua_noetic bash -lc 'source /opt/ros/noetic/setup.bash && cd /workspace/rmua_ws && catkin_make -j4'`
2. 新逻辑必须带 `tools/tests/` 回归测试；提交前全绿。
3. 实飞验证：模拟器 + YOLO + 控制器 + `flight_probe`，数据存 `frames/<run>/`。
4. **每次迭代更新本 README**（§2 路线、§3 提交索引、§4 结论、§5 限制）。
5. 按迭代粒度 commit + push（当前身份：`voicepeak <voicepeak@users.noreply.github.com>`）。

---

## 9. 交接要点（保留）

- **控制链路已跑通**：早期 Stage7/8 实测 10/10 静态门（当时按路线中心判定，见 §4 结论 1）；
  L6 架构最好 s≈797 m / 8 m/s。
- **视觉两种 Detector 可互换**：OpenCV（`gate_detector_opencv.py`）与 YOLO（`gate_yolo_node.py`），
  输出统一 `GateObservation`；`_use_keypoints` 自动检测 bbox/pose 模型。
- **持久 Gate Map**：`gate_tracker.py`/`gate_map.py` + `online_gate_cache.py`（route_follower），
  发布 `/rmua/gate_map`；`use_gate_map:=true` 时控制器可用。
- **Yaw 前视（Stage10）**：`yaw_controller.py`，`yaw_lookahead≈25 m`，`K_yaw=1.2`，限幅 1.0 rad/s。
- **速度调度（Stage12+）**：`v_curve=sqrt(a_lat_max/κ)`、`v_slope`（实测 `vz_available(vxy)`）、
  `v_tracking`（预测穿门误差/Z 掉队/Yaw 误差）；`hard=min(cruise,curve,slope)`，
  `soft=max(floor,tracking)`，`v=min(hard,soft)`；日志含 `CAP=REASON` 与各分量。
- **启动高度保护（Stage14）**：`start_anchor` 自动取当前实际高度，跳变 >1 m 告警并改用当前高度。
- **Gate 6 高度认知修复**：Trace 证明地图/YAML z 正确，问题在前馈前视越门；修复为
  `s_ff` 钳到 `min(s+Lz, next_gate.s - ff_gate_margin)`。L6 的预测爬升在此基础上工作。
- **模型**：`yolo/weights/best.pt` 是检测(bbox)模型（~40 图，mAP50≈0.7，Recall≈0.5），
  上比赛需按 `yolo/README.md` 标注 500+ 图重训；`_use_keypoints:=true` 可自动启用 pose 关键点。
- **避障**：`avoidance_interface.py` 仍是预留（`active=False`）；`lidar_clearance.py` 已接入但未实飞验证。
