# Alpha：比赛优先极简主线

## 目标（唯一 KPI）

固定 seed 集的得分表：完成阶段数、用时、失败点/失败原因。

**T0 完赛 > T1 稳定拿分 > T2 竞速。** 任何改动只认"得分表变好"。

> 每轮迭代怎么做、怎么记录、怎么判定：见 [ITERATIONS.md](ITERATIONS.md)。

## 原则

1. 单入口单出口：`./run.sh <seed>` → 终端一行状态 + 一行得分 + HUD 视频
2. 每轮唯一可信产物：HUD 视频 + `scoreboard.jsonl` 新行
3. **≤10 个源文件**，新增模块必须先删一个
4. 不写过程文档；证据仅对最终提交配置补做
5. 真值只用于离线评分，不进入控制回路
6. 不改官方模拟器/计分设置，不放宽飞行安全边界

## 路线（每阶段有退出条件）

| 阶段 | 内容 | 退出条件 |
|---|---|---|
| P0 基建 | worktree、仿真起停、得分表、视频 | `run.sh 123` 一条命令出分数+视频 |
| P1 采集建图 | pose_gt 引导侦察 → `gates_<seed>.yaml` | 复投影 IoU > 0.8；确认门是否跨 seed 固定 |
| P2 定位 | GPS+IMU EKF | 水平误差 < 0.5m（悬停+直线） |
| P3 在线穿门 | 单目角点 → PnP → 融合 | 连续穿第 1 门 ≥5 次；端到端延迟 <150ms |
| P4 第一段完赛 | 速度剖面+刹车+丢门恢复 | 官方 1→3 段切换 ≥3 次，用时 ≤151s |
| P5 跨种子 | 空图启动、失败归因 | 未调参 seed ≥1 次过第一段 |
| P6 后段任务 | 动态让行、线障、工厂读表、无 GPS 定位 | 六段完赛 ≥1 次 |
| P7 竞速优化 | racing line、速度包络、双策略 | 用时进目标区间且成功率不降 |

## 复用矩阵

| 来源 | 资产 | 用途 |
|---|---|---|
| main | `control/pwm_publisher`、`motor_allocator`、`safety_supervisor` | PWM 闭环（已验证悬停 0.1~0.27m） |
| main | `estimation/imu_gps`、`stationary_bootstrap` | 状态估计骨架 |
| main | `adapters/ros_noetic`、`deployment/`、`tools/start_gpu_env.sh` | 仿真/容器/接口 |
| main | `evaluation/metrics`（gps/延迟/指令追踪） | 得分表与归因 |
| beta | `route_follower`、`rmua_gate_vision`、`start_to_goal`、`race_runner` | 任务链（曾过第一段 151s） |
| beta | YOLO 权重与训练数据 | 门检测起点 |
| beta | `gates_seed123_recorded.yaml` | 采集格式参照 |
| MonoRace | 单目+PnP+已知门几何、RANSAC、IMU 饱和补偿 | 架构与鲁棒性 |

## 不做

VIO / 学习立体 / 占据图 / 时空规划 / 冻结协议 / 逐轮证据链。

## 风险预案

- 门跨 seed 变化 → P1 决策点，逐 seed 自动采集
- 感知延迟打不进 150ms → 自适应 ROI、降分辨率、事件触发
- 角速度异常 → 保护 + 自动重跑（≤2 次），不做根因考古
- 后段任务太难 → T0 优先：保守策略先到终点

## 得分表格式（scoreboard.jsonl）

```json
{"ts":"2026-10-30T12:00:00+08:00","seed":123,"stage":3,"score":0,"time_s":151.4,"fail":"gate4_det","commit":"abc1234","note":"conservative"}
```

## P0 当前状态（2026-11-02）

入口及起停、得分和 HUD 骨架已添加，但 P0 尚未通过：seed 123 的本地 offscreen 启动在话题就绪前退出，退出根因未确认，容器内未发现 ffmpeg。失败场及日志见 `runs/`、`scoreboard.jsonl` 和 ITERATIONS 记录。未产生视频或飞行结果。PWM 占位轨迹和落地确认尚未经过闭环验证；HUD 的速度、门框和阶段目前只显示不可用，官方分数未接入。下一步先恢复模拟器和编码器，再验证控制、落地与三场端到端结果。

后续检查确认：本机未设置 `DISPLAY` 且仅有 `:1`，原默认 `:0` 导致启动失败；已修正。`ffmpeg` 已解压到被忽略的 `alpha/.tools/`，并加载现有 `main/.runtime_ws` 的消息接口。最近一场 PWM 控制器运行了 14 秒、无报告故障，但宿主入口中断，落地没有确认，仍未达到 P0 验收。

迭代 04 的卡点：控制器按参考轨迹返回起始高度，但仅凭 IMU/GPS 估计不能确认接地；现有实验控制器结束时注销 PWM 发布者，并未提供已验证的落地后处置。按用户要求停止后续场次，未放宽保护或宣称落地成功。

补充：迭代 04 的录像已验证可解码，画面包含估计速度和占位任务阶段；落地服务虽在本地广告，但不在当前比赛接口基线白名单，尚未调用。HUD 写盘已移到独立线程，修改尚未经过飞行验证。

按用户更新的 P0 条件，Iter 05 已完成一场高度返回验证（误差 0.008m）；Iter 06 起飞时触发既有角速度保护，验收中止，第三场未运行。上述条件不要求证明接地，也不使用 land 服务；详见 ITERATIONS 及逐场产物。

## P0 验收结论（2026-11-02）

用户更新的占位验收为起飞、悬停 5 秒、返回初始高度 ±0.3m 并清理，不要求接地或门框检测。三个验收位均通过，其中第二位第一次触发角速度保护、一次重试通过，合计四场中的三场通过；逐场见 ITERATIONS 与本地 scoreboard。以上覆盖较早的“P0 尚未通过”状态说明；不代表官方比赛成绩。HUD 的门框检测留待 P3。

环境依赖：Docker `rmua_next_gpu`（或 `RMUA_CONTAINER` 指定的同挂载环境）、`/workspace/rmua_ws/devel`、`/workspace/main/.runtime_ws/devel` 及官方 simulator_12.0.0.5；后者本机实际链接到 beta 模拟器副本。容器需 ffmpeg/ffprobe，或在 `alpha/.tools/` 解包与容器兼容的 Ubuntu focal `ffmpeg` 包（`apt-get download ffmpeg` 后用 `dpkg-deb -x` 解到 `.tools/ffmpeg`）。记录在 runs/ 的源码哈希比未提交场次的 commit 更精确。

## P1 采集进展（2026-11-02）

`./run.sh <seed> --scout` 当前仅采集静止基线，记录前视图像、LiDAR 和开发用 debug pose_gt，明确标记未覆盖赛道；尚未实现真值引导飞行。seed 123/456 起点对照和局限见 ITERATIONS。现有 PWM 实验链距起点 1.8m 的保护与覆盖赛道的要求冲突，P1 门布局跨种子决策仍未作出；不使用 VEL 或修改官方设置。

后续 P1 使用独立 scout-only VEL 和 pose_gt 开发引导，比赛控制路径保持 PWM。seed 123/456 在 beta 1→3 候选路线均到达 60/126 个参考点，随后在同一区域无进展停止；共同覆盖区域的 0.25m 占用端点 IoU 为 0.716924。场景端点差异不能直接认定为门布局差异，且后续路线未覆盖，因此门是否跨 seed 固定仍未判定。录包、配置、故障及比较方法见 ITERATIONS 和 `runs/scout_*`；两次 scout 的 GameConfig 保持原样。

可在相同 ROS 容器环境下调用 `python3 alpha/collect.py --compare-coverage <seed123场次目录> <seed456场次目录> <输出json>` 复算局部占用端点对照。扫描与 pose_gt 使用最近时间戳并检查偏差；LiDAR 轴与外参依照本机模拟器 settings 作假设，未单独标定，未观测空间不视为空闲。
