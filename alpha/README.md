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
| P0 基建 | worktree、仿真起停、得分表、视频 | 1 天 
| P1 采集建图 | pose_gt 引导侦察 → `gates_<seed>.yaml` | 1~2 天 
| P2 定位 | GPS+IMU EKF | 2~3 天 
| P3 在线穿门 | 单目角点 → PnP → 融合 | 3~5 天 
| P4 第一段完赛 | 速度剖面+刹车+丢门恢复 | 3~5 天 
| P5 跨种子 | 空图启动、失败归因 | 1 周 
| P6 后段任务 | 动态让行、线障、工厂读表、无 GPS 定位 | 1~2 周 
| P7 竞速优化 | racing line、速度包络、双策略 | 赛前 1~2 周 

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
