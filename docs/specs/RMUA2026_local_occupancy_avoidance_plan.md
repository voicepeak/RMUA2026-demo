# RMUA2026 局部占据地图避障落地方案

> 版本：2026-10-08
> 适用仓库：`/home/tianbot/RUMA-by-helinjun`（编辑源 `repo/`，运行副本 `rmua_ws/`）
> 关系：本文是 [`RMUA2026_dynamic_avoidance_refactor_plan.md`](RMUA2026_dynamic_avoidance_refactor_plan.md) 与 `避障方案_纵向调度与一次性绕行_20261008.md` 的**可落地裁剪版**，不替换任务层、不引入 SLAM。

## 0. 一句话结论

**不做 SLAM，只做"已知位姿 + 多帧雷达 → 局部滚动占据图(FREE/OCCUPIED/UNKNOWN)"，把默认 legacy 避障从"每帧原始点硬裕度"升级为"局部地图上的单一碰撞谓词 + 固定失败阶梯"，用开关接入、可随时回退。**

## 1. 现状与证据（为什么做）

### 1.1 当前默认链路（`planner_mode=legacy`）

```
pose_gt ──┐
          ├─> PointIndex(原始点+支撑面/顶棚) ──> LidarNavigator.path/_commands
点云 ─────┘         (无跨帧自由空间记忆)
                       └─> ExecutionGuard(final) ─> VelocityResponse(响应模型) ─> VelCmd
```

- **无 SLAM**：位姿用仿真真值 `/airsim_node/drone_1/debug/pose_gt`，点云按同帧真值位姿转世界 NED；定位不是瓶颈。
- 规划器只有当前一帧信息；"点没打到"与"那里有障碍"不可区分，1.25m 硬裕度是在补偿没有 FREE 表示。
- `local_occupancy.py`（rolling voxel + 射线 FREE/OCCUPIED/UNKNOWN）与 `dynamic_tracker.py` 已实现，但只在 `spacetime` 分支使用，legacy 未接。

### 1.2 卡点证据（见 `experiments/band_transient_20261008/README.md`）

| 现场 | 现象 | 根因（已定位） |
| --- | --- | --- |
| run33 s551–555 / s751 / s783 | 反复 5–15s 慢/停 | 进入 1.25m 缓冲后只许脱困/刹车；`initial_buffer_blocked=True` |
| run34 s4.3 / run39 s10.3 | 起点车群卡死 | 全路径搜索 `pd=0`（即使路径缓冲降到 0.45m 仍无解）→ 无侧向"另一条路" |
| run37 s802.8 | 出程 3.1s 停顿 | 高速下探入缓冲（晚到回波），恢复慢 |
| run36 软策略 | s11 卡死 | 只放宽阈值没有空间记忆，候选选择退化 |
| run30–32 纵向模式 | 起步死锁/提前结束 | 缺少失败阶梯与感知预热 |

### 1.3 已完成的批次（远端 main）

- batch 4：带内/地面瞬态、包络基础裕度、侧向让行、局部预算、贴面应急撤退（`experiments/legacy_visible_20261008_33`：Score 95 / 车辆段 790m）。
- batch 5：纵向调度模块与 opt-in 入口（默认关闭，未通过实跑验收）。
- batch 6：缓冲内单调穿缝 + 指令 slew 提到仿真加速度 8（run39 出程 1449m 在窗口内完成，s802 卡点消失）。

## 2. 设计原则

1. **位姿不估计**：坚持 pose_gt；SLAM 只在真机部署时替换位姿源，规划/安全层不变。
2. **单一碰撞谓词**：Planner 与执行最终复核调用同一函数、同一裕度表，禁止两份标准。
3. **软硬分离**：规划裕度是 cost；唯一硬线是"动态障碍全缓冲 + 静态绝对线 + 紧急刹车距离"。
4. **失败有阶梯**：跟车 → 等待 → 一次性绕行/越顶 → 低速认证脱困 → 条件后退，顺序固定。
5. **决策有记忆**：轨迹/侧移承诺 ≥0.6s，禁止逐帧翻转；同一障碍段不反向横移。
6. **可回退**：全部新行为由参数开关启用，默认保持当前已验证配置；每步独立实验目录。

## 3. 架构总览

```
真值位姿 + 多帧雷达 + 同帧传感器原点
        │
        ├─> LocalOccupancy  rolling voxel raycast ── FREE / OCCUPIED / UNKNOWN
        │        ▲ 动态回波按 track 归属分离（低置信度保守保留）
        │
        ├─> LidarScene / DynamicTracker ── 车辆当前位置 + 预测bbox + 不确定度
        │
赛道/门链 (s,y,z) ──> 全局参考（Frenet）
        │
        └─> 局部规划阶梯（全部经唯一谓词认证）
              1 跟车/纵向调度（(s,v,t) DP + WAIT）
              2 一次性横移 / 垂直越顶（承诺0.6s）
              3 低速无路认证脱困（±1/1.5/2m 侧移、前后短移）
              4 条件后退（阻塞>5s 且后方FREE）
                     │
              响应模型 rollout（迟滞/slew/耦合/停车）
                     │
              执行层：唯一发布者 + 0.30s 租约 + 紧急刹车距离复核
                     │
                  VelCmd
```

## 4. 模块设计

### 4.1 局部占据图接入 legacy（核心新增）

**文件**：`scripts/local_occupancy.py`（已有）+ `scripts/route_follower.py`（接）+ `scripts/lidar_navigation.py`（查询）。

- **输入**：世界 NED 点云、该帧传感器世界原点（`_transform_cloud` 已保存 `lidar_frame.sensor_origin` 与 epoch）、`cloud_stamp/pose_stamp`。
- **参数**：`resolution=0.5m`；前向约 25m、横向 ±4m；`free_ttl≈1.2s`、`occupied_ttl≈2.5s`；滚动窗口随无人机移动。
- **更新语义**：
  - 同帧 hit 优先于同帧其他 ray 的 FREE；范围外/无效/过期 → UNKNOWN；
  - 可信动态 track 的回波从 static 层分离（按 track id/位置归属）；未确认/低置信度仍保守保留；
  - teleport/切段/时钟回退 → 清空并递增 epoch。
- **查询**：
  - `tri_state(samples) -> (OCCUPIED, FREE, UNKNOWN)`；
  - `free_distance(path_samples) -> 首个 OCCUPIED/UNKNOWN 的弧长`。
- **语义**：OCCUPIED 硬拒绝；UNKNOWN 首版硬拒绝（保守）；FREE 放行。原始 `PointIndex` 保留为最终复核（防地图漏检）。

### 4.2 唯一碰撞谓词

**文件**：`scripts/trajectory_collision.py` 收敛为唯一实现，Planner/Guard 共用。

```python
clearance(samples, times, occupancy, tracks, route_bounds) -> (min_clearance, reason)
```

- **动态障碍**（tracks/LidarScene，按时间预测 + 不确定度膨胀）：**硬**，需求 = 全缓冲（默认 1.25m；可视情保留侧向 0.9m 让步）。
- **静态 OCCUPIED**：**硬**，需求 = 绝对线（建议 0.45m 起，可标定）。
- **静态 FREE**：放行；**UNKNOWN**：硬拒绝（首版）。
- **道路/高度/地面**：`band_hard` 为硬上限（建议 2.0m），带内偏差计入 cost；地面/顶棚用实测支撑面。
- **紧急刹车距离**：`d_stop = v·0.25 + v²/(2·6) + 0.30`，作用于实际发布的水平速度与走廊自由距离；不足即刹车。
- 返回值必须带首个冲突的：时间、位置、来源（static/dynamic/unknown/bounds）、最小间距。

### 4.3 局部规划阶梯

**文件**：`scripts/longitudinal_planner.py`（已有 DP/一次性/车辆盒）+ `scripts/lidar_navigation.py`（脱困入口）。

1. **跟车/纵向调度**：`plan_longitudinal(s,v,cars,cfg,free_soft)`；(s,v,t) DP，WAIT 节点，跟车距离 `1.5m+v·0.3`。
2. **一次性横移/越顶**：纵向无推进解时枚举 `y∈{±1,±1.5,±2}`；越顶条件 `车高+0.4 < 带上沿`；选定后承诺 0.6s，禁止反向（`long_last_lateral`）。
3. **低速无路认证脱困**：路径搜索无解且 `v<2m/s`、未贴面时，枚举侧移/前后短移，用 `guard.escape/recovery_command_ok` 逐个认证（每个回波不低于 `min(当前距离,1.25)`，尾段有进展）。
4. **条件后退**：阻塞 >5s 且后方 FREE 走廊 ≥3m，才允许 ≤1m/s 后退一小段；进入前记 `retreat_pending`，恢复需重新认证前进。

### 4.4 执行层

- 唯一发布者 + 固定周期（沿用 20Hz）；`CommandDeadline` 0.30s 租约；迟到计算丢弃。
- 发布前对实际指令做 4.2 的最终复核；失败 → 有界反馈制动（沿用 batch 2）并标未认证。
- 反馈制动线程不修改主响应模型；下一周期发布前同步实际历史。

### 4.5 遥测/日志/可视化（必须）

- 每周期计数：`N1 静态占图淘汰 / N2 动态淘汰 / N3 边界淘汰 / N4 动力学淘汰`；`WAIT / NO_PATH / UNKNOWN_BLOCKED / EMERGENCY` 分开记录。
- `clearance` 结构输出：`occupancy_state`、`min_clearance`、`conflict_source`、`emergency_free/need`、`commitment`。
- Markers：FREE 绿、OCCUPIED 红、车辆当前位置蓝、预测橙、候选黄、选中紫；显式 NED frame。

## 5. 建议参数表

| 参数 | 建议值 | 说明 |
| --- | --- | --- |
| voxel resolution | 0.5m | 前向 25m、横向 ±4m |
| FREE/OCCUPIED TTL | 1.2s / 2.5s | 过期转 UNKNOWN |
| 规划软裕度 | 0.60m | cost，不判死 |
| 静态绝对线 | 0.45m | 唯一硬线之一（可标定 0.35–0.5） |
| 动态需求 | 1.25m（侧向让步 0.9m） | 预测+膨胀 |
| 紧急刹车 | `v·0.25+v²/12+0.30` | 唯一动态硬线 |
| 高度带 band_hard | 2.0m | 越带先加 cost，再硬 |
| 承诺时长 | 0.6s | 横移/越顶 |
| 切换阈值 | 省时 15% 或预测碰撞 | 决策记忆 |
| 跟车 | 1.5m + v·0.3 | |
| 等待上限 | 12s | 超时升级阶梯 |
| 后退门槛 | 阻塞>5s 且后方≥3m FREE | 最后手段 |
| 指令 slew | a_up=a_down=8 | 与仿真 va 一致（batch 6 已设） |

## 6. 实施步骤

**P0（解卡，先离线后实跑）**
1. `local_occupancy` 接入 legacy：`route_follower` 每帧更新+epoch 管理；`lidar_navigation` 候选/路径检查改用占图三态，原始点保留最终复核。
2. 单一谓词：Planner 与 `_recertify_publication` 共用 `trajectory_collision.clearance`。
3. 参数开关：`lidar_occupancy:=false|true`、`lidar_hard_floor`、`lidar_band_hard` 默认关闭。
4. 冻结帧 A/B 清单（见第 7 节）全绿后才允许实跑。

**P1（阶梯与记忆）**
1. 低速无路认证脱困（4.3-3）。
2. 一次性横移/越顶 + 承诺（4.3-2，复用 `longitudinal_planner.choose_one_shot`）。
3. 条件后退（4.3-4）+ 遥测计数。

**P2（性能与标定）**
1. 耗时预算：规划 ≤100ms、执行 20Hz、发布间隔监控。
2. 参数标定：静态线、动态膨胀、TTL、band_hard。
3. 只在结构正确后做平滑/剪枝。

**改动文件清单（预计）**
- `scripts/route_follower.py`：占图更新/epoch、阶梯调度、承诺、遥测。
- `scripts/lidar_navigation.py`：占图三态查询、低速脱困入口。
- `scripts/trajectory_collision.py`：唯一谓词收敛。
- `scripts/local_occupancy.py`：与 live 帧对接的小改动（origin/stamp/epoch）。
- `scripts/longitudinal_planner.py`：一次性横移/越顶与承诺参数复用。
- `launch/route_follower.launch`、`tools/race_runner.py`、`tools/race_start_watch.py`：开关与参数。
- `tools/tests/`：占图三态、谓词一致性、阶梯状态机、承诺、冻结帧回归。

## 7. 验收

### 7.1 冻结帧 A/B（必须全过）

| 帧 | 位置 | 通过标准 |
| --- | --- | --- |
| run39 s10.3 | 车辆段起步车群 | 出现经认证的侧向路径并前进，不 COMMAND_BLOCKED |
| run33 s551–555 | 门间走廊 | 全程 ≥0.5m/s 无 `initial_buffer_blocked` 长时间停 |
| run33 s751 / s783 | 贴面/半门 | 前进或后退脱困，不超时 |
| run37 s802.8 | 出程晚到回波 | 停顿 <1s，出程窗口内完成 |
| run34 s4.3 | 起点 | 有路径/绕行，不卡死 |

### 7.2 同 seed 实跑（3 次）

- 每段限时内到达终点；碰撞 0；`COMMAND_BLOCKED` 帧占比 <5%；
- 非计划停车（>0.5s）每车辆段 ≤1 次；左右决策切换 ≤1 次；后退 0（正常跑）；
- 规划耗时 P95 <100ms；发布间隔无 >0.35s 缺口。

### 7.3 记录

每轮新实验目录 + 原始点云/遥测/控制轨迹；冻结复放脚本与结果入库；失败现场保留。

## 8. 风险与边界

- 本文是工程方案，不是形式化安全证明；硬安全仍靠紧急刹车距离与原始点复核。
- UNKNOWN 首版硬拒绝偏保守；放宽需有冻结帧与实跑证据。
- 局部图与位姿真值同源，仿真无漂移；真机需替换位姿源并重新标定。
- 禁止用新增恢复状态机或阈值补丁掩盖 KPI 不达标；先查谓词与图，再查预测，最后查 cost。

## 9. 与现有代码/文档的关系

| 模块 | 处置 |
| --- | --- |
| `local_occupancy.py` | **接入** legacy（P0 核心） |
| `dynamic_tracker.py` / `lidar_scene.py` | 动态层保留；tracker 可选接入 |
| `point_index.py` | 保留为最终原始点复核 |
| `velocity_response.py` / `response_rollout.py` | 保留（执行模型资产） |
| `trajectory_collision.py` | **收敛**为唯一谓词 |
| `longitudinal_planner.py` | 阶梯 P1 复用 |
| `lidar_navigation.py` | 路径搜索与候选检查改接占图；旧硬缓冲逻辑保留开关 |
| `execution_guard.py` | 保留最终复核与刹车；否决权随谓词统一 |
| `st_lattice.py` / `spacetime_runtime.py` | 不动，保持 opt-in |
| batch 4/5/6 行为 | 默认保留；新功能开关默认关闭，实跑验收后再考虑默认切换 |

## 10. 证据与参考

- 卡点现场与批次结论：`experiments/band_transient_20261008/README.md`
- 执行层与响应模型：`repo/docs/30_spacetime_execution_implementation.md`
- 动态跟踪/占据/时空实现记录：`repo/docs/27_…md`、`28_…md`、`29_…md`
- 当前运行状态与约束：`repo/docs/00_current_handoff.md`
- 相关前序方案：`repo/docs/specs/RMUA2026_dynamic_avoidance_refactor_plan.md`
