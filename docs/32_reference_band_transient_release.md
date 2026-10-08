# 参考带内爬升瞬态放行与 run18/19 实跑（2026-10-08）

依据：避障减速/停止代码审查与 `legacy_visible_20261008_17` 冻结帧诊断。证据目录 `experiments/band_transient_20261008/`。

## 问题

车辆段出现"路径净空充足却全候选被拒"的间歇刹停：

- run17 车辆段 s0–13m 与 s341–352 多帧 `COMMAND_BLOCKED`，`path_clearance` 1.5–1.75m、`candidate_count` 60–120。
- 冻结帧 `1791437451.983_s343.0` 诊断：所有候选共享的停车垂直剖面在响应前缀内越过参考 Z±1.25m 带上界（峰值 +0.277m，起点在带内 -0.407m，各情景尾段回到 -0.498m）。原因是实测垂直速度（NED -0.9m/s）在反应延迟+保持窗口内继续运动；原 `_bound_ok(timed=True)` 对起点在带内只允许 1mm 瞬态，直接否定全部横向候选。

## 修改（`scripts/lidar_navigation.py`，源/运行副本已同步）

1. `inside_transient_allowance(velocity,age)=min(0.6, 0.15+|vz|*(reaction+age))`。
2. `_bound_ok` 增加 `inside_transient`：起点在带内时允许峰值超出边界不超过该额度，且每个情景尾段必须回到带内；默认 0 保持原严格行为。
3. `_floor_ok` 的两个高度带检查使用该额度；出发地面/地板检查不变。
4. `select` 按实测垂直速度与云龄计算 `inside_transient`。
5. 原始回波 1.25m 停车包络检查未改动。

补丁：`experiments/band_transient_20261008/lidar_navigation.patch`（基线 run18 运行快照）。

## 验证

- 宿主 492 项、ROS Noetic 容器 492 项通过（含新增 2 项带内瞬态回归）。
- 冻结帧复放：s337/s343 由 `COMMAND_BLOCKED/feasible=false` 变为 `LIDAR_TRACK/feasible=true`。
- run18（修改前）：HUD Finished Time338.905/Score96；监视器中断导致有效窗口只到 StateTime45s，非可比较证据。
- run19（补丁）：HUD Finished Time344.917/Score94；车辆段有效窗口完整。s>50m：`COMMAND_BLOCKED` 36→29、`COMMAND_BRAKING` 24→10、慢速区间 7→2；s341–352 带内阻塞簇消失。单次实跑受车辆场景随机性影响，不能当作成绩结论。

## 未解决

- 车辆段 s0–16m 仍有短刹停：出发地面检查拒绝所有候选，`path_z-floor_limit` 峰值 +0.027~0.055m，超过现有 1mm 定时瞬态。涉及真实地面安全，本次未放宽。
- s553 附近窄缝（净空 1.3–1.5m）按设计慢速通过，未改动。
- 停在缓冲外且无几何路径时缺少原地让行入口（run18 s≈806 刹停）；本次未改。

## 追加：停车包络基础裕度放行（同日第二项）

run19 车辆段 s549-554 窄缝：路径净空 1.45m，响应模型横向滞后使候选停车包络最低 1.155m，被统一 1.25m 阈值全部拒绝，只能爬行/刹停。

- 新增 `ExecutionGuard.command_clearance_components`（原始回波/动态预测分离）与 `LidarNavigator.envelope_margin`。
- `_commands` 在 `lidar_envelope_margin` 启用时允许模拟包络对原始回波低至 1.15m；动态预测仍须 ≥1.25m；路径搜索/缓存前缀保持 1.25m。
- launch 参数默认 0（关闭）；比赛入口 `race_start_watch.py`、`race_runner.py` 设 `lidar_envelope_margin:=1.15`。
- 冻结帧 s551.1 由 `COMMAND_BLOCKED` 变 `LIDAR_TRACK`；宿主/ROS 各 493 项通过；实跑 `experiments/legacy_visible_20261008_20`。
- 边界：非形式化安全证明；要求全样本 ≥1.25m 审计时应关闭该参数。

## 同日续：地面瞬态、局部规划与应急撤退

- 出发地面检查允许 `min(0.1, inside_transient)` 有界瞬态并要求尾段回收；run23 起飞段卡顿降至 2.2s。
- 局部重规划 6→12m、预算 .03；几何搜索重试预算新增 `lidar_budget`（车辆段 0.15）。
- 门间口袋（run26 s≈568，0.5m 内 1370 回波）：脱困"静止"判据改用水平速度；`escape` 去掉与逐点分离重复的一阶否决；贴面退化态（最近 <0.6m 且 0.5m 内 ≥16 点）沿局部外法向 ≤0.6m/s `EMERGENCY_RETREAT`。
- 宿主/ROS 各 493 项通过。实跑：run23 804m/p50 4.75；run27 776m/p50 5.28，口袋未再卡死。
- 未解决：s550–575 走廊全路径搜索被道路高度带拒绝，反复损失 6–18s；完整 3→5 未通过。详见 `experiments/band_transient_20261008/README.md`。


## 批次合并时的最新状态

已完成迭代的完整源码在宿主/ROS各495项测试通过。后续侧向裕度参数 `lidar_side_buffer` 已接入（车辆入口0.9m；关闭为0），前方逼近和动态预测仍使用完整余量；非有限遥测值经过JSON清洗，单次异常不再直接中断诊断发布。

run29实跑在车辆段约9.44m由20s无进度监视停止，HUD仍为State2，官方结果UNKNOWN。冻结场景放行与合成测试不能替代稳定通过；当前仍有起飞停滞和车辆段耗时问题，完整3→5尚未通过。侧向/包络裕度、参考高度/地面瞬态与EMERGENCY_RETREAT包含经验性放宽或未认证回退，不能宣称全部样本保持原1.25m余量或具有形式化安全保证。

完整日志和原始点云保留在工作区；远端随版本提交最终测试日志、源码/运行包SHA256、run17/19/23/27/29摘要与冻结诊断，见[发布证据](validation/gap_release_20261008/README.md)。
