# 2026-09-30 比赛推进与复现

任务：从已经起飞的 1→3 段继续推进，尽可能依次穿过计分门；需要新增 YOLO 感知时通知用户。

## 规则与完成证据

- 比赛顺序：1→3→5→8→12→10→7；检测门依次触发才计分。
- 前三段各限时 180 秒；8→12 起含工厂巡检，仪表猜测上报会使挑战失败。
- 碰撞与静止力场会终止比赛；本轮未修改 `GameConfig.json` 的碰撞/超时开关，仍均为 false。
- 规则文件：[18_rmua2026_rules.md](18_rmua2026_rules.md)。接口核对：
  [官方模拟器 README](https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/blob/RMUA2026-01/README.md)。
- `PASS` 是控制器基于感知门平面的几何记录，不能直接称作官方分数；本地没有可靠的官方成绩话题。
- 分段完成必须观察 `/airsim_node/end_goal` 变化；到达路线末尾或推进门索引不够。

## 已落实的控制修复

1. 雷达横移要求进入路径跟踪参考位置，取消会被路线纠偏抵消的额外侧向速度；所有横向命令仍受速度和加速度约束。
2. 大位移避障提前限速；没有达到净空余量的候选会停车；保留距机体前方 0.15–0.5m 的近墙点。
3. 已知前方被挡时，点云延迟不解除限速。
4. 门洞路径预留 0.8m 跟踪余量，减少弯道切边漏穿。
5. 使用先前双目观测的 seed 123 出口门位置，在进入枢纽前就调整横向路径。
6. 最后 25m 接官方终点 XY；飞行高度保持在标识坐标上方 2.5m，不直接降到标识所在道路表面。
7. 下一段反向飞同一条道路时，稠密 bbox 观测的旧路线法向由当前控制路线重新计算。
8. 视觉复位清掉待推理帧、旧位姿历史与旧地图；跨飞行轮次的帧不能重新污染地图。
9. 新增 `tools/race_runner.py`：只有官方终点更新且飞机在旧终点附近，才启动后续控制；支持前三段，工厂段明确请求巡检实现。
10. `frames/flight_probe.py` 增加初始位置与官方终点记录。

## 已验证结果

| 运行 | 结果 | 限制 |
| --- | --- | --- |
| d2，接手前 | s≈797.5m 卡住，控制器 17 PASS / 5 MISS | 尚未出枢纽 |
| e2/e3 | 补足避障位移，但仍在出口附近停止 | 局部雷达不能修正错误的远端门位置 |
| e4 | s≈1373.4m，30 PASS / 1 MISS | 模拟器进程退出；未确认终点切段 |
| e5 | 全量稀疏门图 + 12m/s，s≈1058m 后失败 | 已撤回该运行组合；不是可用基线 |
| e6 | s=1449.32m，35 PASS / 0 MISS / 0 SKIP | 最后触及道路表面；end_goal 仍为 3 号 |
| e7 | 抵达 3 号目标，36 PASS / 0 MISS / 0 SKIP；记录至 ROUTE_END 约 196s | end_goal 未切换；继承同一模拟器实例，赛事状态是否清空无法确认 |
| e8 | 完整重启模拟器后的干净验证；抵达目标，29 PASS / 1 MISS / 0 SKIP | 仍未切段，记录至 ROUTE_END 约 207s；未通过官方验收 |

e6 在 `(549.04, 518.05, -30.38)` 停住；官方目标是
`(545.56, 519.85, -30.21)`。终点前最后观测门中心约 z=-34.4m；
根据起始位姿与终点图像，道路标识坐标代表道路表面，末端应保留离地高度。
e7/e8 修正后约在 `(544.85, 520.32, -32.94)` 到达目标，目标 XY 距离小于 1m、离地约 2.7m。
官方目标仍未变化，不能把物理抵达称作比赛段通过。
到达后重复计算调速曾产生额外 STUCK 事件；已修复为一次 ROUTE_END 后保持零速度。

完整重启后的结果排除了仅凭机体 reset 就视为新比赛的做法，但仍无法确认漏触发门与超时分别造成的影响。
e8 缓存门 11 在 s≈251m 的控制器横向误差为 -1.68m；只有官方门几何/计分可确认实际是否计分。
综合数据保存在 `frames/race_progress_20260930.json`；报告用时从首条录到的遥测计算，非官方计时。

## 当前 seed 的地图和启动

`config/gates_seed123_recorded.yaml` 保存 18 个双目观测锚点：前段 17 个和一个出口后观测门。
坐标是传感器估计，非模拟器真值；未验证完整门总数，也不能直接用于其他 seed。
`config/guides_seed123_recorded.yaml` 是用前段门拟合的局部 spline 高度外推；后续仍需在线门纠正。
全量候选地图仅归档到 `frames/e4_measured_gates_full.yaml`，未作为基线使用。

在 `/workspace/rmua_ws` 的 ROS 环境中：

```bash
roslaunch route_follower route_follower.launch \
  gates_file:=/workspace/rmua_ws/src/route_follower/config/gates_seed123_recorded.yaml \
  guides_file:=/workspace/rmua_ws/src/route_follower/config/guides_seed123_recorded.yaml \
  gate_center_pull_max:=0 static_correction_max:=2 cruise_speed:=10

# 与第一段控制器并行观察；不发布速度命令
python3 /workspace/repo/tools/race_runner.py --out /workspace/frames/race_mission --cruise 10
```

正式新一轮先确认只存在一个速度发布者，完整重启模拟器并在 30 秒内起飞。
`reset` 服务实际会复位机体，但返回 `success:false`，不能仅用返回值判断是否复位，更不能据此宣称比赛状态已重置。
启动新比赛轮次时也要重新启动 `race_runner`，不沿用已经推进过的 stage 账本。

## 尚待确认与用户感知协助

- 180 秒内完成第一段、所有官方门按序触发、3→5/5→8 实飞都未验收。
- 后续道路缺乏完整地图；首次经过道路需要在线感知。工厂巡检、GPS 失效定位与风场处理未完成。
- YOLO 只有已有 `best.pt` 门 bbox 模型，存在墙框误检与后段漏检。
- 用户协助样本：`frames/yolo_hub_requested/README.md`，包含 03 出口净空、数字标识、多层出口和近墙负样本。
- 补充门洞、数字标识与计分门数据有助于继续；汽车/线状障碍模型在确实遇到时再接入。

验证：`python3 -m unittest discover -s tools/tests`，129 项通过；Python 编译与 `git diff --check` 通过。
