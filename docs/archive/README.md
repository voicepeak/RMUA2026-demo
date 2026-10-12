# 历史设计与实验记录

索引更新：2026-10-12。当前版本、运行状态及下一步统一见[当前交接](../00_current_handoff.md)。下列记录保留设计依据和逐轮证据，参数、测试数与“下一步”属于当时语境。

## 保留的归档

| 位置 | 内容 |
| --- | --- |
| `iterations/01`–`15` | Stage1–14开发设计与调试 |
| [16 全程审查](iterations/16_full_course_audit.md) | 早期代码审查 |
| [17 运行录制与计分](iterations/17_run_recording_and_scoring.md) | 录制与几何评估工具设计 |
| [19 比赛推进](iterations/19_race_progress_20260930.md) | 早期复现与接口判断 |
| [20 速度与返程](iterations/20_speed_window_timer_return.md) | 起跑窗口和返程记录；Finished后分数不作验收 |
| [21 车辆卡点](iterations/21_speed15_car91_diagnosis.md) | 15米/秒策略与停车分析 |
| [22 训练与速度](iterations/22_car_training_adaptive_speed.md) | 训练资产与动态速度记录 |
| [23 下降平滑](iterations/23_descent_smoothing_sensor_strategy.md) | 下坡与传感器策略试验 |
| [24 前轮续修](iterations/24_execution_guard_and_handoff_continuation.md) | 执行检查与历史实跑证据 |

## 已删除的过期副本

用户要求删除过期文档，本次删除以下8份文件。它们描述的当前版本、运行状态或本地配置已被后续交接替代；已提交版本可从Git历史查阅。

- `docs/archive/00_handoff_before_stereo_repair_20261012.md`
- `docs/archive/00_handoff_history_through_run16_20261002.md`
- `docs/archive/00_handoff_history_through_run58_20261004.md`
- `docs/archive/handoffs/RMUA_避障与速度修复交接_20261001.md`
- `docs/archive/handoffs/RMUA2026_control_cleanup_and_climb_solution.md`
- `docs/archive/README_20261001_before_cleanup.md`
- `docs/archive/rules_local_snapshot_20260930.md`
- `docs/handoff_snapshot_20261001.json`

此前文档整理的清单仍在工作区 `experiments/car_collision_fix_20261001/document_cleanup_20261001.json` 和 `experiments/drone_avoidance_research_20261004/handoff_cleanup_manifest.json`，它们是历史操作记录，不代表上述文件仍存在。

实验目录、模型、标注、代码与基线快照未删除。
