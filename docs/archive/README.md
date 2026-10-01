# 历史文档归档

这些文档保留原阶段的分析、参数、试验和计划，统一由 [当前交接](../00_current_handoff.md) 取代。归档日期：2026-10-01。Markdown 相对链接已随文件位置调整；代码块中的命令、行号和路径保留当时语境，不是当前操作指令。

## 归档分组

| 位置 | 内容 | 为什么归档 |
| --- | --- | --- |
| `iterations/01`–`15` | Stage1–14 开发方案与调试 | 早期代码结构、默认参数和验证阶段 |
| [16 全程审查](iterations/16_full_course_audit.md) | 2026-09-29 审查 | 其中开发状态和优先级已被后续工作覆盖 |
| [17 运行录制与计分](iterations/17_run_recording_and_scoring.md) | 录制/几何评估工具说明及历史进展 | 工具设计可参考，运行状态不是当前结论 |
| [19 比赛推进](iterations/19_race_progress_20260930.md) | 早期复现与接口判断 | 后续已明确起跑计时与有效赛时边界 |
| [20 速度与返程](iterations/20_speed_window_timer_return.md) | 起跑窗口和返程记录 | Score91 包含 Finished 后分数，不能作为有效验收 |
| [21 车辆卡点](iterations/21_speed15_car91_diagnosis.md) | 15 m/s 与停车分析 | 当时车辆模型尚未接入，现已接入 |
| [22 训练与速度](iterations/22_car_training_adaptive_speed.md) | 车辆训练资产及动态速度记录 | 训练证据仍可参考，比赛进度已更新 |
| [23 下降平滑](iterations/23_descent_smoothing_sensor_strategy.md) | 下坡与传感器策略 | 此前参数与实验；不能代替本轮结果 |
| [24 前轮续修](iterations/24_execution_guard_and_handoff_continuation.md) | 195 项测试与 `race_dynamic_19` 离屏观察 | HUD 未核实；区别于本轮 `race_car_fix_20261001_19` |
| [旧修复交接](handoffs/RMUA_避障与速度修复交接_20261001.md) | 原工作区根目录交接 | 对应更早的 race_dynamic_15 和停止状态 |
| [控制整理与爬升方案](handoffs/RMUA2026_control_cleanup_and_climb_solution.md) | 2026-09-29 长方案 | 旧基线的规划建议，并非当前已完成实现 |
| [旧 README](README_20261001_before_cleanup.md) | 开发路线、提交索引和旧交接段落 | 状态混杂、195 项测试、尚未接入车辆等描述已过时 |
| [规则页旧本地附录](rules_local_snapshot_20260930.md) | 规则文档原附录三 | 非官方开发快照，部分“未做”已过时 |

归档移动清单及原文件校验和：工作区 `experiments/car_collision_fix_20261001/document_cleanup_20261001.json`。实验目录、模型、标注和基线源码均保留在原处。
