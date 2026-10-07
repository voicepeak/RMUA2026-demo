# 文档索引

整理日期：2026-10-08。当前状态只在 [00 当前交接](00_current_handoff.md) 维护。

## 接手阅读顺序

1. [00 当前交接](00_current_handoff.md)：当前同步版本、实跑失败结果与下一步实施/验证顺序。
2. [26 动态避障重构设计审查](26_dynamic_avoidance_design_review.md)、[27 DynamicTracker首批实现](27_dynamic_tracker_implementation.md)、[28 Rolling Occupancy实现](28_local_occupancy_implementation.md)、[29 离线时空Planner实现](29_spacetime_mvp_implementation.md)与[30 执行层与发布收尾](30_spacetime_execution_implementation.md)：跟踪/预测、三态地图、时空搜索/纯响应和执行层已实现，实跑门槛尚未验收。
3. [无人机避障调研](无人机避障调研_20261004.md)：Fast-Planner、EGO-Planner、RAPTOR、FASTER及应用边界。
4. [候选57清单](../../experiments/drone_avoidance_research_20261004/candidate_manifest57.json)和[第58轮结果](../../experiments/race_car_response_20261004_58/result.json)：候选57历史源码校验与最新实跑证据；其他记录见当前交接第9节。
5. [25 车辆碰撞续修记录](25_car_command_guard.md)：前期技术证据与失败分析；不是当前版本说明。
6. [18 规则转写](18_rmua2026_rules.md)：既有赛事资料；按需要查阅，不承担当前开发状态说明。

## 历史资料

[archive/README.md](archive/README.md) 是归档索引。01–17、19–24、旧根目录交接和旧 README 已归档，不删除历史证据；其中“当前”“下一步”和参数属于当时版本。

[截至第58轮的旧交接全文](archive/00_handoff_history_through_run58_20261004.md)保留逐轮记录。[2026-10-01源码清单](handoff_snapshot_20261001.json)是历史快照，不能代表当前运行包。

模型、标注、实验日志和运行快照仍在原目录，本次不整理或删除这些资产。以后更新当前交接与证据附录，不在多份文档重复维护状态。
