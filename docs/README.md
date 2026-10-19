# 文档索引

整理日期：2026-10-13。最新持续续修见[39 坐标/门深度/准备](39_global_course_frame_and_bootstrap_20261013.md)。当前运行状态由[00 当前交接](00_current_handoff.md)维护，最新审查、修改与跨seed失败证据见[38 种子依赖审查](38_seed_independence_audit_20261013.md)。

## 接手阅读顺序

1. [00 当前交接](00_current_handoff.md)、[38 种子依赖审查](38_seed_independence_audit_20261013.md)：当前统一规则1335b84、缓存/手动入口cac4bd5、572项回归、同配置20–22三轮失败、保留版本和下一步。[37 冷启动](37_seed456_cold_start_20261013.md)及[36 迎面车关联](36_oncoming_scene_and_gate_layer.md)是此前17–19和14–16的历史时点。
2. [34 双目盲区与门位置记忆](34_stereo_blind_sector_and_gate_memory.md)：本轮实现与01–13实跑记录；车辆阶段尚未正式通过。
3. [26 动态避障设计审查](26_dynamic_avoidance_design_review.md)、[27 跟踪实现](27_dynamic_tracker_implementation.md)、[28 三态地图](28_local_occupancy_implementation.md)、[29 时空Planner](29_spacetime_mvp_implementation.md)、[30 执行层](30_spacetime_execution_implementation.md)：模块设计、实现和未验收边界。
4. [31 缝隙延迟与门间停滞](31_gap_latency_and_stationary_stall.md)、[32 参考带瞬态](32_reference_band_transient_release.md)、[33 缝隙发布一致性](33_gap_publication_consistency.md)：此前legacy续修、参数依据和逐轮失败证据。
5. [无人机避障调研](无人机避障调研_20261004.md)、[25 车辆碰撞记录](25_car_command_guard.md)：调研依据、前期诊断与修复记录。
6. [18 规则转写](18_rmua2026_rules.md)：既有赛事资料，不承担当前开发状态说明。

## 迁移后的历史资料

2026-10-19 已冻结到 beta；目录与归档位置见 [迁移说明](migration_20261019.md)，本次回归见 [冻结验证](validation_beta_20261019.md)。

## 方案、验证与历史记录

- [已确认的重构方案](specs/RMUA2026_dynamic_avoidance_refactor_plan.md)；占据避障草稿已随冻结提交保存，不作当前实现承诺。
- 历史 validation、archive 附件已移出当前树，可从标签 `archive/pre-refactor-20261019` 或本机 `/home/tianbot/RUMA-legacy-archive-20261019/tracked_iteration_artifacts/docs/` 恢复。

按用户要求删除了过期交接快照、旧README、非官方本地规则附录和旧源码清单，清单见归档索引。保留当前交接、有效方案和原始证据；旧文档的已提交版本可从Git历史查阅。

模型和运行快照保留；历史标注样例、图像和实验日志的位置以迁移说明为准。

- [40 暂停交接：未提交候选、回归失败及恢复步骤](40_paused_global_adaptation_handoff_20261013.md)（当前入口）
