# 文档索引

整理日期：2026-10-12。当前运行状态由[00 当前交接](00_current_handoff.md)维护；新会话完整说明见工作区[迎面车辆与交叉轨道交接](35_session_handoff_20261012.md)。

## 接手阅读顺序

1. [00 当前交接](00_current_handoff.md)、[37 seed456冷启动](37_seed456_cold_start_20261013.md)及[36 迎面车关联与门层级](36_oncoming_scene_and_gate_layer.md)：当前冷启动修复d99f1fb、17–19和14–16续跑、保留版本、GPU环境和下一步。
2. [34 双目盲区与门位置记忆](34_stereo_blind_sector_and_gate_memory.md)：本轮实现与01–13实跑记录；车辆阶段尚未正式通过。
3. [26 动态避障设计审查](26_dynamic_avoidance_design_review.md)、[27 跟踪实现](27_dynamic_tracker_implementation.md)、[28 三态地图](28_local_occupancy_implementation.md)、[29 时空Planner](29_spacetime_mvp_implementation.md)、[30 执行层](30_spacetime_execution_implementation.md)：模块设计、实现和未验收边界。
4. [31 缝隙延迟与门间停滞](31_gap_latency_and_stationary_stall.md)、[32 参考带瞬态](32_reference_band_transient_release.md)、[33 缝隙发布一致性](33_gap_publication_consistency.md)：此前legacy续修、参数依据和逐轮失败证据。
5. [无人机避障调研](无人机避障调研_20261004.md)、[25 车辆碰撞记录](25_car_command_guard.md)：调研依据、前期诊断与修复记录。
6. [18 规则转写](18_rmua2026_rules.md)：既有赛事资料，不承担当前开发状态说明。

## 方案、验证与历史记录

- [已确认的重构方案](specs/RMUA2026_dynamic_avoidance_refactor_plan.md)；占据避障草稿仍在specs目录，未跟踪且不作当前实现承诺。
- [时空模块验证](validation/spacetime_release_20261007/README.md)、[10月8日缝隙验证](validation/gap_release_20261008/README.md)、[10月10日验证](validation/gap_release_20261010/README.md)：对应当时版本的测试与证据。
- [历史归档索引](archive/README.md)：早期Stage设计、训练和实验记录；其中参数与开发进度属于当时版本。

按用户要求删除了过期交接快照、旧README、非官方本地规则附录和旧源码清单，清单见归档索引。保留当前交接、有效方案和原始证据；旧文档的已提交版本可从Git历史查阅。

模型、标注、实验日志、运行快照和基线备份仍在原目录。
