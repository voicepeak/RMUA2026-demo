# 当前代码与证据基线

核查日期 2026-10-28。此处固定的是**文档整理时读取的代码基线**，不是永远不变的 HEAD；后续开发应记录自己的 commit 和工作树状态。本次仅阅读代码及已有记录，没有运行模拟器或独立复现历史成绩。

| 分支 | 核查的提交 | 定位 |
|---|---|---|
| `main` | `384fede966f471e5b84783d848efd7acc611202c` | 新架构开发基线 |
| `beta` | `79350b1999f6e49b8b74640ef5655dfdf40e1091` | 旧实现和历史实验对照 |

## main：已存在与未证明

- `estimation/` 有 IMU/GPS 估计；`control/` 有有界 PWM 和 VEL 实验；`evaluation/seed_runner/` 有模拟器诊断和实验入口。开发镜像默认只读，见 [部署说明](../deployment/README.md)。
- 记录显示已进行低高度、有界悬停与阶跃实验；PWM 的偶发角速度冲击、持续失联与制动验收仍未关闭。正式比赛控制保持关闭。
- 地图、双目、门候选、动态障碍和局部规划有代码和诊断；在线自主穿门、第一段和工厂任务尚无通过证据。见 [实施状态历史记录](implementation_status.md)。
- 真值用于独立评测，不能推断其属于正式可用控制输入。传感器权限需以官方规则和服务器实际接口分别核验。

## beta：运行能力和依赖边界

- `tools/start_race.py --seed N` 是 beta 记载的比赛实验入口，`start_seed123_race.py` 为兼容入口。当前 beta 的 `tools/race_seed.py` 生成本轮空门图/空高度引导，不按 seed 选控制策略；但不等于跨种子通过。
- 当前 beta 的 `ros_ws/src/route_follower/scripts/route_follower.py` 和 `ros_ws/src/rmua_gate_vision/scripts/gate_yolo_node.py` 订阅 `/airsim_node/drone_1/debug/pose_gt`；历史方案的正式输入合规性不能由本地可用性推断。
- 较早的 seed 123 路线使用过 `gates_seed123_recorded.yaml` 和 `guides_seed123_recorded.yaml` 等预录参考。不要把这一历史配置当作当前 beta 默认入口，也不能把预录结果当作新里程碑通过。
- 历史 run 07 记录第一段 151.414 秒正式切段，第二段最后有效 HUD 113、约 1162.663 m；该轮使用 seed 123、VEL 链路及文档所述环境，记录为 75,000 LiDAR 点/秒，第二段未通过。参见 [beta 历史交接](https://github.com/voicepeak/RMUA2026-demo/blob/beta/docs/35_session_handoff_20261012.md)。具体历史配置须以对应原始实验清单核对；其完整日志不在本次检出的 main 中。
- beta 后期空图跨种子测试中，123/456/789 均未通过第一段；后续 456 最高记录 62 分但仍未通过。不同配置不能直接比较成绩。参见 [种子审查](https://github.com/voicepeak/RMUA2026-demo/blob/beta/docs/38_seed_independence_audit_20261013.md)与[后续交接](https://github.com/voicepeak/RMUA2026-demo/blob/beta/docs/40_paused_global_adaptation_handoff_20261013.md)。
- beta README 还提及不同时间的候选和测试状态；冻结验证记录包括一项失败。不要将测试数量视为当前飞行验收。

## 复现前

历史记录依赖 Linux、ROS Noetic、GPU、官方模拟器与本地未纳入 Git 的数据/容器；本次整理所在 Windows 检出未验证具备这些条件。先核对实际可用主机、源码/运行副本、模拟器版本、ROS 消息、容器与设备。原命令见 [开发命令](main_development_commands.md)，不可直接假定本机路径可用。
