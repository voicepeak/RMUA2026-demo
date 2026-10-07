# RMUA 2026 无人机竞速

本仓库包含 ROS 控制与视觉源码、YOLO 权重、实验工具及开发文档。工作区为 `/home/tianbot/RUMA-by-helinjun`，Git 仓库位于其 `repo/` 子目录。

**当前交接请读 [docs/00_current_handoff.md](docs/00_current_handoff.md)。** 入口更新日期：2026-10-08。跟踪、三态地图、时空Planner/响应序列及独立执行器/共享Guard已实现，宿主/ROS各449项测试通过。源码和运行副本已同步69文件，默认仍为legacy；真实桥接失效时间和完整车辆段实跑尚未验收。代码收尾与证据见[执行层记录](docs/30_spacetime_execution_implementation.md)。

## 文档入口

| 文档 | 用途 |
| --- | --- |
| [当前交接](docs/00_current_handoff.md) | 当前状态、源码与运行副本、实验边界、卡点、恢复顺序；接手先读 |
| [动态避障重构设计审查](docs/26_dynamic_avoidance_design_review.md) | 基线调用链、模块接口、保留/废弃边界、已确认的分批修改清单与验收要求 |
| [DynamicTracker首批实现](docs/27_dynamic_tracker_implementation.md) | 新跟踪/预测接口、行为测试、连续场景预览、实测验证边界与后续实施 |
| [Rolling Occupancy实现](docs/28_local_occupancy_implementation.md) | 同帧原点、三态射线证据、动态归属、性能测试及交互重放 |
| [离线时空Planner实现](docs/29_spacetime_mvp_implementation.md) | 多情景响应、时空搜索、共享checker、完整制动、理想闭环证据及上线边界 |
| [执行层与发布收尾](docs/30_spacetime_execution_implementation.md) | 新旧入口、独立发布、共享Guard、版本/时效、故障验证及实跑门槛 |
| [无人机避障调研](docs/无人机避障调研_20261004.md) | 一手来源、可借鉴方法、已实施范围与后续依据 |
| [车辆修复记录](docs/25_car_command_guard.md) | 前期改动、逐轮比赛证据及失败分析 |
| [规则转写](docs/18_rmua2026_rules.md) | 既有规则资料；本地开发状态已从规则页分离 |
| [文档索引](docs/README.md) | 文档分类及阅读顺序 |
| [历史归档](docs/archive/README.md) | 早期 Stage 方案、旧交接、训练和此前实验记录 |

## 工作目录

| 路径（相对工作区） | 用途 |
| --- | --- |
| `repo/ros_ws/src/route_follower/` | 控制器编辑源 |
| `repo/ros_ws/src/rmua_gate_vision/` | 双目门/车辆视觉源码 |
| `repo/ros_ws/src/airsim_ros/` | 模拟器消息和服务定义 |
| `rmua_ws/src/` | Docker 中实际 ROS 运行源码副本 |
| `repo/yolo/weights/` | 门模型 `best.pt`、车辆模型 `car_score91_best.pt` |
| `repo/tools/` | 比赛启动、分段观察、离线重放、评估与测试工具 |
| `experiments/`、`frames/`、`runs/` | 比赛日志、点云、画面、历史实验及快照 |
| `simulator/simulator_12.0.0.5/` | 官方模拟器本地目录 |

## 操作约定

代码在 `repo/ros_ws/src/` 修改，确认版本后同步到 `rmua_ws/src/`。当前候选清单、实际启动快照和未提交改动的边界见[当前交接](docs/00_current_handoff.md)。飞行中不修改运行包；每轮使用新实验目录，不覆盖历史证据。

比赛入口为 `tools/start_seed123_race.py`，真实窗口使用 `--mode render`；启动条件、监视顺序和验收要求见当前交接第6–8节。整套实例停止命令是 `docker stop -t 5 rmua_noetic`。

此前开发路线和提交索引保存在 [README 历史副本](docs/archive/README_20261001_before_cleanup.md)，其中的“当前状态”和操作约定属于历史。
