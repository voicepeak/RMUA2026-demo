# RMUA 2026 无人机竞速

本仓库包含 ROS 控制与视觉源码、YOLO 权重、实验工具及开发文档。工作区为 `/home/tianbot/RUMA-by-helinjun`，Git 仓库位于其 `repo/` 子目录。

**当前交接请读 [docs/00_current_handoff.md](docs/00_current_handoff.md)。** 更新日期：2026-10-01。比赛实例已关闭，用户要求暂停试跑；车辆避障未通过验收，最新完整版本第一阶段 180 秒超时、HUD Score17。此前的 221 项单元测试通过不能代表比赛成功。

## 文档入口

| 文档 | 用途 |
| --- | --- |
| [当前交接](docs/00_current_handoff.md) | 当前状态、源码与运行副本、实验边界、卡点、恢复顺序；接手先读 |
| [车辆修复记录](docs/25_car_command_guard.md) | 本轮改动、逐轮比赛证据及失败分析 |
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

代码在 `repo/ros_ws/src/` 修改，确认版本后同步到 `rmua_ws/src/`。最新控制改动尚未提交，交接快照见 [源码状态清单](docs/handoff_snapshot_20261001.json)。恢复工作先做离线分析和单项比较；当前暂停要求仍有效，不自动启动模拟器、视觉或比赛。

比赛入口为 `tools/start_seed123_race.py`，真实窗口使用 `--mode render`；后续用户要求试跑时再使用。整套实例停止命令是 `docker stop -t 5 rmua_noetic`。

此前开发路线和提交索引保存在 [README 历史副本](docs/archive/README_20261001_before_cleanup.md)，其中的“当前状态”和操作约定属于历史。
