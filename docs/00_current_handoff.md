# 当前交接（2026-10-13）

**持续续修中：用户要求继续到跨种子适配完成，并明确要求推远端。最新实现f49b625、主机/ROS各591项测试通过，已推分支，运行副本同步。详见[39 坐标/门深度/准备续修](39_global_course_frame_and_bootstrap_20261013.md)。23–28均未通过第一阶段；当前29可见实跑，仍未整体验收。本段覆盖下文旧审查时点状态；任务未结束、没有安排休眠。**

先读[38 种子依赖审查与统一导航修改](38_seed_independence_audit_20261013.md)。用户观察seed456向三号入口运动、过门慢，要求全链路审查和全局修改。审查确认旧运行链有seed123专用预录门/高度输入、激进能力表及配置差异；不能继续称为已验证的全局优化。

## 当前版本与结果

- 分支`feature/seed-independent-navigation-20261013`；实现`1335b84`（统一现场建图与控制规则）、`cac4bd5`（耗尽缓存失效、清除手动入口旧图）。本地已提交，未推远端。
- 主机/ROS容器各572项测试通过。运行控制器/视觉源码与仓库同步，native ABI12未改；手动启动脚本根目录与仓库tools副本同步。
- 所有seed从空门/高度图启动、起点等待首张现场证据；双目占据、PersistentLidarScene、已接受路径速度代价对全部LiDAR阶段统一。标准入口为`tools/start_race.py --seed N`，旧文件名只作兼容包装。
- 历史图和seed123实验能力表仍保留，默认链不引用。Runner输入地图必须属于本轮，返程只用上一阶段实际位姿证据；不把期望参考当测量。
- 禁止BBox-only门拉偏XY，修完整停车响应投影和末段直线截角；所有阶段恢复搜索采用单独0.1s软预算。缓存路径耗尽按道路站距判定，保留弯道/末端短路径。
- 20/21/22已结束，固定源码和配置哈希一致，**全部未通过第一阶段**：123到时33分/798.320m；456地图不足HOLD、停滞24分/484.346m，官方UNKNOWN；789到时33分/798.610m。原始目录分别为`experiments/global_visible_20261013_seed123_20`、`...seed456_21`、`...seed789_22`。
- cac4bd5在三轮后修改，仅做572项回归和20冻结缓存复核；尚未新目录实飞。不能用三轮记录替它声明通过。入口、持续感知、泛化速度均未验收。

## 继续工作

用户要求的是全局优化；消除seed分支只是第一批工作。下一步优先检查多seed门检测/框体深度的覆盖和错误率、公共高度模型的可观测性、入口真实开口和道路/高度约束的冲突，再用固定配置复跑最新候选。无需也不允许为s≈798或某seed添加固定绕行坐标。

123/789最后可信门图停在约374m，依靠本轮早期观测校准的道路高度走到入口；456最后可信门约229m、模型不足，进入有限RECON后HOLD。权重best.pt是旧YOLO detection，元数据验证Recall约0.542，缺少跨场景独立验证，不能认为共享权重就是泛化模型。

13迎面车运动漏检仍未解决；双目正占据没有独立速度预测。PointIndex默认legacy缺少完整FREE/UNKNOWN语义；外部控制桥指令失效契约未测，时空模式不能绕过FAILSAFE。Factory inspection仍未实现，完整比赛未验收。具体证据及参数/传感器假设见38。

## 运行与保留边界

工作区`/home/tianbot/RUMA-by-helinjun`，Git仓库`repo/`；编辑`repo/ros_ws/src/`，实际ROS源为`rmua_ws/src/`，新轮前必须同步。飞行中冻结运行源码/入口工具，使用全新目录；保留官方碰撞、道路边界和180秒限时。

统一审查基线`9fd410b`保存在`local/20261013-before-global-audit-run19`；旧分支和07的114分候选`local/20261012-score114-run07`保留。旧轮失败、点云、图像、模型、标定和源码快照不删。用户未跟踪的`docs/specs/RMUA2026_local_occupancy_avoidance_plan.md`未改动、未提交。

目前20–22的比赛控制/视觉/Runner/记录进程已停，最后模拟器窗口保留Finished画面。没有安排本轮休眠；用户正在审查/观看，入口与全局性能还未搞定。

GPU恢复容器`rmua_noetic`保留：主机内核NVIDIA595.91.07与已安装595.99.02用户库不同，容器显式GPU设备、只读匹配595.91.07库。CUDA tensor预检通过RTX3060 Laptop、torch2.11.0+cu128。原容器`rmua_noetic_before_driver_recovery_20261012`和备份镜像仍保留。本轮未重启主机或替换驱动。环境恢复说明在`experiments/gate_car_repair_20261012/ENVIRONMENT_RECOVERY.md`。

可见验证模板（从工作区运行，目录必须全新）：

```bash
DISPLAY=:0 python3 repo/tools/start_race.py --seed 789 --mode render \
  --clock-speed 1 --lidar-point-rate 75000 --stall-seconds 60 \
  --obstacle-backend lidar_nav --planner-mode legacy --outbound-response coupled \
  --out experiments/NEW_UNUSED_DIRECTORY
```

## 历史证据入口

[37 冷启动](37_seed456_cold_start_20261013.md)属于d99f1fb时点：19最终33分、796.96m附近停滞，未通过，其“仍在运行”已过期。[36 迎面车关联](36_oncoming_scene_and_gate_layer.md)、[35 旧交接](35_session_handoff_20261012.md)保留13–16证据；第一段/车辆段区别及seed123默认数据等旧指令已由38统一规则覆盖。

设计/实现见[26 审查](26_dynamic_avoidance_design_review.md)、[27 跟踪](27_dynamic_tracker_implementation.md)、[28 占据地图](28_local_occupancy_implementation.md)、[29 Planner](29_spacetime_mvp_implementation.md)、[30 执行层](30_spacetime_execution_implementation.md)。31–34记录此前缝隙、延迟、高度/门筛选和双目盲区修复，不代表当前完整比赛通过。
