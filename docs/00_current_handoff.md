# RMUA 当前交接（2026-10-01）

本文件是唯一维护当前状态的交接文档。README 和工作区的 `交接入口.md` 只指向这里；逐轮技术证据见 [25 车辆修复记录](25_car_command_guard.md)，更早文档统一为[历史归档](archive/README.md)。后续交接应更新本文件，不再新增互相冲突的“当前状态”。

## 1. 接手先确认

- 用户要求先充电、关实例、暂停试跑。本轮文档整理未启动模拟器、ROS、视觉、控制器或测试。
- 容器 `rmua_noetic` 已停止，状态 `Running=false / exited`；比赛窗口及相关进程已关闭。
- **车辆避障没有修好，实跑表现回退；当前完整版本未验收。** 不应宣称“已修复撞车”“返程已通过”或“全场零碰撞”。
- 代码和实验记录保留，未回滚、提交、推送。后续恢复先比较基线和单项修复，继续试跑需遵循用户后续安排。

## 2. 当前版本在哪里

工作区：`/home/tianbot/RUMA-by-helinjun`；工作区本身不是 Git 仓库，Git 位于 `repo/`。Docker 将工作区挂载到 `/workspace`。

| 内容 | 工作区路径 | 状态 |
| --- | --- | --- |
| 控制器编辑源 | `repo/ros_ws/src/route_follower/scripts/` | 最新修复改动未提交 |
| 实际运行副本 | `rmua_ws/src/route_follower/scripts/` | 五个本轮修改模块与编辑源一致 |
| 门/车辆视觉 | `repo/ros_ws/src/rmua_gate_vision/` | 本轮未重新训练模型 |
| 门模型 | `repo/yolo/weights/best.pt` | 既有权重保留 |
| 车辆模型 | `repo/yolo/weights/car_score91_best.pt` | 已训练并接入，不能按旧文档重做“首次接入” |
| 官方比赛入口 | `repo/tools/start_seed123_race.py` | 固定 seed123，支持实际 render 窗口 |
| 分段调度/监视 | `repo/tools/race_runner.py`、`race_start_watch.py` | 控制官方目标切段及记录 |
| 模拟器窗口脚本 | `simulator/simulator_12.0.0.5/run_simulator.sh` | 已改为 windowed 1280×720；位于 Git 仓库外 |

本轮控制改动涉及 `execution_guard.py`、`point_index.py`、`predictive_avoidance.py`、`lattice_detour.py`、`route_follower.py`；回归改动在 `repo/tools/tests/test_execution_guard.py` 和 `test_route_follower_startup.py`。

[交接源码清单](handoff_snapshot_20261001.json) 保存整理时 Git HEAD、未提交状态、上述源码/运行副本的 SHA256、模型及测试日志校验和。**HEAD 不是当前完整代码版本**，需要结合未提交文件和校验和判断。文档整理不代表新一次飞行验证。

接口约定：世界位置/速度检查使用 NED（Z 向下为正）；`VelCmd.vz` 向上为正，检查前后要转换；`VelCmd.yawRate` 用度/秒。位姿话题 `/airsim_node/drone_1/debug/pose_gt`，终点 `/airsim_node/end_goal`，控制 `/airsim_node/drone_1/vel_body_cmd`。

## 3. 已验证什么、没验证什么

| 证据 | 可以确认 | 不能据此确认 |
| --- | --- | --- |
| 宿主和 ROS 系统 Python 的 221 项 unittest | 本轮已有回归测试通过；日志在 `experiments/car_collision_fix_20261001/host_tests.log`、`ros_tests_v17_final.log` | 比赛通过、物理无碰撞、实时预算满足 |
| 独立一阶响应静态车模型 | 合成场景最小净空约 2.166 m，推进到 x≈45.16 m | AirSim 官方场景验收 |
| 修改前 `race_visible_20261001_02` | 曾进入返程并观测推进约 119 m，仍有接近障碍/碰撞风险 | 稳定基线、无碰撞 |
| `race_car_fix_20261001_04` | HUD 有效 State2、Score65；返程约 6 m 后受阻 | 完整车辆段通过 |
| `race_car_fix_20261001_13` | 分段实验返程约 10.7 m，可靠近车区域出现约 0.238 m 近表面距离并结束 | 完整统一版本成功；完整车身模型可靠 |
| `race_car_fix_20261001_14`、`15` | 有效 State2；未重现 run13 的早期接触结束，分别停车/短退后超时 | 完整绕行通过、全场零碰撞 |
| `race_car_fix_20261001_16` | 起点连续慢退，未及时出准备区 | 可用的脱困出发流程 |
| `race_car_fix_20261001_17` | State2/Score65，仍在首组车附近；实际加载 220 项版本 | 221 项版本完成车辆段 |
| `race_car_fix_20261001_18` | UE 在首个位姿前崩溃，没有实际飞行 | 控制器成绩或控制失败原因 |
| `race_car_fix_20261001_19` | 完整 221 项版本第一阶段 180 秒超时；HUD Score17、OutTime0、Finished | 有效返程、车辆段修复成功 |

run19 最后有效 HUD：[hud_005.png](../../experiments/race_car_fix_20261001_19/hud/hud_005.png)，显示 `Time180.005 / Score17 / OutTime0 / StateFinished`。控制日志末尾 PATH 约 s323，之后 probe 的 s≈340 是残留本地进度，不能代替正式比赛状态。

run13–17 使用归档出程代码进入返程，再由新返程进程加载当轮修复版，属于分段定位实验。各轮 `controller_versions.json` 记录版本；不能包装成完整统一版本成功。

**实验名必须写全。** `race_dynamic_19` 是此前 docs/24 的离屏续修实验（观察约 160 m，HUD 未核实），`race_car_fix_20261001_19` 是本轮最新超时实验，两者不是同一轮。历史 `car_f5` 的 Score91 包含 Finished 后分数，不能作为有效 Score91 验收。

## 4. 本轮修补与主要回退

已找到并修补的漏洞：近车视觉面一概被丢弃；零速度指令检查遗漏完整惯性滑行；Z 指令未联合检查；其它参考的零限速错误施加到已接受路径；随机 KDTree 最近距离偶有高估；离散点间空隙不能表示连续顶棚。详细实现和证据在 [25](25_car_command_guard.md)。这些代码修补尚未形成可用的整场控制效果。

当前净空仍为 1.15 m 加 0.1 m 缓冲；制动模型通常使用配置最小值 4 m/s²、前视 30 m；0.8 s 响应常数只由一个局部速度衰减窗口支持。规划预算现为 1.5 s，绕行垂直范围恢复 ±1.5 m，侧移网格为 0.25 m。旧文档里的 0.5 s 预算、±0.5 m 限制和测试数均不是当前配置。

| 卡点 | 实际影响 | 当前证据边界 |
| --- | --- | --- |
| 车辆/道路几何不完整 | 离散点漏检；薄体积和局部面也可能堵住窄通道 | 顶棚/地面只在拟合支撑凸包内有效；没有完整车身占据或车辆未来运动预测 |
| 空间偏移规划与执行检查不一致 | 规划给出方向，最终检查持续否决；短退后仍无路 | 尚无刹停、侧移、绕多车、接回路线的统一三维时间轨迹 |
| 退出与重规划状态过于复杂 | 反复停车、短退；部分版本出发超时 | 目标冻结、提交余量、等待新路径等补丁仍未通过整段实跑 |
| 动力学校准不足 | 保护模型容易过保守，也未覆盖全速度响应 | 局部 0.8 s 拟合不等于高速转向/升降耦合验收 |
| 执行与后台计算耗时 | 控制验证/搜索可能超预算 | docs/24 已记录耗时问题；当前版本没有新的全面实时验收 |

迭代过程同时改动了感知几何、保护、搜索和状态机，未持续固定基线做单项比较，是性能回退的重要原因。run07、run10 中间版本异常时提前启动；比赛进入 Finished 后记录工具仍可能继续采样，必须筛选有效赛时数据。

工厂巡检、后续完整多阶段比赛、跨 seed 泛化及正式提交镜像仍未完成；当前优先任务是恢复可靠出程和可执行车辆绕行。

## 5. 保留的基线与证据

以下路径相对工作区：

| 路径 | 用途及边界 |
| --- | --- |
| `experiments/car_collision_fix_20261001/before/` | 本轮开始前的 `route_follower.py`、`execution_guard.py` 两个文件；不是完整源码快照 |
| `experiments/race_car_fix_20261001_13/initial_controller_sources/` | 五个关键模块的分段出程快照；用于比较，不代表完整已验收控制器 |
| `experiments/repair_continuation_20261001/` | 前轮离线重放、校验和、旧版本及性能证据 |
| `experiments/race_car_fix_20261001_01` 至 `_19` | 本轮原始日志、点云、画面和部分 HUD；详情见 25 |
| `experiments/car_collision_fix_20261001/` | 测试日志、合成验证、原始备份、文档归档映射 |
| `datasets/car_score91/`、`datasets/car_score91_yolo/` | 既有标注及训练数据；63 张图、424 框，不必重标/重训 |
| `repo/docs/archive/` | 原文与旧迭代方案；不能直接当当前配置执行 |

## 6. 恢复工作顺序

1. 保持当前实例关闭，先核对源码清单、有效 HUD 和基线备份。不要把 Git HEAD 或两个 `before/` 文件当成完整可复现基线。
2. 建立独立工作副本，组合并记录完整基线版本。优先恢复已能完成出程的行为，再逐项验证有证据的修补；不覆盖当前待修源码或原始实验。
3. 对同一车辆场景离线比较候选与实际响应，区分真实净空不足、保守几何封路、规划失败、状态机互相等待。一次只改变一项。
4. 生成从停车位置到穿过整组车辆并接回路线的完整可执行路径，统一制动、侧移、高度和时间预算；不靠缩小净空强行通过。
5. 用户恢复试跑后，每轮记录完整版本及 HUD；遇 Finished 结束该轮并归档。完整统一版本同时完成出程和有效车辆返程，才可认定本次修复通过。

接手可先查看 `git -C repo status --short`、本页链接和 `repo/tools/replay_avoidance.py`。比赛启动命令不属于当前暂停期间要执行的动作；启动方式查 `repo/tools/start_seed123_race.py`，用户要看实际比赛时使用 render 窗口。
