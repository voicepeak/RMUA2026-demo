# 时空执行层与发布收尾（2026-10-08）

本次补齐[已确认设计](26_dynamic_avoidance_design_review.md)的Phase 6实现：新旧入口、独立发布线程、隔离规划进程、共享Guard、版本与有效期、故障处理、诊断、可视化和冻结输入记录。另完成直线路线投影与高度路径缓存优化，以及停车余量软代价修复。源包和实际运行副本69文件一致，运行工作区重新构建；默认`planner_mode=legacy`。完整车辆段实跑验收仍未通过，不能把本次代码收尾写成赛事验收完成。

## 实现与执行契约

| 文件 | 行为 |
| --- | --- |
| `spacetime_navigation.py` | 同帧真实曝光原点→Tracker→三态地图；发布不可变射线时钟与track快照；持续spawn规划进程、限时回收挂起/退出任务 |
| `trajectory_tracker.py` | 只选首primitive；位姿/速度偏差、epoch、模型、计划时间检查；搜索后缀不持续执行 |
| `trajectory_executor.py` | 独立数值发布权威；最终XYZ指令保持一个周期并认证完整制动；计划过期或工作进程故障转制动，无法认证明确FAILSAFE |
| `execution_guard.py:certify_spacetime_command` | 纯检查入口；实际已补偿指令、旧驱动及coast延迟分支、完整停止均使用Planner的`TrajectoryCollision`，不搜索escape/shift/retreat |
| `spacetime_runtime.py` | 独立单调时钟发布线程；感知/归档/IPC在协调线程；唯一发布者、逾期计算结果拒绝、clock/reset/切段处理 |
| `route_follower.py` | `planner_mode=legacy|spacetime`；新模式任务线程只更新路线/门账本，执行器负责发布；显式NED→机体XY/向上Z转换；修复安装包配置目录 |
| `planning_snapshot_io.py` | 包内共享JSON/NPZ冻结输入codec，工具入口兼容；含指令/物理历史、观测时间与实际复核时刻、曝光原点 |
| launch / 三层比赛启动工具 | 模式与桥接参数完整传递、记录；默认legacy；非法模式/数值在启动前拒绝 |
| `planning_visualization.py` / `control_trace.py` | 有界ENU轨迹Marker、搜索后缀与认证命令/制动分开标注；独立录制planning_debug和实际指令 |
| `validate_spacetime_execution.py` / `summarize_spacetime_kpi.py` | 隔离工作进程故障与调度测量；实际ROS发布间隔和控制诊断分别汇总，不猜测官方比赛结果 |

20Hz是默认调度目标，最快响应情景与实际period一致；另外保留160/400ms反馈情景，共九情景。执行每次从最新测量和实际已发布命令历史生成一个最终指令，检查其保持段及完整反馈制动，随后只作等价坐标转换，发布成功后提交一次历史。Guard没有通过的零指令明确标记为未认证FAILSAFE，不能称安全等待。

规划进程预热后持续接收冻结任务，搜索预算100ms、任务总时限300ms；结果同时检查单调时钟龄期、轨迹绝对时刻、epoch、模型及绑定在轨迹/回复中的碰撞配置hash。下一次发布周期跨过首段有效期也必须制动。新观测推翻旧计划、道路参考变化、跟踪误差、时钟回退、切段/瞬移均使旧计划失效。

射线证据不可变地保存每个voxel的真实时钟，执行器自行按实际复核时刻求TTL，不等待地图写锁，也不把旧FREE复制成“当前FREE”。位姿时间与当前复核时间分别保存，不能借位姿滞后延长地图生命。更新中的较新LiDAR帧未处理完成时，不以旧地图认证新前进指令。

## 自查和证据

可随仓库阅读的证据在[验证目录](validation/spacetime_release_20261007/README.md)。完整本地日志、修改前运行包、冻结输入与交互预览在工作区`experiments/spacetime_execution_refactor_20261007/`，不把二进制实验目录整体提交。

| 验证 | 结果与边界 |
| --- | --- |
| 宿主 / ROS Noetic测试 | 各449项通过；覆盖同快照Planner/最终Guard、指令保持与完整停止、所有过期/失联/版本检查、真实新模式初始化及发布独占 |
| 隔离Catkin安装 / 安装包smoke | 构建安装通过；从安装目录导入codec、Planner、Guard，生成并复核计划及完整停止；不发布指令 |
| 实际运行包同步/构建 | 同步前53文件逐项匹配候选57基线，先完整备份；同步后69文件与编辑源SHA256一致，控制包构建通过 |
| 九类数值故障 | 0.2/0.6/1.2/2.2s延迟、挂起、退出、pose/cloud失联、reset全部通过；独立决策间隔各例P95最高52.00ms、最大61.27ms |
| 安装节点实际ROS零发布 | 111条全部零指令；间隔P95 50.42ms、最大50.58ms；退出码0；没有启动比赛，也没有验证桥接真实失效时间 |
| 五例理想FREE/CV闭环回归 | 静态绕行32步、横穿17步、双车21步、暂堵34步、窄通道9步，全部到达测试目标；每步执行首段及完整制动重复认证 |

五例回归的规划P95约352–356ms，仍是350ms预算下的离线验证，没有达到实时规划5–10Hz目标；数值故障使用静止理想观测和假定的外部桥接失效契约。零指令ROS测试证明安装/入口/周期发布可用，不能证明带运动误差和75000/200000点真实感知负载的20Hz完整闭环。

自查保留了失败记录：直线投影优化改变deadline内扩展顺序后，静态绕车在9步处无法继续认证。原因是软目标偏好停车尾段距离障碍仅毫米级的动作。新增停车末端净空的有界软代价后，重新跑五例通过；**没有放宽硬碰撞/道路限制、车体尺寸或不确定度。** 历史失败在`oracle_regression/`，修复单例在`static_stop_reserve/`，最终源码回归以`oracle_checked/`为准。

## 仍需实跑确认的条件

仓库的`airsim_ros/VelCmd.msg`没有duration/expiry字段，也没有桥接实现源码。当前无法核实桥接在发布者挂起或进程退出后持有速度多久。新模式默认`spacetime_bridge_verified=false`，明确输出FAILSAFE零；只有提供实际测得、且不长于执行周期的桥接失效契约才能允许运动，不能直接填一个假定数字当成测量。

后续仍需核实这个外部契约，完成真实连续LiDAR/Tracker/UNKNOWN/响应联合验证、运动中最终指令与备用制动交接、负载下发布最坏间隔、完整3→5车辆段A/B和重复验证。native序列迁移、轨迹平滑/commitment及赛事默认切换尚未验收。本次默认保留legacy，未宣称这些项目已完成。

## 复现

```bash
python3 -m unittest discover -s tools/tests
python3 tools/validate_spacetime_execution.py --out ../experiments/新的执行故障目录
python3 tools/validate_spacetime.py --out ../experiments/新的理想时空目录 --budget .35
python3 tools/summarize_spacetime_kpi.py ../某次实验/control_trace/control.jsonl
```

从`repo/`执行，验证目录须不存在。`--planner-mode spacetime`贯穿比赛工具，但完整比赛前必须满足上述实跑条件；本次没有发起新比赛。
