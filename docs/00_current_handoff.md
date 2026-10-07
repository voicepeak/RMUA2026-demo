# RMUA 当前交接（2026-10-08，执行层收尾）

这是唯一维护当前代码、运行状态、验证结果与下一步顺序的文档。[工作区交接入口](../../交接入口.md)和README指向这里。逐轮记录已完整保存在[第58轮后整理前历史快照](archive/00_handoff_history_through_run58_20261004.md)，其中旧的“当前”“准备启动”“暂停”不再代表现状。

## 1. 接手先确认

- 当前目标：按用户提供的[动态避障重构方案](specs/RMUA2026_dynamic_avoidance_refactor_plan.md)，将空间路线加指令否决改为动态预测加时空轨迹。[设计](26_dynamic_avoidance_design_review.md)已获用户确认，不再重复请求确认；[DynamicTracker、预测](27_dynamic_tracker_implementation.md)、[Rolling Occupancy/同帧原点](28_local_occupancy_implementation.md)及[离线时空Planner/纯响应序列](29_spacetime_mvp_implementation.md)已实现。[新旧入口、独立执行器与共享Guard](30_spacetime_execution_implementation.md)已实现并完成故障验证；实跑条件尚未验收，未启动新比赛。
- **完整3→5车辆段尚未通过。** 最好单次第54轮761.35m，约占1353.34m路线的56%；它的可见复跑停在5.17m，不能视为稳定方案。
- **源码与运行包现已同步69文件，默认`planner_mode=legacy`，保留候选57旧模式行为。** 新模式含跟踪、三态地图、时空搜索、纯响应、独立执行及Guard；native ABI12未变。宿主及ROS容器各449项测试通过、隔离Catkin安装和运行工作区构建通过。同步前53文件已逐项校验并完整备份，源码/运行清单和验证摘要见[发布证据](validation/spacetime_release_20261007/README.md)。新模式默认桥接契约未验证，输出FAILSAFE零，不得据此宣称实飞可用。
- 最新第58轮与第57轮使用相同控制包：出程149.707s通过，车辆有效最高19.271373m、Score65、最后有效StateTime54.426后提前Finished，原因UNKNOWN，未通过。第57轮在切段时模拟器SIGABRT，未取得车辆验证数据。
- 当前比赛、控制、监视和标定进程已结束；Docker `rmua_noetic`内模拟器仍显示第58轮Finished画面，桌面DISPLAY=:0。启动新实例前重新核对进程，避免多个实例叠加。
- **下一步：核实桥接命令失效契约，做真实连续感知/运动交接/负载时序验证及完整车辆段A/B和重复验收。** 新执行层入口已接好，20Hz零指令ROS发布与隔离故障验证通过，但不能据此证明运动闭环20Hz达标。Phase 7 native序列优化、平滑/commitment及默认切换尚未验收。
- 2026-10-07验证边界：带噪声/遮挡的61帧合成场景三个稳定ID、2s误差P95约0.089m；第58轮48帧过于稀疏，0个confirmed帧，不能证明实际车辆ID稳定。Finished实例只读采集8秒0帧。[证据与修改前53文件备份](../../experiments/dynamic_avoidance_refactor_20261007/)已保存；没有新比赛通过证据。
- Phase 3地图验证：51帧最近命中射线场景全部有效，地图更新及快照P95约20.52ms；75000回波有界测试P95约36.22ms，全部endpoint保留。只有29帧具有可信动态归属，其余保守保留静态；不能推断整体控制周期或实飞效果。[地图证据、源码清单与修改前备份](../../experiments/dynamic_occupancy_refactor_20261007/)已保存。第58轮旧档案没有曝光origin，被明确拒绝做FREE重建。
- Phase 4离线MVP：所有响应情景的时空动作、动态段内相交、UNKNOWN拒绝、实际等待及完整制动已实现。五个理想观测闭环场景（静态绕行/横穿/双车/暂堵/窄通道）到达测试目标，Case E速度突变拒绝由单独checker测试覆盖；使用理想FREE地图与受控CV对象，不代表实飞。350ms预算的每例P95约354–357ms，尚未达到5–10Hz目标；默认100ms冻结输入审计只能证明短前缀可用。[最终证据及此前高度交接失败](../../experiments/spacetime_mvp_refactor_20261007/)完整保留。新轨迹通常只认证首0.25s及完整制动，不能整条持续执行。
- Phase 6收尾：449项测试/两种环境、九类隔离故障、安装节点111条实际ROS零发布通过；P95发布50.42ms。五例最终离线回归全部通过；自查发现并修复了静态绕行停车余量退化，失败证据保留。详见[执行层记录](30_spacetime_execution_implementation.md)。
- 继续保留节能模式。不要用CPU条件不同的单次成绩归因算法；出现运行退化先查时钟、资源和进程，必要时重启。本次按用户指令提交并推送；版本以Git和发布清单为准。

## 2. 代码、快照与证据位置

工作区`/home/tianbot/RUMA-by-helinjun`本身不是Git仓库，Git在`repo/`；Docker挂载工作区到`/workspace`。本次收尾将此前候选57修订与本次重构一起入库；不要重置或清理模型、日志和用户资产。

| 内容 | 路径（相对工作区） |
| --- | --- |
| 编辑源 / 运行副本 | `repo/ros_ws/src/route_follower/` / `rmua_ws/src/route_follower/` |
| 当前候选清单 | `experiments/drone_avoidance_research_20261004/candidate_manifest57.json` |
| 改57前的56源包备份 | `experiments/drone_avoidance_research_20261004/before_candidate57/` |
| 最好54轮完整运行包、native及工具快照 | `experiments/race_car_response_20261003_54/controller_sources/`、`tool_sources/` |
| 54轮可见复跑失败现场 | `experiments/demo_best54_20261004_005107/`，含`failure_scene.png`、`failure_diagnosis.md` |
| 当前57包实跑 | `experiments/race_car_response_20261004_57/`、`experiments/race_car_response_20261004_58/` |
| 调研、重放、测试、时序及净空审计 | `experiments/drone_avoidance_research_20261004/` |
| 动态跟踪首批代码清单、隔离构建、测试与重放 | `experiments/dynamic_avoidance_refactor_20261007/`；说明见[实现记录](27_dynamic_tracker_implementation.md) |
| Rolling Occupancy、原点、隔离构建、测试与连续射线重放 | `experiments/dynamic_occupancy_refactor_20261007/`；说明见[地图实现记录](28_local_occupancy_implementation.md) |
| 离线时空搜索、响应状态、共享checker、冻结重放与理想闭环验证 | `experiments/spacetime_mvp_refactor_20261007/`；说明见[时空MVP实现记录](29_spacetime_mvp_implementation.md) |
| 执行层、真实ROS零发布、故障验证与运行同步备份 | `experiments/spacetime_execution_refactor_20261007/`；说明见[执行层记录](30_spacetime_execution_implementation.md) |
| 前期逐轮实验与标定 | `experiments/car_avoidance_implementation_20261002/README.md` |
| 调研结论及一手来源 | [无人机避障调研](无人机避障调研_20261004.md) |

`controller_versions.json`记录实际启动包、native、工具和模拟器启动脚本SHA256；`settings.json`记录该轮传感器和时钟。候选清单与实际启动快照都要核对，不能仅凭Git版本。

## 3. 当前控制策略与参数

1. 用同周期位姿把激光点转换到世界坐标，保留全部原始点；连续表面只在实测支撑范围内补面。动态关联只对连续且边界平移一致的局部观测赋予速度。当前表示不是完备车辆体积或自由/未知空间地图。
2. 独立进程向前搜索24m三维几何路径。共享当前观测网格，分别生成居中、左右约±1.25m偏好；优先原路径，无法产生通过检查的指令时尝试其他路径。缓存接头用最新位姿、点云复核；必要时尝试当前点云6m局部路径。
3. 对实际准备发布的XYZ指令预测延迟段、指令保持和完整制动尾段。三组响应情形结合80/160/400ms制动反馈周期，共9种情形；逐点检查原始观测、支撑面、动态预测、道路和地面约束。
4. 高度参考沿选择的路径和道路坡度变化；保留最后通过检查的停止高度路径。发布前使用最新独立位姿、速度和点云再次检查；失败转入制动回退，条件恢复单独标记。
5. 候选57中性回退：车辆已在缓冲内、实测速度和准备指令的水平速度都低于0.05m/s时，若中性候选通过原道路/地面约束，取消未通过检查的名义高度修正。仍报COMMAND_BLOCKED，不当作安全认证；有运动时保留制动。
6. 原粗恢复全部失败后，增加0.125m间隔局部三维目标，先批量筛目标净空、再按位移尝试。每个实际动作仍须满足全部逐点不接近、支撑面、道路、地面与9种完整停止检查。

| 参数 | 当前车辆模式 | 出程差异 |
| --- | --- | --- |
| 几何/正常停止净空 | 1.25m，路径代价偏好1.85m | 保留原出程策略 |
| 恢复目标净空 | 至少1.45m；条件恢复须每个预测尾段取得进展 | 新中性/细化恢复仅车辆模式 |
| 道路先验 | XY中心带2.25m、参考Z±1.25m；已有偏离仅按原回界规则处理 | 不是力场或真实地面真值 |
| 水平增速 / 减速 | 4 / 8m/s²；反向制动反馈0.6 | 统一经验响应框架 |
| 水平指令与实测速度差上限 | 4.5m/s，并保留垂直余量限制 | 出程coupling_limited=false |
| 实际高度耦合补偿 | 0.095 | 明确旧0.110 |
| 预测物理耦合 | 0.075 / 0.110 / 0.130 | 明确旧低值0.090 |
| 高度增益与余量 | 前20m由1→2、余量0→0.5m | 增益1、余量0 |
| 反应预算 | 0.25s加云龄，发布复核记录预算超出 | 0.35s，旧条件复核 |
| 提前障碍代价 | 前方9m，软代价 | 0 |

坐标为世界NED，Z向下为正；`VelCmd.vz`向上为正、yawRate为度/秒。位姿`/airsim_node/drone_1/debug/pose_gt`，终点`/airsim_node/end_goal`，控制`/airsim_node/drone_1/vel_body_cmd`。模型和边界为经验约束，不是形式化安全证明。

## 4. 最新实跑与验证结论

| 实验 | 有效车辆结果 | 结论 |
| --- | --- | --- |
| `race_car_response_20261003_54` | 761.347935m / Score94 / StateTime179.887 | 单次最远，180s超时；未通过 |
| `race_car_response_20261004_55` | 748.002795m / Score94 / StateTime179.538 | 180s超时；中间速度档未证明整段收益 |
| `race_car_response_20261004_56` | 566.710612m / Score91 / StateTime176.830 | 20s停滞诊断停止，官方UNKNOWN；未通过 |
| `demo_best54_20261004_005107` | 最高约5.66m，最终停在5.17m | 原54版复跑失败，程序与位姿流仍更新 |
| `race_car_response_20261004_57` | 无有效车辆验证数据 | 模拟器SIGABRT；通信连接清理堆栈，根因未定 |
| `race_car_response_20261004_58` | 19.271373m / Score65 / StateTime54.426 | 提前Finished、原因UNKNOWN；不是180s超时或通过 |

第58轮车辆有效148个遥测周期，约2.54Hz；导航耗时中位183.16ms/P95962.81ms/最大1879.86ms。有效发布中5次中性回退、4次原粗恢复、0次细化目标，因此不能声称细化恢复已有实飞效果。

车辆352个匹配原始观测帧最低净空1.034813m，13帧低于1.25m，0帧低于1m，最大采样间隔1.977s；出程1474帧最低0.824656m。采样缺口、遮挡与隐藏体积仍未覆盖，不能称连续安全。

12个实际静止帧带历史指令重放：原方案与仅细化目标均0/12；假设中性指令后下一帧仍静止的两周期反事实重放12/12可条件恢复，最低预测停止尾段约1.219m，仍低于正常1.25m。该假设未构成实飞验证。

`host_tests57.log`、`ros_tests57.log`各317项通过，覆盖原生/Python窄间隙恢复、新障碍拒绝、中性回退、运动制动、出程策略与道路约束。日志中预期的几何子进程EOF测试不影响最终OK；不要把测试数量当作比赛完成。

## 5. 调研结论与应用边界

完整比较和作者链接见[调研](无人机避障调研_20261004.md)。

| 方法 | 借鉴内容 | 本项目状态 |
| --- | --- | --- |
| Fast-Planner / EGO-Planner | 动力学可行轨迹、时间分配、按轨迹需求计算障碍信息 | 目前仍是几何路径加离散指令搜索，没有移植完整B样条优化 |
| RAPTOR | 多拓扑路径及提前观测风险区域 | 居中/左右偏好仅为近似，没有完整拓扑和主动感知规划 |
| FASTER | 持续保有已知自由空间中的安全备用轨迹 | 目前保存停止高度路径，尚无按时独立执行的完整备用策略；自由/未知空间条件也未建立 |

候选57只验证了局部恢复思路，不是上述算法的完整实现。论文实验和安全结论不能直接继承。

## 6. 执行层问题与新模式上线前要求

本节保留2026-10-04确认的执行层问题和实施约束；整体开发顺序现以[重构设计第10节](26_dynamic_avoidance_design_review.md#10-第一阶段具体修改文件与实施顺序)为准，以下要求纳入新模式上线前验证。对应执行层、版本/超时检查和制动复核代码已实现，详见[执行层收尾](30_spacetime_execution_implementation.md)；以下历史要求中的真实桥接契约、运动交接和整段实跑仍需验收。

1. **核对实际指令契约。** 追踪主节点、ROS发布、桥接订阅到模拟器执行，确认VelCmd的有效持续时间、更新/超时行为、延迟、坐标和时钟。工作区`airsim_ros`目前只有消息/服务定义，未找到桥接订阅实现；不能直接把发布间隔当作实际作用时长。记录规划位姿、发布源位姿、发送/接收和命令生效时间能取得的证据。
2. **整理预测与执行时序。** `velocity_response.py`中`envelope/envelopes`默认hold=0.35s；80/160/400ms是预测制动反馈周期。第58轮188条车辆发布记录，间隔中位0.2145s/P951.0740s/最长2.1906s，57个超过0.35s、50个超过0.4s。检查`route_follower.py`的计算、发布、`_recertify_publication`和`VelocityResponse.commit`交接，统一实际持有、制动开始与反馈更新时刻。
3. **设计并实现唯一发布者的执行层。** 将重规划和重建点索引等重计算移出指令执行路径。执行层按经验证周期读取原子快照，20Hz仅作初始设计目标；先测最坏耗时和超时行为，不能仅设置ROS rate就宣称达到。明确计划版本、观测版本、有效期、过期处理、重启/reset及坐标转换，避免两个线程同时发布或提交不同控制状态。
4. **交接完整备用制动策略。** 规划器交付已检查的路径、高度参考和与真实控制策略一致的备用制动信息。失去及时新计划时按匹配模型执行并复核备用策略；不能只是继续复用旧速度、发布心跳，或把XYZ一律归零。最新障碍、观测丢失、越界、过期与恢复必须有明确处理。持续检查物体、道路/地面及真实控制周期，暂不宣称FASTER式安全保证。
5. **先做故障注入和有界短程验证。** 注入规划延迟0.2/0.6/1.2/2.2s、挂起、退出、新障碍、位姿/点云丢失及reset，检查发送间隔、过期计划拒绝、实际制动开始/停止距离和高度、原始净空。测试应验证执行行为，而非仅验证计时变量。经验响应仍需实测跟踪误差与完整随时间包含检查。
6. **再做可见正式实跑。** 使用新实验目录、相同seed123/ClockSpeed1/75000点，核对启动版本、官方切段和原始审计；先查入口s1.34→4.42m的预测/实际偏离以及19m提前Finished的原因。完成完整3→5与重复验证后，再恢复原200000点及扩大场景。

执行层交付应包含执行时序契约、单一发布者的实现与失败处理、故障注入结果和有界实测；禁止以增加速度档或调整净空/恢复阈值替代这些工作。

## 7. 运行约束与验收

- 飞行中不改正在运行的控制包、native或入口工具；任何候选修改后重新测试、同步、保存清单，使用新目录。第58轮目录已占用，不能覆盖。
- 保留官方场景、碰撞、传感器物理设置与限时；当前验证显式75000点，原默认200000点还需最终复核。保留用户节能设置。
- 出程完成且3→5车辆段官方终点改变、进入下一阶段，才算车辆整段通过；还需原始记录和重复验证。Finished后的分数/进度不计入有效结论。
- 前3个竞速段各180s、准备区30s；当前调度只覆盖前3竞速段，工厂巡检和后续全阶段尚未实现。车辆通过也不能称整场6段完成。
- 有界标定29/30和多模型坐标极值只提供局部经验依据，不能替代随时间完整轨迹包含或连续安全。

## 8. 核对与复现命令

从工作区根目录执行。以下比赛命令用于后续实跑条件满足之后；本次已实现代码、完成自查并同步运行包，未启动比赛。

```bash
python3 -m unittest discover -s repo/tools/tests
docker exec rmua_noetic bash -lc 'source /opt/ros/noetic/setup.bash; source /workspace/rmua_ws/devel/setup.bash; python3 -m unittest discover -s /workspace/repo/tools/tests'
# 新目录必须未存在；桌面可见模式
python3 repo/tools/start_seed123_race.py --mode render --clock-speed 1 --lidar-point-rate 75000 --out experiments/新的实验编号
# render启动后，确认该新实例startup.json和位姿流已就绪，再运行HUD监视
python3 repo/tools/watch_debug_race.py --out experiments/同一新的实验编号 --display :0 --stall-seconds 20
# 结束后审计，不打印整个大型JSON
python3 repo/tools/audit_control_trace.py --run experiments/实验编号 --out experiments/审计输出.json
```

后台观察可使用`--mode background`，入口自动启动DISPLAY=:2的监视。第57轮发现监视可能覆盖已有失败原因，随后工具已增加保留已有stop.json，并将SIGABRT识别为模拟器崩溃；这些是记录工具修正，不是控制策略收益。

## 9. 继续工作前必读证据

- [无人机避障调研与下一步依据](无人机避障调研_20261004.md)
- [候选57清单](../../experiments/drone_avoidance_research_20261004/candidate_manifest57.json)
- [第58轮正式结果](../../experiments/race_car_response_20261004_58/result.json)
- [实际发布时序](../../experiments/drone_avoidance_research_20261004/publication_timing58.json)
- [原始观测审计](../../experiments/drone_avoidance_research_20261004/raw_audit58.json)
- [带实际历史的原恢复重放](../../experiments/drone_avoidance_research_20261004/stopped_entry_before.json)与[中性后的反事实重放](../../experiments/drone_avoidance_research_20261004/stopped_entry_candidate57.json)
- [第57轮崩溃说明](../../experiments/race_car_response_20261004_57/crash_diagnosis.json)；模拟器日志为完整堆栈依据。第57轮result/stop曾被不同终止路径记录，原因解释以日志及崩溃说明为准。
- [完整逐轮交接归档](archive/00_handoff_history_through_run58_20261004.md)、[早期归档](archive/00_handoff_history_through_run16_20261002.md)、[实施证据](../../experiments/car_avoidance_implementation_20261002/README.md)

`handoff_snapshot_20261001.json`和旧candidate_manifest为历史资料；57清单和58启动快照只代表旧实跑基线；当前源码以[发布清单](validation/spacetime_release_20261007/release_manifest.json)与Git为准。
