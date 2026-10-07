# 离线时空Planner与响应序列实现（2026-10-07）

本批实现已确认[设计](26_dynamic_avoidance_design_review.md)的Phase 4离线MVP及其必需的Phase 5 Python纯响应序列接口。正常搜索包含前进、减速、等待、横向偏移与上下动作，提前检查动态障碍的未来位置。本记录描述Phase 4/5实施时的状态。后续新入口/独立执行器/Guard已经实现并同步运行包，见[执行层收尾](30_spacetime_execution_implementation.md)；默认legacy，未启动新比赛，不宣称车辆实飞通过。

## 1. 实现范围

| 模块 | 职责 |
| --- | --- |
| `scripts/route_coordinates.py` | 冻结局部路线采样、世界NED↔(s,y,z)、道路/高度/已提供floor边界；冻结高度/横向目标控制策略 |
| `scripts/trajectory_types.py` | 只读响应状态、绝对时间轨迹、PlanningSnapshot、TimedTrajectory、冲突与计划结果；保留命令及物理积分记忆 |
| `scripts/response_rollout.py` | 固定参数多情景primitive/sequence、旧指令反应延迟、反馈制动至全部情景停稳；不修改live模型 |
| `scripts/velocity_response.py` | 新增`freeze_parameters/snapshot_state/rollout_primitive/rollout_sequence`；旧入口保持原行为 |
| `scripts/trajectory_collision.py` | 同一冻结快照下的静态/动态/UNKNOWN、完整机体、道路/高度/floor、速度及加速度检查 |
| `scripts/st_lattice.py` | 有时间/扩展预算的Weighted A*，物理响应边、控制记忆去重、安全短轨迹及完整制动认证 |
| `scripts/dynamic_tracker.py` | 新增批量CV中心/不确定度预测，与原`predict_at`完整协方差数值一致；不改跟踪/关联门槛 |
| `config/spacetime_planner.yaml` | 离线搜索与共享检查参数；100ms仅是计算预算，尚非经验证的10Hz执行能力 |
| `tools/spacetime_snapshot.py` | 完整JSON/NPZ冻结快照归档/读取，不补造缺失自由空间或响应历史 |
| `tools/replay_spacetime_navigation.py` | 单快照重放、计划/指令序列/完整制动NPZ、原因与输入SHA256、交互HTML |
| `tools/spacetime_scenarios.py`、`validate_spacetime.py` | 显式理想FREE地图、受控CV障碍、只执行已认证首段的闭环数值验证及交互记录 |

新模块及配置已加入Catkin安装。修改前[源包备份与manifest](../../experiments/spacetime_mvp_refactor_20261007/)保存了已有58个源文件；本批源包64文件，runtime仍为原53文件。没有物理删除旧backend，也没有重新堆叠side-shift/retreat/recovery逻辑。

## 2. 响应与时间契约

坐标全部世界NED，速度和命令的Z均向下；本批没有ROS指令转换或发布。`ResponseParameters`冻结旧模型的tau、物理coupling情景、slew、补偿、误差限额与反馈周期；模型key包含参数及序列积分版本。保留当前使用的80/160/400ms反馈情景，可生成九个独立情景。

每个`ResponseState`保留position、velocity、nominal memory、applied command、last/next feedback clock，以及effective drive和next physics clock。等待通过原`braking_target/slew/compensate/commit`记忆规则真实减速；侧向和高度动作使用冻结目标反馈策略，下一段继续使用各情景惯性，不能把同一五元组下不同速度/历史直接合并。

primitive边界不强制更新指令，也不重置反馈或物理积分阶段。同一指令的一段0.5s与两段0.25s终点在1e-10精度内一致。反应延迟只在搜索前积分一次：有已施加指令时按其驱动，没有历史时按旧模型coast假设；之后持续一阶执行器响应。

新序列走Python参考实现，ABI12/native旧包络保持原样。固定驱动片段已经与旧Python/native数值对照，尚未新增native序列接口。**旧包络中“coast与已施加驱动同时并集”的延迟额外分支没有自动扩展到新序列。** 新接口的延迟采用其冻结状态指定的假设；Phase 6接真实命令契约时须统一这个不确定性范围，不能把单驱动预测声称为旧并集的等价替代。

所有轨迹样本保存绝对时间。障碍预测直接查询这个时刻，相对于track快照时间前推，禁止另加一次cloud age。地图时效以PlanningSnapshot的当前pose时间检查，不能用未来轨迹样本时刻把当前FREE过期，也不能把旧map的evaluated_at冒充新时刻。只有地图owner在新时刻生成快照后才可重新认证。

## 3. 共享碰撞检查与搜索

静态层查询完整扫掠AABB，使用summed-volume index；随机查询与原OccupancySnapshot完整盒查询一致。未照射、过期或窗口外空间硬拒绝。动态层逐情景检查相对运动线段与膨胀CV bbox相交，能发现两端不相交而车辆中间快速横穿的冲突；协方差膨胀随预测时间增长，暂定、coasting和静止track也全部检查。缺失动态归属对应track、错误/非有限时钟或非法协方差均拒绝。

primitive物理积分段内速度采用指数响应，记录段内最大加速度并为中心曲线的弦偏差增加`a*dt²/8`覆盖。默认机体半尺寸0.25m和margin 1m，在各轴总共应用原1.25m障碍净空一次；这些半尺寸是离线假设，未作为实测无人机尺寸。道路/floor检查使用机体角点及曲线余量；未提供地面测量时floor为无穷，仍保留道路/高度先验，不能宣称真实地面已建图。

冲突记录reason、区段/相交时刻、位置、情景和track_id。静态/UNKNOWN/道路结果标记的是被拒扫掠区段的起始时刻；动态结果是模型膨胀盒的首次相交时刻。它们不是实测碰撞，且无冲突时不虚构“最小间距”。当前HTML绘制物理CV bbox和轨迹中心线；完整不确定度/机体膨胀在checker中，后续ROS规划Marker另随Phase 6接入。

搜索展示状态为(s,y,z,v,t)，实际label另外保留全部情景位置/XYZ速度、名义/已施加/effective指令和控制/积分阶段。默认dt=0.25s、4s时间窗口、前向不超过25m、速度档0/2/4/6/8m/s。有限横向目标包含相邻0.5m目标及道路内侧目标；允许零纵向速度的横向/垂向运动，它们也是提前认证的搜索动作。WAIT不会把非零速度清零。

代价初版包括时间、中心/高度偏移、速度与指令变化；启发式结合slew/lag和动态到达时隙。它是估计，不保证最优或有界次优；离散label剪枝也可能舍弃可行路线。`NO_PATH`只表示本次有限格点/窗口的结果，超时单独报告`SEARCH_TIMEOUT`。

**只有首个primitive可执行**：`valid_until`通常start+0.25s，含显式反应延迟时相应后移。即便搜索轨迹更长，也不得把整条轨迹或第一条速度一直执行下去。首段必须连同完整反馈制动尾段认证；尾段超过4s仍继续模拟，不把窗口末尾当成已经停车。预算过期时只交付此前完成认证的短轨迹，没有认证结果就返回空。预算为软计算截止，单次rollout/check及最后组合检查会有数毫秒余量；实时隔离和看门狗仍待实现。

`WAIT_FOR_DYNAMIC`须有认证等待/制动及明确动态阻塞依据。受控直线路段被移动盒覆盖整个道路截面且在本次窗口内仍覆盖时，可提前选择已认证等待，记录blocking_track_id；不把它称为任意曲线道路的无路证明。静态/未知阻塞不会自动改叫动态等待。

## 4. 验证与复测记录

最终证据：[spacetime_mvp_refactor_20261007](../../experiments/spacetime_mvp_refactor_20261007/)。

| 验证 | 结果 |
| --- | --- |
| [宿主](../../experiments/spacetime_mvp_refactor_20261007/host_final_tests.log) / [ROS](../../experiments/spacetime_mvp_refactor_20261007/ros_final_tests.log) | 各426项通过，新增51项；ROS使用隔离构建的ABI12 library |
| [Catkin构建](../../experiments/spacetime_mvp_refactor_20261007/catkin_build_install.log) / [最终安装](../../experiments/spacetime_mvp_refactor_20261007/catkin_final_install.log) | 成功，未安装到runtime工作区 |
| [安装包smoke](../../experiments/spacetime_mvp_refactor_20261007/installed_smoke.log) | 已安装模块导入、九情景计划及完整停止认证通过 |
| 响应 | prepare/commit记忆对照、Python/native固定驱动、一阶弦偏差、反馈阶段跨动作保留、零纵向侧移/升降、真实等待、序列切分不变、model mismatch与未停稳截断拒绝 |
| checker | 静态/UNKNOWN段内穿越、完整机体、快速横穿、绝对时间/观测龄、协方差增长、暂定/静止/coasting、多情景单独冲突、旧计划被速度突变推翻（Case E） |
| 搜索 | 已知窄通道、动态堵塞等待后恢复、不同惯性/历史/控制阶段label分离、预算与无认证拒绝、超过搜索窗口的完整停止、制动高度参考保持 |
| 冻结重放 | 完整响应历史/covariance/无限误差限额无损归档；缺失历史和参数hash变更明确拒绝；输出可审查计划及制动序列 |

既有几何worker EOF故障注入仍在ROS日志输出预期EOFError，末尾为OK。

最终五个理想观测闭环场景，[汇总](../../experiments/spacetime_mvp_refactor_20261007/verified_height_handoff/summary.json)与[交互预览](../../experiments/spacetime_mvp_refactor_20261007/verified_height_handoff/preview.html)。每步只执行已认证的0.25s首段，三个物理响应情景中取中间情景作为观察真值，再刷新理想观测；每步首段和完整制动均重复共享检查，未返回认证失败计划。

| 场景 | 结果 | 模拟耗时 / 步数 | WAIT首段 |
| --- | --- | --- | --- |
| A 静态车辆 | 通过x=8.1m测试目标；预先横向规划，最大侧向约1.80m | 7.0s / 28 | 6 |
| B 横穿车辆 | 等待/减速后从车后通过 | 4.0s / 16 | 5 |
| C 两车反向 | 按先后时隙通过两车 | 5.25s / 21 | 7 |
| D 暂时完全堵塞 | WAIT_FOR_DYNAMIC后恢复前进 | 8.5s / 34 | 20 |
| F 窄通道 | 保持通道内推进 | 2.25s / 9 | 0 |

B/C/D关闭横向替代动作以隔离时隙选择；五个场景均关闭高度动作，侧移/升降由单独响应/动作测试覆盖。理想地图完全FREE，障碍为受控CV对象，使用零协方差膨胀系数来隔离搜索结构；这不是实际tracker默认参数、更不是LiDAR遮挡覆盖。真实默认预测不确定度、UNKNOWN以及完整3D动作仍需后续联合验证。

最终108次重规划中多次达到350ms预算后返回已认证短轨迹；每例P95约354–357ms，没有达到5–10Hz目标。[100ms默认预算审计](../../experiments/spacetime_mvp_refactor_20261007/default_budget_audit.json)另记录同一冻结输入上的计算/首段可用率，不证明连续执行周期。所有模拟时间与实际CPU耗时分开；真实Planner延迟对发布及制动的影响没有在本批测试中被消除。

中间[连续复测失败](../../experiments/spacetime_mvp_refactor_20261007/verified_oracle/summary.json)完整保留：D/F在高速度后不能认证新制动。原因是制动高度参考取预测终点Z，逐轮追随coupling偏差。修复保留所选primitive原高度目标，新增回归测试后用新目录复测五例通过；没有放宽道路或障碍净空来掩盖问题。另一项状态修复保留effective drive/physics clock，使primitive切分不再改变耦合积分。

## 5. 复现

从工作区根目录执行；新目录须不存在。

```bash
python3 -m unittest discover -s repo/tools/tests
python3 repo/tools/validate_spacetime.py \
  --out experiments/新的时空理想验证目录 --budget .35
python3 repo/tools/replay_spacetime_navigation.py \
  --snapshot experiments/spacetime_mvp_refactor_20261007/verified_height_handoff/B_crossing_vehicle/initial.json \
  --config experiments/spacetime_mvp_refactor_20261007/verified_height_handoff/B_crossing_vehicle/planner.yaml \
  --out experiments/新的冻结计划重放目录
```

每例`planner.yaml`保存该例搜索/碰撞参数；不传config时使用源码完整3D默认参数，结果可能不同。冻结快照格式为`rmua_spacetime_snapshot_v2`，保存完整地图/路线/track covariance及响应指令/反馈/物理阶段；旧点云不能用假定零速度或猜测命令历史转换成这种快照。重放工具不发布飞行指令。

## 6. 上线前未完成工作

以下为Phase 4/5完成时的后续要求；实现及当前未验收项以[执行层收尾](30_spacetime_execution_implementation.md)为准。

当时下一批Phase 6接`spacetime_navigation/trajectory_tracker/trajectory_executor`、纯Guard入口与独立唯一发布者，明确实际控制周期、计划/参数版本、剩余前缀复核、条件备用制动和reset/切段作废。须核实VelCmd桥接持续时间并注入0.2/0.6/1.2/2.2s规划延迟、挂起/退出、观测失联与新障碍；规划失效不能继续持有旧前进速度。

随后做真实连续感知/响应/地图联合测试与CPU优化，保证在实际规划延迟下仍可按时制动，再做有界飞行及完整车辆段A/B和重复验证。当前没有实机/模拟器命令发布证据，不能继承理想场景的成功率；默认切换、真实时序、native序列优化、轨迹commitment和完整赛事验收均未完成。
