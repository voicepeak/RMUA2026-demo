# Rolling Occupancy 实现与验证（2026-10-07）

本批完成已确认[设计](26_dynamic_avoidance_design_review.md)的Phase 3。局部地图、同帧LiDAR原点、动态回波归属、重放与可视化已实现；尚未接入新的飞行Planner/Guard，实际运行副本仍为候选57，未启动新比赛。

## 1. 修改范围

| 文件 | 实际职责 |
| --- | --- |
| `scripts/local_occupancy.py` | 世界对齐滚动体素、射线更新、三态查询、完整AABB查询、只读快照与时效 |
| `config/local_occupancy.yaml` | 分辨率、路线窗口、证据TTL、射线/体素预算与动态归属门槛 |
| `scripts/lidar_points.py` | 支持PointCloud2行填充及大小端的解码；只读`LidarFrame`保存原始世界点、曝光时间、真实传感器原点和epoch |
| `scripts/route_follower.py` | 用LiDAR时间插值位姿，按原有-0.05m安装偏移同步保存origin；reset/teleport清除frame；debug记录原始单帧及时间/epoch |
| `scripts/planning_visualization.py` | 有界FREE/静态OCCUPIED/UNKNOWN体素Marker，保留原有动态track预测显示 |
| `tools/replay_local_occupancy.py` | 严格读取JSON/NPZ曝光契约，调用Tracker及地图，输出JSONL、来源SHA256、耗时及交互HTML |
| `tools/validate_local_occupancy.py` | 移动观测者、横穿车辆、墙壁/地面遮挡的最近命中射线生成与真实模块重放 |

`CMakeLists.txt`安装新模块及配置。旧飞行路径仍使用原来的`lidar_points`；新的`LidarFrame.points`保存单帧，不混入旧模式的历史点云拼接。源码以[修改前备份](../../experiments/dynamic_occupancy_refactor_20261007/before_phase3/)为比较基准，避免把已有未提交改动计入本批。

## 2. 地图与证据契约

默认resolution=0.5m、路线前方25m/后方2m、横向±4m、相对垂向-3到3m。路线方向只改变保留窗口；体素始终世界对齐，移动/旋转窗口复制重叠证据并淘汰范围外数据。窗口重新回来不会恢复已淘汰的FREE。单张地图最多100万体素。

射线起点是点云曝光时刻的`position + rotation @ [0,0,-0.05]`，与点云原有变换完全同源。使用精确体素DDA，只把有正长度穿越的格子置FREE；仅触碰角/边不清空旁格。全部当前frame hit优先，射线在任一当前hit体素处终止；命中后、未照射区域和窗口外仍为UNKNOWN。每frame最多8192根清空射线，但保留全部有效endpoint占据，预算不足只减少FREE证据。

FREE、静态hit、动态hit分别存储观测时间，TTL默认0.5/1.0/0.75s。新ray穿过旧障碍可清除它；障碍消失、TTL到期或track标签改变本身不会制造FREE。空/全无效frame以及最新frame超过free_ttl使整个快照无效并返回UNKNOWN。时效以**规划/执行当前时刻**评估，未来轨迹时刻应只用于动态预测；不能把未来样本时间作为当前地图年龄。

`owned_dynamic_returns`仅分离当前已确认、运动一致、未截断、置信度及速度足够的track回波；其他回波继续作为静态hit。多个track争同一点回退为静态，同voxel静态/动态混合保留静态；多个动态ID同voxel显式记录多归属。动态层消失后，静态层只有真实且未过期的ray证据才能FREE，否则UNKNOWN。**所有track（包括暂定、coasting、低速）仍须由后续轨迹checker检查**；静态分离不是忽略障碍的许可。

快照复制并锁定数组，支持点及完整机体AABB查询；中心点FREE不足以通过机体查询。时钟回退、来源epoch改变清空旧证据；重复frame不重复累计。非法输入/几何在reset之前拒绝。过期、无效、超窗口的结果均不会默认放行。

## 3. 验证结果及边界

证据目录：[dynamic_occupancy_refactor_20261007](../../experiments/dynamic_occupancy_refactor_20261007/)。

| 验证 | 结果 |
| --- | --- |
| [宿主测试](../../experiments/dynamic_occupancy_refactor_20261007/host_final_tests.log) / [ROS测试](../../experiments/dynamic_occupancy_refactor_20261007/ros_final_tests.log) | 各375项通过；新增35项覆盖地图、同帧转换与重放 |
| 几何与证据 | 遮挡、当前hit优先、正负轴/角穿越、窗口外sensor裁剪、滚动/旋转、TTL、动态/混合归属、完整机体与UNKNOWN拒绝；随机射线与独立逐格线段-AABB相交oracle一致 |
| [隔离Catkin构建/安装](../../experiments/dynamic_occupancy_refactor_20261007/catkin_build_install.log) | 成功；native ABI12保持原样，未写入runtime |
| [安装包/ROS消息](../../experiments/dynamic_occupancy_refactor_20261007/installed_marker_smoke.log) | 导入安装后模块/配置成功；真实CUBE_LIST MarkerArray序列化与反序列化成功 |
| [连续最近命中场景](../../experiments/dynamic_occupancy_refactor_20261007/verified_scene/validation.json) | 51帧×5487有效回波、51张有效地图、最大12155体素；占据更新及snapshot P50约19.84ms/P95约20.52ms/max约21.12ms |
| [有界射线性能](../../experiments/dynamic_occupancy_refactor_20261007/ray_budget_benchmark.json) | 8192/20000/75000回波各20轮；P95约29.43/30.34/36.22ms，最多8192射线，所有endpoint仍OCCUPIED |
| [旧档案拒绝](../../experiments/dynamic_occupancy_refactor_20261007/legacy_archive_rejection.json) | 第58轮档案缺真实曝光origin，被明确拒绝；未从后来的飞机位置补造FREE |
| [源码/runtime清单](../../experiments/dynamic_occupancy_refactor_20261007/phase3_manifest.json) | 源码6个新增/变更文件，runtime与修改前SHA256逐项一致 |

合成连续场景29帧具有可信动态回波归属，并非51帧全程跟踪成功。末帧车辆track为COASTING、age约0.3s；车辆当前回波保守回退静态，未借修改确认门槛掩盖关联限制。最后样本静态1180/动态0/FREE2327/UNKNOWN8648体素。合成最近命中几何验证地图更新，不证明实际车辆ID或实飞效果。

性能测量是宿主机地图更新加快照，未包含聚类、时空搜索、最终Guard及控制发布，不能据此宣称整体规划/执行周期达标。既有几何worker EOF故障注入仍打印预期EOFError，两套测试最终均OK。

可直接打开[连续场景交互预览](../../experiments/dynamic_occupancy_refactor_20261007/verified_scene/replay/preview.html)：时间滑块、XY/XZ、UNKNOWN开关、动态当前位置与1s/2s预测。HTML脚本通过`node --check`；显示抽样不影响地图查询。ROS显示显式NED→ENU(y,x,-z)，不把默认Z向上混入算法坐标。

本批FREE是离散射线观测证据；体素分辨率、遮挡及传感器覆盖仍有限，不能称连续空间安全证明。下一批使用完整机体膨胀、UNKNOWN硬约束与动态预测复核，而非只看中心点。

## 4. 复现

从工作区根目录执行；新的输出目录必须未存在。

```bash
python3 -m unittest discover -s repo/tools/tests
python3 repo/tools/validate_local_occupancy.py --out experiments/新的地图验证目录
python3 repo/tools/replay_local_occupancy.py \
  --cloud-dir experiments/dynamic_occupancy_refactor_20261007/verified_scene/clouds \
  --out experiments/新的地图重放目录
```

重放要求JSON/NPZ中真实`sensor_origin`与`cloud_stamp`，优先读取`frame_points`；两份时间/origin/epoch冲突立即拒绝。旧档案不能仅凭世界hit point恢复自由空间。

已source ROS时，加`--publish-ros --rate 10`只发布感知诊断，不发布飞行指令：`/rmua/perception/occupancy_debug`、`/rmua/perception/occupancy_markers`、`/rmua/perception/dynamic_tracks_markers`。RViz Fixed Frame为`rmua_tracker_preview_enu`。

## 5. 后续实施

后续Phase 4离线MVP及其必需的Phase 5纯响应序列已实现，见[时空Planner实现记录](29_spacetime_mvp_implementation.md)。本页375项测试与源码/runtime清单为Phase 3交付历史，最新状态见当前交接。后续Phase 6飞行入口、独立执行器与共享Guard也已实现，见[执行层收尾](30_spacetime_execution_implementation.md)；当前实跑门槛尚未验收。等待必须通过实际响应减速，搜索节点保留侧向/垂向惯性及命令记忆；可执行前缀与完整停止尾段均需认证。之后再接独立唯一发布者及Guard，不新增side-shift/retreat/recovery补丁，也不切默认飞行模式。
