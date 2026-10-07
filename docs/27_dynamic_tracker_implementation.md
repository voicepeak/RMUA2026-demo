# 动态避障重构：第一批实现（2026-10-07）

用户已确认[设计审查](26_dynamic_avoidance_design_review.md)。本批完成设计Phase 2的DynamicTracker、纯聚类提取、预测接口、连续重放工具及诊断可视化。后续实现不再重复请求设计确认。

**这是一批感知模块实现，不是整套避障重构完成。** Occupancy、Space-Time Planner、VelocityResponse多primitive接口、独立执行器及新Guard入口均尚未实现。新tracker未替换当前飞行Planner；运行副本仍是候选57。车辆3→5整段仍未通过。

## 1. 本批代码

| 文件（相对repo） | 改动 |
| --- | --- |
| `ros_ws/src/route_follower/scripts/lidar_scene.py` | 抽出纯函数`extract_clusters`及`LidarCluster`，保留旧Scene关联/速度/距离逻辑和稀疏ROI短时保留行为 |
| `ros_ws/src/route_follower/scripts/dynamic_tracker.py` | 稳定ID、带门限的一对一Hungarian关联、CV Kalman、Joseph协方差更新、遮挡/过期/reset、不可变预测快照 |
| `ros_ws/src/route_follower/config/dynamic_tracker.yaml` | 集中定义tracker与聚类参数；默认confirmed遮挡寿命0.75s、tentative寿命0.4s |
| `ros_ws/src/route_follower/scripts/planning_visualization.py` | 有界tracker MarkerArray，当前位置蓝、1s/2s预测含不确定度bbox橙；显示ID、速度、age |
| `tools/replay_dynamic_tracker.py` | 顺序重放原有JSON/NPZ云、记录诊断JSONL、输入/源码SHA256、预测观测残差、独立HTML时间滑块；可显式发布ROS诊断/Marker |
| `tools/record_tracker_clouds.py` | 有界只读ROS采集，同时间插值位姿、保存真实传感器原点、处理PointCloud2行步长与clock reset；不发送任何控制/重置请求 |
| `tools/validate_dynamic_tracker.py` | 已知解析运动真值的可复现连续场景：反向横穿双目标、静态目标、移动观察者、点噪声、短遮挡 |
| `tools/tests/test_dynamic_tracker.py`、`test_tracker_replay.py` | 23项新增行为测试，包含完整云→cluster→tracker→日志重放与跨epoch处理 |
| `ros_ws/src/route_follower/CMakeLists.txt`、`package.xml` | 安装新增Python模块/config；声明Marker和YAML运行依赖 |

没有改动Planner入口、Guard、响应模型、控制发布路径或标定值来接入新tracker。运行包没有同步，也没有启动新比赛。

## 2. 接口和约束

```python
tracker = DynamicTracker(TrackerConfig(...))
obstacles = tracker.update(world_points, sensor_pose_position, cloud_stamp,
                           route_forward, center_height, s)
# 也可输入extract_clusters生成的观测：
obstacles = tracker.update_clusters(clusters, cloud_stamp)
prediction = obstacles[0].predict(2.0)          # 相对该快照timestamp
prediction = obstacles[0].predict_at(stamp)    # 绝对传感器时钟时间
diagnostics = tracker.summary()
```

- 过滤状态为世界NED `[x,y,z,vx,vy,vz]`，使用真实观测间隔，不把无人机移动算到目标速度中。位置与速度协方差来自同一个6×6滤波状态。
- 关联先通过欧氏距离/Mahalanobis门限，再以创新似然及尺寸代价做一对一分配；高噪声观测不能仅因其Mahalanobis距离更小而抢占关联。
- tentative需要连续观测才能confirmed；confirmed短时未观测成为coasting，协方差随时间增长；超时在下一次关联前删除。没有观测的时段调用`update_clusters([], stamp)`推进预测/生命周期；只调用`snapshot()`不会自行推进时钟。
- bbox变化、ROI截断会降低运动归属可信度、增加观测/预测不确定度；可见bbox历史最大尺寸暂不收缩。完整车体仍不可从可见回波保证，不能把这个bbox当成独立车辆真值。
- `point_indices`是当前帧原始回波归属；coasting时为空。为后续静态/动态占据分层保留这一契约。
- `motion_confirmed`仅说明运动回波归属质量，**不能作为碰撞检查忽略其他track的过滤条件**。低速、tentative、截断与coasting目标均保留预测和不确定度。
- 同时间戳重复观测不再次滤波；时钟倒退/reset增加epoch、作废旧track，ID也不重复使用。快照复制并只读封装数组，后续滤波不会修改已交给规划器的预测输入。
- `extract_clusters`支持可选的实测`ground_height`（NED地面）；没有实测地面时不从中心线虚构地面高度。第一批默认仍用现有ROI和连通网格，不声称有车辆语义分类。

ROS预览显式将NED `(x,y,z)`转换为ENU `(y,x,-z)`，固定frame为`rmua_tracker_preview_enu`。这是显示变换，不影响跟踪或控制坐标。HTML预览分别显示原始NED XY/XZ投影。

## 3. 验证结果及证据

实验根目录：[dynamic_avoidance_refactor_20261007](../../experiments/dynamic_avoidance_refactor_20261007/)。修改前53文件备份为`before_phase2/`，确认前的基线清单/317项测试日志也保留。

| 检查 | 结果与边界 |
| --- | --- |
| 宿主全量测试 | [340项通过](../../experiments/dynamic_avoidance_refactor_20261007/host_tracker_verified_tests.log)，含原有317项与新增23项 |
| ROS容器全量测试 | [340项通过](../../experiments/dynamic_avoidance_refactor_20261007/ros_tracker_verified_tests.log)，使用隔离构建的native库；不修改实际运行工作区 |
| Catkin构建/安装 | [独立构建](../../experiments/dynamic_avoidance_refactor_20261007/tracker_catkin_build.log)及[安装](../../experiments/dynamic_avoidance_refactor_20261007/tracker_verified_install.log)成功，位置`tracker_build_ws/` |
| ROS Marker消息 | [5个Marker的真实序列化检查通过](../../experiments/dynamic_avoidance_refactor_20261007/marker_serialization.log)；未宣称已经在RViz肉眼验证 |
| 预览脚本 | 独立HTML生成，内嵌JS通过`node --check`；不需要外部CDN |
| 连续合成真值 | [61帧、147个confirmed观测](../../experiments/dynamic_avoidance_refactor_20261007/cv_verified_validation/truth_validation.json)，三个目标各一个稳定ID；点噪声σ=0.03m，两次3帧遮挡，观察者前进 |
| 合成预测误差 | 速度误差P95约0.036m/s；1s位置误差P95约0.052m、2s约0.089m；仅这个匀速合成场景，不能外推车辆真实表现 |
| 跟踪耗时 | 合成61帧P95约1.1ms；第58轮48帧P95约1.9ms。测量包含聚类/关联/滤波，不含日志、Marker、磁盘IO或Planner耗时，不代表整套系统规划频率 |
| 第58轮原始重放 | [48帧，0个confirmed帧](../../experiments/dynamic_avoidance_refactor_20261007/replay_run58_verified/summary.json)；观测间隔中位1.188s/P951.968s/最大2.868s，超过默认寿命。没有延长阈值来人为制造稳定ID，因此没有可报告的1s/2s实测预测残差 |
| 当前模拟器只读采集 | [8秒内0帧](../../experiments/dynamic_avoidance_refactor_20261007/continuous_finished58/capture_summary.txt)，Finished场景不再发送新传感器数据；没有重置/启动模拟器来掩盖这个限制 |

已有几何worker EOF故障注入测试仍会打印子进程EOFError；宿主/ROS测试末尾均为OK，不是新增跟踪异常。

两份预览：[带噪声与遮挡的连续场景](../../experiments/dynamic_avoidance_refactor_20261007/cv_verified_validation/replay/preview.html)、[第58轮历史点云](../../experiments/dynamic_avoidance_refactor_20261007/replay_run58_verified/preview.html)。

## 4. 复现

从工作区根目录执行；输出目录须不存在，避免覆盖证据。

```bash
python3 -m unittest discover -s repo/tools/tests
python3 repo/tools/validate_dynamic_tracker.py --out experiments/新的连续验证目录
python3 repo/tools/replay_dynamic_tracker.py \
  --cloud-dir experiments/race_car_response_20261004_58/mission/leg_3_5/clouds \
  --out experiments/新的历史重放目录
```

在已source ROS的容器中，重放命令加`--publish-ros --rate 10`可发布：

- `/rmua/perception/dynamic_tracks`：每frame的IDs、当前速度、1s/2s预测、协方差和关联统计。
- `/rmua/perception/dynamic_tracks_markers`：MarkerArray；RViz Fixed Frame设置为`rmua_tracker_preview_enu`。

只读连续采集需当前同赛段参考及正在更新的传感器：

```bash
python3 /workspace/repo/tools/record_tracker_clouds.py \
  --reference /workspace/experiments/同赛段已有点云/某帧.json \
  --out /workspace/experiments/新的连续采集目录 --duration 8 --rate 10
```

采集器的参考来自指定历史frame，不随当前路线重建而自动更新；阶段改变后必须换对应参考。它用于有界诊断，不是最终在线感知调度层。

## 5. 后续实施

后续Phase 3已完成，见[Rolling Occupancy实现记录](28_local_occupancy_implementation.md)：同帧sensor origin、ray casting、遮挡、hit优先、过期和动态点云归属。下一批按已确认设计实现时空搜索及响应/Guard对齐。本页340项测试与runtime状态为Phase 2交付时的历史结果，最新汇总见当前交接。

Tracker还需新的连续车辆观测验证ID与速度方向、1s/2s预测残差，以及目标合并/遮挡下的误关联情况。仅几何观测无法保证完全重叠目标的身份，此类track必须保守处理，不能以合成测试替代实际验证。旧模式继续作为回归/A/B基线；默认切换仍须时序故障注入和重复完整车辆段验收。
