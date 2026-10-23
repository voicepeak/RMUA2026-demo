# 在线 LiDAR 端点占据（迭代 28，2026-10-23）

按设计第 6.1 节和 Sprint C 推进正式传感器处理，P1 未通过事项继续保留。本轮先核对[官方 README](https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/blob/RMUA2026-01/README.md) 和 [settings.json](https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/blob/RMUA2026-01/settings.json)：LiDAR 为 32 线、10Hz、30m、SensorLocalFrame。原模拟器/配置保持不变。此处的单帧端点证据不代表三维感知阶段验收。

实现有界 PointCloud2 解码与单帧端点体素化。严格检查布局、字节长度、XYZ 浮点类型与 frame，支持字段重排、行/点填充、大小端、float32/float64。NaN/Inf 即使 is_dense=true 也被丢弃。数组由不可写 bytes 支撑。快照只接受同 epoch、有效时间内的扫描，负坐标用 floor；缺失、过期、未来或其他 epoch 查询为 UNKNOWN。

传感器 frame 保留为 lidar，不做未经标定的世界转换或逐点时间补偿。0.25m 体素内有有效端点则记 OCCUPIED，其余 UNKNOWN；不使用缺失回波、零距离或最大量程返回生成自由空间，不将地面/近场回波自动剔除。每帧重新构建，避免把运动传感器坐标下的旧点直接累计。该快照表达扫描时刻的固定坐标，不能直接用于当前机体净空判定。

独立只读节点只保留最新消息，回调不执行体素化或磁盘写入；处理位于自己的循环与进程。等待流超过 0.5s 后失败，首次 5s 无有效流也失败；这些是本地观察器的保护，不是飞行失联策略。记录每帧处理时间和接收时刻，原始首帧载荷/字段描述保留供独立重放。ROS 图审计要求唯一业务订阅为 lidar，唯一基础设施发布为 rosout，没有控制发布或 debug 输入。

```bash
OPENBLAS_NUM_THREADS=1 python3 -m unittest tests.unit.test_lidar_occupancy -v
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.lidar_observation --seed 2102 --duration 10
```

启动器先保存精确源码归档，再启动全新未插桩场次，观察结束精确关闭自有模拟器/ROS。离线审计在 Noetic 内用 sensor_msgs.point_cloud2.read_points 独立解码保存的首帧，与新解码器逐点比较，并用 Python math.floor 独立检查首帧端点体素数。最后快照检查无重复体素、全为 OCCUPIED、时间/epoch/frame 与末帧一致。

| seed | 10s 处理帧数 | 首帧有效点 | 首帧体素 | 处理耗时 P99 / 最大 | 观测处理频率 |
|---|---:|---:|---:|---:|---:|
| 2101 | 96 | 15745 | 4529 | 41.76 / 42.84ms | 9.58Hz |
| 2102 | 98 | 15881 | 4638 | 34.19 / 39.46ms | 9.84Hz |

两场独立解码审计通过、无接收/处理事件或节点故障，业务订阅均只有 LiDAR。处理耗时不包含完整传感器生成/传输链，也不是运动条件下时延认证；这些只读地面场次没有飞行输入、未检验运动补偿、门或细线漏检。

宿主 85 项测试通过；Noetic 82 项通过、3 项需宿主 Git 的测试跳过。首次 Noetic 测试发现 NumPy 1.17 不支持对空 (0,3) 数组执行 unique(axis=0)，已显式处理空输入后重跑完整两套测试。测试覆盖空/无效点、填充与大小端、坏布局、负坐标、未知/遮挡区、过期/未来/epoch 不符，以及下一空帧不保留旧占据。

首场运行在空数组修复及离线审计器实现之前，审计为事后执行；第二场源码归档包含这些修订。最终将流超时检查移至每轮循环入口，避免持续坏消息绕过有效流超时；seed2103 的 4s 只读冒烟处理 39 帧，独立解码和清理审计通过。三场都运行前后源码及原环境不变，三个归档逐文件独立重核，不构成冻结性能对照。机器可读证据见 docs/evidence/p2_lidar_endpoints.json。后续仍需同步双目/IMU、外参/时间标定、运动补偿、保守世界占据与可信 FREE 证据；本模块尚未接入控制或任务规划，P1/P2 均未验收。
