# LiDAR 到静态规划的只读链路（迭代 44）

2026-10-27。复核[官方规则中心](https://bbs.robomaster.com/wiki/20204847/809873)仍列出 RMUA 2026 规则 V1.0.0；重新下载比赛规则及参赛手册，两份 PDF 的 SHA-256 与 `configs/official_rules_baseline.json` 一致。核对[官方模拟器说明](https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/blob/RMUA2026-01/README.md)。不修改规则、模拟器或比赛控制配置。

新增 `evaluation.pipeline.lidar_static.LidarStaticProbe`，将新鲜历史状态、扫描时刻历史位姿、世界坐标端点图和静态候选规划串接。只接受历史缓冲中最新状态，扫描和状态必须在同一 epoch 内且有效，历史姿态不可外推，过期/重复/容量溢出拒绝并保留原因。接口返回诊断原因和候选检查结果，不返回轨迹或控制权限；调用方须将状态缓冲更新与探针调用串行化。

`evaluation.pipeline.replay_lidar_static` 用合法采集的 LiDAR 和 IMU/GPS 估计状态，按记录的到达时间回放。两场各 6 帧，全部得到 `NO_STATIC_CANDIDATE`：6 帧地图有效期不足以覆盖模型停车时域，6 帧运动学界不满足。这里使用极短的 0.05 秒诊断候选和假设机体尺寸，只用于连通性检查；这些拒绝不证明真实飞行障碍或制动能力。位姿仍未标定，图只含占用、不提供 FREE，不可用于运动控制。

首个宿主回放失败：宿主 Python 未安装 `rospy`；随后在 Noetic 容器成功运行，失败保留于 [证据](evidence/p2_lidar_static_probe.json)。专项五项测试在宿主和 Noetic 通过；宿主完整 194 项通过，Noetic 191 项通过、3 项跳过。回放未启动模拟器、未发布控制、未使用真值。没有证明在线端到端时效，P1/P2 和穿门验收仍未完成。

```bash
OPENBLAS_NUM_THREADS=1 ./tools/enter_env.sh python3 -m evaluation.pipeline.replay_lidar_static runs/20261023T065455Z_abd6fdcb40e8464ab03367bc3e81d3fe
```
