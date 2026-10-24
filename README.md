# RMUA next

`main` 是按 [完整重构设计 v1.0](docs/refactor_design_v1.0.md) 建立的新工程；旧版对照代码在 `beta` 分支，本机位置为 `../beta/repo`。

当前交付为 **P0 工程、状态观测与有界 PWM 辨识**。包含不可变数据契约、三态占据查询、制动速度包络、控制前置检查、不可覆盖实验记录和 ROS 接口探测。P1 PWM 悬停、P2 感知定位和后续比赛闭环尚未验收，比赛配置保持关闭。阶段进度见 [实施状态](docs/implementation_status.md)。

本机继续使用 `rmua_noetic` 容器，无须重新安装依赖。`simulator/` 是旧模拟器的独立本地副本，不进入 Git。源码与小型配置通过分支同步；模拟器二进制、模型、原始数据和日志保留在本机。

```bash
cd /home/tianbot/RUMA-by-helinjun/main
./tools/build_interfaces.sh
./tools/enter_env.sh python3 -m unittest discover -s tests/unit -v
# 终端一：仅启动模拟器，不发布控制
./tools/run_sim.sh 456 offscreen
# 终端二：采样只读接口；每次创建独立实验目录
./tools/audit.sh --seed 456 --duration 10
# 审计完成后关闭本场
./tools/stop_sim.sh
```

新增只读状态观测（在已启动的对应种子场景内运行）：

```bash
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.state_observation --seed 1001 --duration 8
```

状态链包含 IMU/GPS ESKF、零 GPS 拒绝和 reset epoch；噪声参数尚未标定，不能据此宣称 P2 已验收。

学习双目已完成 [C-Fast-FoundationStereo / SGBM 离线对照](docs/learned_stereo_comparison.md) 和 [只读在线时效实验](docs/learned_stereo_live_latency.md)，有独立 GPU 环境、版本锁与原始输入追踪；在线结果全部过期，尚未接入深度融合或控制。

[TensorRT 候选](docs/stereo_tensorrt_candidate.md) 已实际构建并对照，仍存在数值偏差，未替换默认后端。

有界 PWM 辨识入口（自动启动新场次，结束时关闭模拟器；须先停止已有场次）：

```bash
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 1002
# 已辨识方向的逐电机小脉冲
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 1003 --motor-pulses --pulse-seconds .1
# 低高度、有限时长的 PWM 闭环；不是比赛入口
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 1004 --hover-seconds 10
# 独立进程记录本地真值，飞行结束后才计算误差；不输入控制器
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 1005 --hover-seconds 14 --evaluate-truth
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 1006 --hover-seconds 14 --step --evaluate-truth
# VEL 必须是另一个全新场次；租约与 ROS 图检查禁止同场混发
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 1007 --mode VEL --hover-seconds 14 --step --evaluate-truth
# 显式本地故障注入；不会修改 ROS 原始话题
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 1008 --hover-seconds 12 --sensor-fault GPS_ZERO --fault-seconds .15 --evaluate-truth
# 局部悬停的短时 GPS 桥接；不等于工厂无 GPS 导航
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 1014 --hover-seconds 14 --sensor-fault GPS_ZERO --fault-seconds .8 --gps-bridge --evaluate-truth
# 小幅偏航、角速度单位与响应验证（两个 mode 分别运行）
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 1015 --mode PWM --hover-seconds 14 --yaw-motion --evaluate-truth
# 低速制动响应与正式传感器场景快照；不会自动认证制动下界
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 1016 --mode PWM --hover-seconds 14 --braking --snapshot --evaluate-truth
```

仅用于本地动力学辨识，会因高度/姿态/状态过期而中止，不能作为比赛控制入口。

冻结参数的开发复现批次：

```bash
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.p1_regression
```

批次记录每场结果与完整源码哈希，禁止运行途中调参；这是开发复现，不是留出集或阶段验收。

两个版本共享容器、GPU 和 ROS master，依次运行。启动脚本发现现有 ROS/模拟器会退出，先关闭已有场次。目录迁移和环境说明见 [工作区迁移](docs/workspace_migration.md)。

另有固定基础 digest、无需 beta 挂载的独立开发镜像：`bash tools/build_image.sh`，部署范围和限制见 [独立部署说明](deployment/README.md)。

双目转换最新进展见 [Fourier 导出偏差定位与验证](docs/stereo_conversion_diagnosis.md)：同精度 FP32 对照已恢复，旧 AMP 门限和在线时效仍待通过。

后续 [同图 FP16 对照](docs/stereo_frozen_fp16.md) 记录了 12 组中的两组失败；离线约 122ms 不代表在线默认有效期通过。

[混合精度约束实验](docs/stereo_mixed_precision.md)已验证 92 层 FP32 约束候选，仍有一组有效掩码失败，继续定位几何坐标精度。

[几何采样精度核验](docs/stereo_geometry_precision.md)修正并验证了可见坐标格式，但 12 组数值对照仍有两组失败，候选保持离线用途。

[静态轨迹核验](docs/static_swept_volume.md)已连接轨迹与三态占据契约，检查连续机体扫掠体积、未知空间和体素有效期；仍需可信地图及其余安全约束才能用于自主飞行。

[局部静态规划候选](docs/static_local_planner.md)已实现五次参考、连续运动学界、绕行候选与短租约选择，并在合成地图中验证；真实地图与飞行接入仍待验收。
