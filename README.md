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
```

仅用于本地动力学辨识，会因高度/姿态/状态过期而中止，不能作为比赛控制入口。

两个版本共享容器、GPU 和 ROS master，依次运行。启动脚本发现现有 ROS/模拟器会退出，先关闭已有场次。目录迁移和环境说明见 [工作区迁移](docs/workspace_migration.md)。
