# RMUA next

`main` 是按 [完整重构设计 v1.0](docs/refactor_design_v1.0.md) 建立的新工程；旧版对照代码在 `beta` 分支，本机位置为 `../beta/repo`。

当前交付为 **P0 工程与只读接口审计基线**。包含不可变数据契约、三态占据查询、制动速度包络、控制前置检查、不可覆盖实验记录和 ROS 接口探测。P1 PWM 悬停、P2 感知定位和后续比赛闭环尚未验收，当前没有发布飞行指令的入口。阶段进度见 [实施状态](docs/implementation_status.md)。

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

两个版本共享容器、GPU 和 ROS master，依次运行。启动脚本发现现有 ROS/模拟器会退出，先关闭已有场次。目录迁移和环境说明见 [工作区迁移](docs/workspace_migration.md)。
