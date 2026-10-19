# P0 验证记录（2026-10-19）

- 环境：复用 rmua_noetic（Python 3.8.10、ROS Noetic、OpenCV 4.2.0），RTX 3060 Laptop 驱动 595.91.07 可访问；未重装环境。
- 纯 Python 核心单元测试 16/16 通过；所有新 shell 入口通过 bash -n。
- 本地复制的模拟器已成功启动；运行日志确认版本 12.0.0.5。
- seed456、10 秒只读审计：9 个白名单输入全部收到，时间戳回退计数全部为 0，PWM/VEL 无发布者。图像接收约 17.4 Hz、IMU 100 Hz、GPS 10 Hz、LiDAR 9.5 Hz；本机接收频率不是官方频率验收。
- ROS 时钟与传感器 stamp 的中位差约 2 秒；不能当作端到端延迟，进入控制前必须统一时钟域。
- 发现控制类型 airsim_ros/RotorPWM 和 airsim_ros/VelCmd、工厂服务类型 airsim_ros/MeterReport。PWM 物理映射、范围、失联行为、首次信号锁定及正式服务器权限尚未验证。
- 已固定官方控制消息，使用现有 catkin 工具编译独立 .runtime_ws。MeterReport 的正式 .srv 定义仍待确认，不猜测也不调用。

初次成功审计目录：runs/20261019T103731Z_ca657cfed91a4de5b48cf2837f9bb03c（本机保留，Git 忽略）。记录包括 seed、镜像 ID、Git 状态、完整前后源码 SHA256、配置、话题清单及 UNKNOWN 官方结果；该轮源码哈希前后一致。初次审计运行于未提交的新源码，具体源码以该轮哈希为准。

后续闭环验收见 implementation_status.md；本记录不代表 PWM 悬停或完整重构已完成。

已提交版本复测：seed789，10 秒，提交 940d520，Git 工作树干净；9 个白名单输入全部收到、无时间戳回退、PWM/VEL 发布者均为空，源码 SHA256 前后一致。完整本机目录为 runs/20261019T104104Z_6e494c2e44904567856ea9e50b336bea；小型接口结果与摘要保存在 docs/evidence/，原始日志与全量 manifest 不进入 Git。两个种子均仅作开发接口审计，不能作为闭环多种子比赛验收。
