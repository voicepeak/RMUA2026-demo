# 官方控制消息来源

源仓库：RoboMaster/IntelligentUAVChampionshipBase，RMUA2026 分支，固定 commit `f23cd23ec07825372baf05a6dc33f98e5bdcb7aa`。

- https://github.com/RoboMaster/IntelligentUAVChampionshipBase/blob/f23cd23ec07825372baf05a6dc33f98e5bdcb7aa/basic_dev/src/airsim_ros/msg/RotorPWM.msg
- VelCmd 已在 2026-10-19 部署审计中改为模拟器提供的定义：https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/blob/9f0afeaa857b3b1f2a52d03b9f57b2716d5765a0/VelCmdmsg/VelCmd.msg 。分支 RMUA2026-01，固定 commit `9f0afeaa857b3b1f2a52d03b9f57b2716d5765a0`。基础教程中的 Twist 版本与本赛季模拟器 vx/vy/vz/yawRate/va/stop 定义不同，不能使用类型名称相同就当作兼容。

消息字段保持原样，仅统一换行。构建文件是本工程的最小适配，未引入官方示例飞行节点。上游 LICENSE 原文随包保存。

包名必须是 airsim_ros 才能保持官方 ROS 类型名。此包仅覆盖控制消息，不是官方完整 airsim_ros 包。MeterReport 服务虽在本地可见，但本次固定的官方源码未包含其 .srv，仍需核实正式定义；不猜测服务请求/响应字段。
