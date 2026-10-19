# 官方控制消息来源

源仓库：RoboMaster/IntelligentUAVChampionshipBase，RMUA2026 分支，固定 commit `f23cd23ec07825372baf05a6dc33f98e5bdcb7aa`。

- https://github.com/RoboMaster/IntelligentUAVChampionshipBase/blob/f23cd23ec07825372baf05a6dc33f98e5bdcb7aa/basic_dev/src/airsim_ros/msg/RotorPWM.msg
- https://github.com/RoboMaster/IntelligentUAVChampionshipBase/blob/f23cd23ec07825372baf05a6dc33f98e5bdcb7aa/basic_dev/src/airsim_ros/msg/VelCmd.msg

消息字段保持原样，仅统一换行。构建文件是本工程的最小适配，未引入官方示例飞行节点。上游 LICENSE 原文随包保存。

包名必须是 airsim_ros 才能保持官方 ROS 类型名。此包仅覆盖控制消息，不是官方完整 airsim_ros 包。MeterReport 服务虽在本地可见，但本次固定的官方源码未包含其 .srv，仍需核实正式定义；不猜测服务请求/响应字段。
