# 官方控制消息来源

源仓库：RoboMaster/IntelligentUAVChampionshipBase，RMUA2026 分支，固定 commit `f23cd23ec07825372baf05a6dc33f98e5bdcb7aa`。

- https://github.com/RoboMaster/IntelligentUAVChampionshipBase/blob/f23cd23ec07825372baf05a6dc33f98e5bdcb7aa/basic_dev/src/airsim_ros/msg/RotorPWM.msg
- VelCmd 已在 2026-10-19 部署审计中改为模拟器提供的定义：https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/blob/9f0afeaa857b3b1f2a52d03b9f57b2716d5765a0/VelCmdmsg/VelCmd.msg 。分支 RMUA2026-01，固定 commit `9f0afeaa857b3b1f2a52d03b9f57b2716d5765a0`。基础教程中的 Twist 版本与本赛季模拟器 vx/vy/vz/yawRate/va/stop 定义不同，不能使用类型名称相同就当作兼容。

消息字段保持原样，仅统一换行。构建文件是本工程的最小适配，未引入官方示例飞行节点。上游 LICENSE 原文随包保存。

包名必须是 airsim_ros 才能保持官方 ROS 类型名。此包仅覆盖已核实的接口，不是官方完整 airsim_ros 包。

MeterReport 的固定公开源码未附 .srv。本地 12.0.0.5 插件的只读序列化检查显示请求为两个有符号 int64，响应为 bool；`int64 index / int64 value --- bool success` 的 ROS MD5 为 `40e1aa17dd96ff6a8c88396581d6a491`，与运行服务器的 TCPROS 探测握手完全一致。插件 SHA256 与证据见 `configs/simulator_baseline.json` 和 `docs/evidence/p0_meter_report_contract.json`。这份 .srv 是经本机协议验证的兼容定义，不冒充上游发布文件。测试只验证线格式，握手没有发送业务请求；正式权限、success 的业务语义与重试行为仍未验证。不得用测试数值调用比赛上报服务。
