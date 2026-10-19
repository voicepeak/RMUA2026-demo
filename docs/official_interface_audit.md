# 官方接口审计

P0 输入和控制话题候选记录于 configs/official_baseline.yaml；实际类型以每轮 interface_audit.json 为准。未观察到的类型保持 null，不猜测 PWM 的消息定义或数值范围。

官方依据（核对于 2026-10-19）：

- https://bbs.robomaster.com/wiki/20204847/809873?source=7 ：官方规则中心列出比赛规则手册 V1.0.0（2026-01-30）、参赛手册 V1.0.0（2026-01-15）；本轮网页可确认版本，PDF 附件未成功提取，不能声称逐页复核了原文。设计与旧版结构化条款继续作为待对照的需求。
- https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/blob/RMUA2026-01/README.md
- https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/blob/RMUA2026-01/settings.json
- https://github.com/RoboMaster/IntelligentUAVChampionshipBase/blob/RMUA2026/README.md

configs/simulator_settings.json 从上述官方 RMUA2026-01 settings.json 固定，启动时显式指定。旧版只启用了前视双目，新版使用官方前后双目配置，旧版配置保持原样。配置哈希与来源见 simulator_baseline.json。

本地模拟器 Build 来自 simulator_12.0.0.5，随附 Git checkout 仍是 RMUA2025-01。Git 文档分支不能单独证明二进制赛季；需对照运行日志、发布版本和接口观测进一步核验。启动脚本与二进制复制来源及关键 SHA256 已记录。

审计工具订阅白名单传感器，检查控制话题发布/订阅者，并只查询服务类型，不调用服务。正式服务器开放权限、电机旋转方向和首次控制锁定行为需要独立验证。

2026-10-19 第一轮实施前重新读取了官方 RMUA2026 模拟器 README、settings.json 和基础开发 README：IMU 时间戳作为全局时钟；正式服务器仅开放规则话题；容器自动启动、无 GUI，不在提交镜像入口硬编码 ROS_IP/ROS_MASTER_URI。官方给出质量 0.9kg、电机距中心 0.18m、惯量、电机系数和最大 RPM；电机系数单位、旋向和实际 PWM 映射仍需辨识。

seed1001 实测：GPS 为 geometry_msgs/PoseStamped，正常消息含模拟器时间戳；initial_pose 同类型但 header.stamp=0，按合法初始化信息使用，禁止当持续定位。IMU 静止比力约 (0,0,-9.81)，与 FRD/NED 契约相符。只读状态节点仅订阅 initial_pose、IMU 和 GPS，不使用 debug 真值。

部署审计发现官方两个仓库的 VelCmd 定义不同：基础教程含 geometry_msgs/Twist；模拟器 VelCmdmsg/VelCmd.msg 则是 Header + vx/vy/vz/yawRate/va/stop。已改为模拟器版本，与 beta 本机实跑消息一致。此前 P0 只核对了类型名称，未验证 VEL MD5，不能将旧审计解释为 VEL 通信已验收；PWM 悬停使用 RotorPWM，不受此差异影响。后续独立 VEL 场次必须验证连接和指令符号。
