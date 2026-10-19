# 官方接口审计

P0 输入和控制话题候选记录于 configs/official_baseline.yaml；实际类型以每轮 interface_audit.json 为准。未观察到的类型保持 null，不猜测 PWM 的消息定义或数值范围。

官方依据（核对于 2026-10-19）：

- https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/blob/RMUA2026-01/README.md
- https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/blob/RMUA2026-01/settings.json
- https://github.com/RoboMaster/IntelligentUAVChampionshipBase/blob/RMUA2026/README.md

configs/simulator_settings.json 从上述官方 RMUA2026-01 settings.json 固定，启动时显式指定。旧版只启用了前视双目，新版使用官方前后双目配置，旧版配置保持原样。配置哈希与来源见 simulator_baseline.json。

本地模拟器 Build 来自 simulator_12.0.0.5，随附 Git checkout 仍是 RMUA2025-01。Git 文档分支不能单独证明二进制赛季；需对照运行日志、发布版本和接口观测进一步核验。启动脚本与二进制复制来源及关键 SHA256 已记录。

审计工具订阅白名单传感器，检查控制话题发布/订阅者，并只查询服务类型，不调用服务。正式服务器开放权限、电机旋转方向和首次控制锁定行为需要独立验证。
