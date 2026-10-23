本模块按 docs/refactor_design_v1.0.md 规划。验收状态与依赖见 docs/implementation_status.md。
# 历史位姿

history.py 保留最多 1s/256 条正式 IMU/GPS ESKF 状态，供延迟图像查询对应测量时刻。要求同 epoch、新鲜且健康的当前源、健康且不超过 30ms 的历史括号，不外推或延长原状态有效期；插值跳变检查不代表已标定误差上界。返回独立的 HistoricalPose，不能替代当前控制 StateEstimate。

FlightSensors 默认不保留历史，独立只读节点可传入 PoseHistory；时钟回退清空缓存并维持故障锁存。真实图像头关联和重放证据见 docs/historical_pose_observation.md。
