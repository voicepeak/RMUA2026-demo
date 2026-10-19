# 架构

目录遵循设计第 14 节。估计、感知异步产生不可变数据；世界模型整合状态、三态占据与动态预测；任务和局部规划产生轨迹，经独立校验后进入飞控及唯一发布者。

当前真实实现：interfaces/contracts.py、world_model/occupancy/grid.py、planning/speed_envelope/braking.py、control/safety_supervisor/preflight.py、adapters/ros_noetic/interface_audit.py、evaluation/seed_runner/manifest.py。

occupancy/query 是小规模离线基线，不满足实时大体素地图的性能要求。preflight 只提供必要条件，不证明碰撞安全，也不执行紧急制动。其余目录标记为待实施，不能作为已运行的飞行模块。

PWM/VEL 发布器尚未实现，因此 P0 审计不会锁定模拟器控制接口。不得直接迁移旧 route_follower.py 作为新架构入口。
