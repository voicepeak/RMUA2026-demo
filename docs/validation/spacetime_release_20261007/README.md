# 发布验证摘要

收尾日期：2026-10-08，开发批次起于2026-10-07。完整说明见[执行层记录](../../30_spacetime_execution_implementation.md)。

- `release_manifest.json`：69文件源码SHA256、运行副本一致性、测试及验收边界。
- `host_checked_tests.log`、`ros_checked_tests.log`：两端各449项通过；ROS旧几何worker EOF为预期故障测试输出。
- `oracle_summary.json`：五例理想FREE/CV闭环通过；350ms离线计算预算，不能宣称实时实跑。
- `fault_summary.json`：九类数值调度/进程故障通过，桥接失效契约为假定。
- `ros_io_summary.json`：安装节点111条实际ROS指令全部零，周期验证通过；没有起飞/新比赛。
- `catkin_install.log`、`runtime_build.log`：构建/安装完整输出；仅去掉行尾空格，原始日志保留在本地实验目录。
- `installed_checked_smoke.log`：安装目录codec/Planner/共享Guard及完整停止认证通过。

完整日志、历史失败、NPZ冻结输入、交互预览、运行包修改前备份在工作区`experiments/spacetime_execution_refactor_20261007/`，未把大型实验目录提交到Git。新模式真实桥接失效时间与完整车辆段A/B/重复验收尚未完成；默认legacy。
