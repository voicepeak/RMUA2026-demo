# 缝隙延迟与门间停止证据

实现及逐轮边界见[修订记录](../../31_gap_latency_and_stationary_stall.md)。原始实验位于工作区`experiments/legacy_visible_20261008_02`至`29`和`experiments/gap_diagnosis_20261008`，不整体提交点云与比赛日志。

- `host_final_tests.log`、`ros_final_tests.log`：批量高度、相机视野和静止回退修订，各455项通过。
- `host_pose_tests.log`、`ros_pose_tests.log`：加上真实飞行位置高度校准，各463项通过。ROS日志保留无效数值测试warning和几何工作进程退出的EOF输出；最终退出码0。
- `host_brake_tests.log`、`ros_brake_tests.log`：加上未认证高度回退制动及控制参考/校准记录分离，各466项通过。
- `host_coupled_tests.log`、`ros_coupled_tests.log`：出程coupled参数与高速阻塞制动余量，各468项通过。
- `runtime_manifest.json`：实际运行包与编辑源71文件逐项一致的SHA256；05轮启动目录另含不可变源/工具/模拟器快照。
- `batch_benchmark.json`：合成高度混合计算对照，不能视作实跑倍率。
- `stall_03_telemetry.json`、`stall_cycles.json`：1300m静止现场及两周期反事实重放，不能视作真实脱困。
- `comparison.json`：02/03/04轮统一取车辆控制器开始后的180墙钟秒；没有HUD有效时间证明，官方车辆终点未切段。
- `pose_prior_replay.json`：04轮真实位置的高度校准离线结果，不是飞行通过证明。
- `vertical_block_replay.json`：05轮107个冻结现场，以记录的上一条实际指令重放Guard；停止高度曲线重建，不能代替完整现场或实飞。
- `outbound_coupling_loss.json`：06轮发布复核前遥测，未限幅水平误差对应补偿超过垂直能力；不是精确动力学辨识或独立碰撞原因证明。

默认入口仍是legacy。没有验证spacetime真实桥接失效时间，也未解决首轮可见新模式的观测/发布时钟竞态；本目录不是新模式运动验收证据。

- `host_departure_tests.log`、`ros_departure_tests.log`：起飞条件恢复、禁止前向追踪和过门期间洞口约束，各472项通过。
- `legacy_hold_07.json`：实际车辆指令发布间隔，9/454超过停止包络默认0.35s；不是外部桥接失效测量。
- `departure_stall_08.json`：第二阶段起飞指令零速/Guard拒绝的真实记录；官方结果和车辆刷新条件未确认。
- `departure_replay_08.log`：08轮早期冻结点云及同期位姿恢复入口回放；不替代飞行验证。

- `simulator_archive_check.json`、`simulator_files_check.json`：官方对象大小及完整zip CRC64匹配，尝试分片ETag未匹配所以未用于结论，以及运行二进制/场景pak与zip的CRC32匹配。
- `simulator_spawn_09.json`：3→5切段的112条生成碰撞警告；不是全部车辆未生成的证明。

- `host_terminal_tests.log`、`ros_terminal_tests.log`：触发区外衔接高度，各477项通过。
- `host_watchdog_tests.log`、`ros_watchdog_tests.log`：指令超时、真实发布入口及stop=1，各486项通过。
- `host_floor_recovery_tests.log`、`ros_floor_recovery_tests.log`：残余漂移恢复与每个响应情景停止尾部，各488项通过。
- `watchdog_install.log`、`floor_recovery_install.log`：两个ROS包的catkin构建/安装通过。
- `cuda_after_repair.json`：重载闲置计算模块后，实际GPU张量预检通过。
- `vehicle_gap_stall13.jpg`、`hud_stall13.png`：停在9.58m时，前视车辆和可见通路、HUD State2。
- `replay_floor_stall13.json`：保留实际残余速度的冻结对照；停车曲线重建，不是完整飞行状态重放。

14轮在13轮卡点恢复30秒，9.582m→58.430m；15轮出程超时，不能宣称车辆段通关。16轮出程通过、车辆段345m力场超时失败；17轮坡度制动修正验证进行中。

- `resume14_summary.json`：同一卡点的真实局部恢复，不是新比赛验收。
- `hardware_stop13_observations.json`：移动中stop=1后的惯性记录，否定瞬时停止假设。
- `flight15_summary.json`、`hud_finished15.png`：出程180.025s、Score62结束，没有车辆段通过证明。
- `host_feedback_deadline_tests.log`、`ros_feedback_deadline_tests.log`：撤回高速硬停、使用私有模型与实际反馈制动，各489项通过。
- `feedback_deadline_install.log`：本次修正的catkin构建安装通过。

- `flight16_summary.json`、`hud_finished16.png`：出程167.686s切段，车辆段约345m、HUD OutTime3.808s失败；原9.58m卡点已越过，但不是整段验收。
- `host_grade_brake_tests.log`、`ros_grade_brake_tests.log`：保留坡度的未认证制动及静止抑制，各490项通过。
- `grade_brake_install.log`：本次修正的catkin构建安装通过。


## 本次批次合并的最终状态

最终宿主/ROS各495项测试通过：`host_merge_final_tests.log`、`ros_merge_final_tests.log`。`runtime_manifest.json`已刷新为当前已合入71文件一致清单；此前各日志保留其历史版本边界。`merge_final_install.log`记录实际运行包的catkin构建安装。

`flight17_summary.json`、`flight19_summary.json`、`flight23_summary.json`、`flight27_summary.json`、`flight29_summary.json`保留实跑结果。run23/27约804/776m均未完成3→5；run29约9.44m诊断停止，官方UNKNOWN。`band_transient_experiments.md`与`frozen_replay_patched.json`保留后续诊断，原点云和完整日志仍在工作区。

最新比赛入口开启1.15m原始回波包络基础裕度和0.9m侧向裕度；参考带与出发地面允许有界瞬态；贴面EMERGENCY_RETREAT是未认证回退。原始1.25m完整余量不能套用到这些启用的分支，测试通过不代表整段稳定通关。四批均在独立分支提交、检查后合入main并推送；清单见`merge_batches.json`。进行中的新纵向规划器、对应测试及后续控制器接线不在本次合并内。

- `batch1_tests.log`、`batch2_tests.log`、`batch3_tests.log`：从各批提交的独立工作树检查，分别463/493/495项通过；第3批完整源码另经ROS495项检查。

发布日志副本仅清理行尾空格；原始日志仍保留在实验目录或原采集位置。
