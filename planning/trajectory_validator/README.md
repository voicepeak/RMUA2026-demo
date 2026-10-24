# 静态扫掠体积核验

`static_sweep.check_static_sweep(trajectory, snapshot, now, epoch, bounds)` 对已有
`Trajectory` 和 `OccupancySnapshot` 检查从 now 到有效期终点的连续扫掠空间，返回
`StaticSweepResult`。未知、过期、其他 epoch、坐标系不匹配及工作预算不足均拒绝。

机体使用 world_ned 轴向包围盒；`SweepBounds.half_extents_m` 必须由调用方提供，并覆盖
允许姿态下的完整机体/旋翼。再按跟踪误差和 `a_max * sample_dt**2 / 8` 膨胀，用于包住
采样间的连续弯曲轨迹。后者要求规划器保证整个区间的参考加速度上界，仅检查采样值
无法证明该假设。裁剪到轨迹后半段时仍保留原始采样间隔的偏离界。

逐体素用线段与膨胀盒的 slab 相交区间检查，接触面也算相交，防止离散采样穿墙。
FREE 的原始来源、frame、epoch 和有效期必须覆盖该体素的接触区间；不会续期或创造
FREE。超出候选体素/分段预算直接拒绝，不把部分检查当成通过。

`clear=True` 仅表示在给定地图、包络和偏差假设下通过静态空间检查。它不证明实际
传感器/包络已标定，也不包含动态对象、道路/力场、制动、推力余量和控制健康检查。
当前 LiDAR 端点图仍在传感器坐标系且无 FREE，不能直接用于放行飞行。

实现对应设计 6.1、11.3；阶段验收状态见 docs/implementation_status.md。
