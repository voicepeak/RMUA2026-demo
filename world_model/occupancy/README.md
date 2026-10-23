grid.py 提供不可变三态快照；缺失、过期、未来或其他 epoch 的体素查询为 UNKNOWN。

lidar.py 的 endpoint_snapshot 将单帧有效回波落入 0.25m 体素，仅生成 OCCUPIED。零距离、达到/超过 30m 的不确定回波、NaN/Inf、不见区域和遮挡后方均不生成 FREE。每帧单独构建，不累积旧传感器坐标，不擅自去除近场或地面回波。

输入来自 adapters/ros_noetic/lidar_cloud.py 的有界 PointCloud2 解码，支持组织点云、行/点填充、字段重排、大小端与 float32/float64；保留 lidar 原始 frame 和 stamp，不隐式转换到 world_ned。点数组由不可写 bytes 支撑。构建时拒绝过期、未来和 epoch 不一致。

这是扫描时刻固定传感器坐标下的端点证据，尚无逐点运动补偿、外参/时延标定、位置不确定度膨胀或自由空间认证。不能直接当作运动中的世界地图、规划净空或三态地图验收。规则要求的三态语义已保留，FREE 的可信生成仍待实现。

只读在线验证及 ROS 独立解码对照见 docs/lidar_endpoint_observation.md；整体状态见 docs/implementation_status.md。
