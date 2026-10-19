# 坐标与时间契约 v0.1

世界坐标使用 world_ned，米、秒、弧度，Z 向下；四元数顺序 w,x,y,z，表示机体系到 NED 的旋转。机体系按 FRD（前、右、下）约定；相机光学系和 LiDAR 实际外参必须在 P0/P2 标定后显式转换。

stamp/valid_until 均为同一个模拟器时间域，使用半开有效期 [stamp, valid_until)。reset 或时钟回退必须递增 epoch，旧 epoch 数据全部失效。有效性检查不能代替输入时间戳单调性监控。

wall monotonic 只用于执行 deadline 和接收频率统计，不与模拟器 stamp 相减。审计记录的 ROS now-stamp 仅作时钟差诊断，不宣称是端到端控制延迟。GPS 归零无效判别和估计器融合尚待 P2 实现。

冻结 dataclass 的嵌套字段必须为 tuple。协方差或保守边界的矩阵布局与单位将在各传感器适配器接入前单独版本化；当前不据此宣称估计不确定度已校准。
