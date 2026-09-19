# RMUA 2026：Gate 四角关键点、时序同步、反投影与高速转弯稳定方案

## 0. 本阶段目标

当前系统已经完成：

- YOLO Gate 检测；
- 双目深度估计；
- Gate Camera → Body → World 坐标转换；
- Persistent Gate Track / Gate Map；
- Gate Chain；
- Yaw 前视控制；
- 连续三维参考轨迹；
- 基础 Z Profile；
- 高速飞行尝试。

当前主要问题集中在：

```text
1. 转弯 / 高速时 Gate 仍可能丢失；
2. 同一 Gate 的 World Z 可能随姿态变化产生漂移；
3. bbox 中心并不是 Gate 实体平面上的可靠深度点；
4. 图像、双目、IMU / pose 可能存在时间不同步；
5. Gate 关联仍可能发生前后串错；
6. 一旦错误 Gate Z 进入 Path，会继续把飞机向下拉。
```

本阶段目标：

> **把 Gate 三维定位从“bbox + 单点深度”升级为“四角几何 + 时间同步 + 世界地图预测 + 反投影校验”的稳定感知链。**

最终要求：

```text
飞机高速转弯
↓
Yaw 提前看弯
↓
Gate 仍尽量保持在 FOV 中
↓
即使部分丢失，也能靠 Persistent Gate Map 预测
↓
重新检测后正确关联到原 Gate
↓
同一 Gate World XYZ 尤其 Z 基本稳定
↓
不会因为姿态 / 错匹配把 Z Path 拉向下方
```

---

# 1. 当前优先级重新排序

当前最值得做的功能：

## Priority 1

```text
YOLO Pose / 四角关键点
```

## Priority 2

```text
Image + Stereo + IMU / Pose 时间同步
```

## Priority 3

```text
Gate 四角 3D 几何一致性检查
```

## Priority 4

```text
Persistent World Gate → Image 反投影
```

## Priority 5

```text
Yaw FOV correction
```

当前继续暂缓：

```text
LiDAR Gate Detection
后视双目主控
复杂避障
```

---

# 2. 为什么普通 YOLO bbox 不够

Gate 是一个空心矩形：

```text
┌─────────────┐
│             │
│     空      │
│             │
└─────────────┘
```

bbox 中心：

```text
×
```

通常落在：

```text
门洞内部
```

而不是：

```text
门框实体。
```

因此如果通过：

```text
bbox center
```

直接采双目 disparity / depth，

得到的可能是：

```text
Gate 后方道路
墙
天空
其他物体
```

导致：

```text
Depth 错
↓
Camera XYZ 错
↓
World XYZ 错
↓
尤其 World Z 错
↓
Z Path 被错误拉低 / 拉高
```

---

# 3. 改成四角关键点检测

推荐 Gate Detector 输出：

```text
TL ───────── TR
│             │
│             │
BL ───────── BR
```

即：

```text
Top Left
Top Right
Bottom Right
Bottom Left
```

同时保留：

```text
bbox
confidence
track_id
```

统一输出：

```text
Gate2DObservation
```

字段：

```text
track_id
bbox
tl
tr
br
bl
center
confidence
timestamp
```

---

# 4. YOLO Pose 方案

后续模型建议使用：

```text
YOLO Pose / Keypoint Detection
```

每个 Gate 输出：

```text
4 keypoints
```

顺序固定：

```text
0 = TL
1 = TR
2 = BR
3 = BL
```

不要让训练集不同样本角点顺序变化。

否则会导致：

```text
左右双目角点关联错误。
```

---

# 5. 双目四角匹配

左图：

```text
TL_L
TR_L
BR_L
BL_L
```

右图：

```text
TL_R
TR_R
BR_R
BL_R
```

分别匹配：

```text
TL_L ↔ TL_R
TR_L ↔ TR_R
BR_L ↔ BR_R
BL_L ↔ BL_R
```

然后分别进行：

```text
Stereo Triangulation
```

得到：

```text
P_TL
P_TR
P_BR
P_BL
```

四个真实 3D 点。

---

# 6. Gate Center 不再由 bbox center 得到

新的 Gate Center：

\[
P_G =
\frac{
P_{TL}+P_{TR}+P_{BR}+P_{BL}
}{4}
\]

优点：

```text
Gate Center 来自门框真实几何
```

而不是：

```text
门洞背景深度。
```

---

# 7. 同时计算 Gate Plane

使用：

\[
v_1 = P_{TR}-P_{TL}
\]

\[
v_2 = P_{BL}-P_{TL}
\]

Gate Normal：

\[
n =
\frac{
v_1 \times v_2
}{
\|v_1 \times v_2\|
}
\]

并得到：

```text
Gate Width
Gate Height
Gate Plane
Gate Normal
```

这些以后可直接用于：

```text
穿门判定
Gate Corridor
避障
局部轨迹规划
```

---

# 8. 四角几何一致性检查

一个 Gate 不能只因为：

```text
YOLO confidence 高
```

就成为 3D Hard Anchor。

必须通过以下检查。

---

# 9. 深度一致性

四个角点深度：

```text
d_tl
d_tr
d_br
d_bl
```

如果：

```text
max(depth) - min(depth)
```

异常大，

则：

```text
Reject 当前观测。
```

注意：

```text
斜着看 Gate 时
四角深度本来会有一定差异
```

所以不能要求：

```text
完全相等。
```

应根据 Gate Plane 拟合残差判断。

---

# 10. Gate Plane Residual

用四点拟合平面：

```text
ax + by + cz + d = 0
```

计算四点到拟合平面的误差：

```text
plane_rmse
```

如果：

```text
plane_rmse > threshold
```

则说明：

```text
至少一个角点三角化异常
```

这一帧：

```text
不更新 Persistent Gate。
```

---

# 11. Width / Height 检查

从 3D 四角计算：

```text
width_top
width_bottom
height_left
height_right
```

要求：

```text
width_top ≈ width_bottom
height_left ≈ height_right
```

允许透视和估计误差。

如果：

```text
一边长 2m
另一边长 15m
```

明显错误，

直接：

```text
Reject。
```

---

# 12. Rectangle Ratio 检查

Gate 真实长宽比应处于合理范围。

保存：

```text
width
height
aspect_ratio
```

如果：

```text
aspect_ratio
```

偏离长期统计太多，

标记：

```text
suspect.
```

---

# 13. 四角输出最终 Gate Quality

每一帧输出：

```text
GateGeometryQuality
```

例如：

```text
keypoints_complete
stereo_valid
plane_valid
size_valid
aspect_valid
depth_valid
```

最后：

```text
geometry_valid =
all checks passed
```

---

# 14. Hard Anchor / Soft Anchor

当前正式区分：

## Soft Anchor

满足：

```text
YOLO 检测成功
双目大致正常
```

但：

```text
四角质量一般
远距离
support 少
sigma_z 较大
```

作用：

```text
只提供未来趋势。
```

不能：

```text
强制决定 z_profile。
```

---

## Hard Anchor

必须满足：

```text
四角完整
+
Stereo 四角有效
+
Plane 合理
+
Width / Height 合理
+
多帧稳定
+
World XYZ sigma 足够小
```

才能：

```text
参与真实 3D Path。
```

---

# 15. 第二个核心：时间同步

即使：

```text
Camera → Body → World
```

数学完全正确，

如果：

```text
姿态时间戳
```

和：

```text
图像时间戳
```

不一致，

World Gate XYZ 仍然会漂。

---

# 16. 高速下时间不同步为什么危险

例如：

```text
Camera frame:
t = 10.000s
```

姿态却使用：

```text
IMU latest:
t = 10.050s
```

在：

```text
10m/s
```

下，

50ms 已经移动：

```text
0.5m
```

如果此时还在：

```text
高速转弯
pitch
roll
```

那么：

```text
camera pose(t_camera)
≠
pose(t_imu)
```

最终：

```text
World XYZ 漂移。
```

---

# 17. 统一时间戳

所有 Gate 3D Observation 必须绑定：

```text
stereo_timestamp
```

推荐：

```text
t_stereo
=
(left_stamp + right_stamp) / 2
```

或选择：

```text
同步后的公共时间。
```

之后查找：

```text
t_stereo
```

对应的：

```text
IMU quaternion
position
```

---

# 18. IMU Quaternion 插值

缓存最近：

```text
0.5 ~ 1 秒
```

IMU 数据。

寻找：

```text
t0 < t_camera < t1
```

两个姿态：

```text
q0
q1
```

通过：

```text
SLERP
```

插值：

```text
q_camera
```

即：

> **真正使用拍摄这一帧时刻的姿态。**

---

# 19. Position 也要时间对齐

同理：

```text
World Position
```

也应该插值到：

```text
camera_timestamp。
```

位置可以：

```text
Linear Interpolation
```

姿态：

```text
Quaternion SLERP
```

---

# 20. Camera → Body → World 正确链路

Camera Point：

\[
P_C
\]

Camera → Body：

\[
P_B
=
R_{BC}P_C+t_{BC}
\]

Body → World：

\[
P_W
=
R_{WB}(t_{camera})P_B
+
t_{WB}(t_{camera})
\]

其中：

```text
R_WB
```

必须来自完整 quaternion：

```text
roll
pitch
yaw
```

不能只用：

```text
yaw。
```

---

# 21. 第三个核心：转弯 / 高速姿态必须完整补偿

高速：

```text
8 ~ 10m/s
```

时，

无人机会明显：

```text
Pitch
Roll
Yaw
```

如果只使用：

```text
Yaw
```

那么相机：

```text
俯仰 / 横滚
```

会被错误解释为：

```text
目标 Z / Y 变化。
```

---

# 22. 必做验证：固定 Gate World Z Test

选一扇固定 Gate。

让无人机：

```text
左右转弯
加速
减速
Pitch
Roll
```

记录：

```text
gate_camera_xyz
gate_world_xyz

roll
pitch
yaw
```

理想：

```text
World Gate Z
```

基本保持不变。

例如：

```text
-7.21
-7.18
-7.24
-7.20
```

允许小幅波动。

如果出现：

```text
-7.2
-8.0
-9.1
-10.4
```

则：

```text
3D Transform / Timestamp
```

仍然有问题。

在这个问题解决前：

```text
不要继续调 Z Controller。
```

---

# 23. 第四个核心：Persistent Gate Map 反投影

现在已经是：

```text
Image
↓
Gate
↓
World XYZ
```

下一步增加反向：

```text
Persistent World Gate
↓
当前 Camera Pose
↓
Project
↓
Expected Pixel Position
```

---

# 24. World → Camera

已知：

```text
Gate World XYZ
```

和：

```text
当前 Camera Pose
```

转换：

\[
P_C
=
R_{CW}(P_W-t_{WC})
\]

如果：

```text
Z_camera <= 0
```

说明 Gate：

```text
在相机后面
```

不应成为：

```text
Future Visible Gate。
```

---

# 25. Camera → Pixel

使用内参：

\[
u =
f_x \frac{X_c}{Z_c}+c_x
\]

\[
v =
f_y \frac{Y_c}{Z_c}+c_y
\]

得到：

```text
predicted_u
predicted_v
```

---

# 26. 反投影用于 Gate 数据关联

例如：

```text
Persistent Gate 7
```

预测当前应出现在：

```text
u=620
v=330
```

YOLO 检测：

```text
Detection A:
u=615
v=336
```

则：

```text
高概率是 Gate 7。
```

另一个 Detection：

```text
u=180
v=500
```

即使 confidence 更高：

```text
也不应该关联 Gate 7。
```

---

# 27. Gate Association Score

建议：

```text
score =
w_pixel * pixel_distance
+
w_depth * depth_difference
+
w_s * route_progress_difference
+
w_size * size_difference
+
w_track * temporal_difference
```

最终寻找：

```text
minimum cost match。
```

不要只依赖：

```text
bbox IoU
```

或：

```text
深度排序。
```

---

# 28. 反投影还能解决“前后 Gate 串错”

未来 Gate Queue：

```text
G0
G1
G2
G3
```

每个都有：

```text
predicted image location
predicted depth
path_s
```

新 YOLO Detection 必须同时满足：

```text
像素位置合理
+
深度合理
+
s 顺序合理
```

才能更新对应 Gate。

这样可以显著降低：

```text
远处 Gate
被误认为最近 Gate
```

的问题。

---

# 29. 第五个核心：Yaw FOV Correction

当前已经有：

```text
Yaw → Future Path Tangent
```

继续保留作为主控制。

新增一个：

```text
视觉 FOV correction
```

只用于：

> **让未来 Gate 尽量保持在相机中间区域。**

---

# 30. 计算图像 Gate 中心偏差

未来可靠 Gate：

```text
u_gate
```

图像中心：

```text
cx
```

定义：

\[
e_{img}
=
\frac{u_{gate}-c_x}{f_x}
\]

如果有多个 Gate：

```text
使用近 2~3 个可靠 Gate
```

做加权平均：

```text
e_img_mean
```

---

# 31. Yaw 新控制

原：

\[
yawRate
=
K_{path}e_{yaw}
\]

新：

\[
yawRate
=
K_{path}e_{yaw}
+
K_{vision}e_{img}
\]

其中：

```text
Kvision
```

必须明显小于：

```text
Kpath。
```

---

# 32. 这个视觉项不是 Gate 吸附

特别注意：

```text
位置控制
```

仍然沿：

```text
3D Reference Curve。
```

视觉只影响：

```text
机头朝向。
```

因此：

```text
无人机不会被 Gate 直接拉过去
```

但：

```text
相机会主动稍微看向未来 Gate。
```

---

# 33. Yaw 优先级

推荐：

```text
第一：
Future Gate Chain tangent

第二：
XY Route tangent

第三：
Velocity direction
```

视觉 FOV Correction：

```text
作为小修正项。
```

---

# 34. Gate Lost 时不要用视觉修正

如果：

```text
future_gate_count = 0
```

则：

```text
Kvision = 0
```

Yaw：

```text
只跟 Route。
```

避免：

```text
错误 Detection
```

把机头拉偏。

---

# 35. Z Path 的输入规则必须重新明确

当前：

```text
Hard Anchor
```

才能强约束：

```text
z_profile。
```

Soft Anchor：

```text
只能影响未来趋势
```

不能：

```text
单独把曲线拉到某个极端 Z。
```

---

# 36. Gate Z 更新规则

新的 Gate Z 进入 Hard Anchor 前：

必须通过：

```text
1. Keypoint complete
2. Stereo valid
3. Plane valid
4. Size valid
5. Timestamp synchronized
6. Full quaternion transform
7. Multi-frame stable
8. sigma_z below threshold
9. route order valid
```

---

# 37. 无可靠 Future Gate 时的 Z 行为

如果：

```text
Future Hard Anchors > 0
```

正常：

```text
跟踪 z_profile。
```

如果：

```text
只有 Soft Anchors
```

则：

```text
曲线只做有限趋势调整。
```

如果：

```text
Future Gate = 0
```

则：

```text
停止继续外推 dz/ds。
```

逐渐：

```text
dz/ds → 0
```

最终：

```text
Hold Last Reliable Z。
```

---

# 38. 禁止 Z 无限下降

增加：

```text
z_extrapolation_horizon
```

例如：

```text
10 ~ 20m
```

或：

```text
1 ~ 2s
```

超过后：

```text
Z slope → 0。
```

这样：

```text
视觉丢失
```

不会变成：

```text
无人机一路向下。
```

---

# 39. 删除旧的停车爬升策略

正式删除正常情况下：

```text
z_error 大
↓
xy_speed = 0
↓
原地爬升
```

Z Error 只用于：

```text
适当降速。
```

例如：

```text
|z_err| < 0.3
→ 100% speed

0.3~0.6
→ 80%

0.6~1.0
→ 60%

>1.0
→ 40%
```

但仍：

```text
继续沿三维轨迹前进。
```

只有：

```text
Emergency
```

才：

```text
真正停车。
```

---

# 40. Gate Map 数据结构升级

推荐每个 Gate：

```yaml
id: 7

world:
  x: ...
  y: ...
  z: ...

normal:
  x: ...
  y: ...
  z: ...

size:
  width: ...
  height: ...

quality:
  confidence: ...
  support: ...
  sigma_x: ...
  sigma_y: ...
  sigma_z: ...
  plane_rmse: ...
  geometry_valid: true
  timestamp_valid: true
  hard_anchor: true

tracking:
  last_seen: ...
  age: ...
  predicted_u: ...
  predicted_v: ...
```

---

# 41. 推荐新增模块

视觉侧：

```text
rmua_gate_vision/
├── gate_detector_yolo_pose.py
├── stereo_keypoint_matcher.py
├── gate_geometry.py
├── timestamp_sync.py
├── gate_world_transform.py
├── gate_tracker.py
├── gate_map.py
└── gate_reprojection.py
```

控制侧：

```text
route_follower/
├── gate_chain.py
├── reference_3d.py
├── yaw_controller.py
├── z_controller.py
├── speed_scheduler.py
└── route_follower.py
```

---

# 42. timestamp_sync.py

职责：

```text
缓存 IMU
缓存 Pose

输入：
image timestamp

输出：
position(t)
quaternion(t)
```

内部：

```text
position:
linear interpolation

quaternion:
SLERP
```

---

# 43. gate_geometry.py

职责：

```text
输入：
4 个 3D 角点

输出：
center
normal
width
height
plane_rmse
geometry_valid
```

---

# 44. gate_reprojection.py

职责：

```text
World Gate
+
Current Camera Pose
+
Camera Intrinsics
↓
predicted pixel
predicted depth
visible flag
```

用于：

```text
tracking
association
FOV correction
```

---

# 45. gate_tracker.py

Tracker 更新不再只依赖：

```text
当前 YOLO Detection
```

而是：

```text
Persistent prediction
+
YOLO detection
+
Stereo depth
+
Route s
```

共同关联。

---

# 46. 当前不做 LiDAR Gate Detection

LiDAR 继续预留：

```text
Obstacle
Geometry Safety
Local Planner
```

但当前：

```text
不把它加入 Gate 主感知。
```

避免一次改太多系统。

---

# 47. 当前不做后视双目主控制

后视双目后续用途：

```text
Gate Pass Confirmation
Recovery
```

当前仍以：

```text
Front Stereo
```

为主。

---

# 48. 第一轮开发顺序

## Step 1

实现：

```text
YOLO Pose 四角输出。
```

先只看：

```text
图像四角是否稳定。
```

---

## Step 2

实现：

```text
Stereo 四角三角化。
```

输出：

```text
4 × XYZ。
```

---

## Step 3

加入：

```text
Plane
Width
Height
Geometry Quality。
```

---

## Step 4

实现：

```text
Image / IMU / Pose 时间同步。
```

---

## Step 5

确保：

```text
Camera → World
```

使用：

```text
完整 quaternion。
```

---

## Step 6

做：

```text
Fixed Gate World Z Test。
```

高速转弯时：

```text
World Z 必须稳定。
```

---

## Step 7

实现：

```text
World Gate → Image Reprojection。
```

---

## Step 8

用：

```text
Reprojection
```

强化：

```text
Gate Association。
```

---

## Step 9

Yaw 加：

```text
小幅 FOV correction。
```

---

## Step 10

重新测试：

```text
6m/s
8m/s
10m/s
```

观察：

```text
Gate 是否更稳定留在前方
World Z 是否仍稳定
Z 是否还会异常下坠。
```

---

# 49. 最关键的验证实验

选定：

```text
同一扇 Gate
```

连续跑：

```text
直线
左转
右转
加速
减速
上坡
```

同时记录：

```text
image timestamp

TL/TR/BR/BL pixel

4-corner camera XYZ

Gate camera center XYZ

Gate world XYZ

roll
pitch
yaw

predicted_u
predicted_v

YOLO detected_u
YOLO detected_v

sigma_z
plane_rmse
hard_anchor
```

---

# 50. 成功标准

同一个固定 Gate：

```text
World X/Y/Z
```

在无人机运动过程中：

```text
保持稳定。
```

特别是：

```text
World Z
```

不能随着：

```text
pitch
roll
yaw
```

持续漂移。

---

# 51. Gate Tracking 成功标准

高速转弯时：

```text
Detection 暂时丢失
```

允许。

但是：

```text
Persistent Gate Track
```

不能马上消失。

重新看到 Gate 时：

```text
通过反投影
```

应重新匹配到：

```text
原 Track ID。
```

---

# 52. Yaw 成功标准

转弯前：

```text
Yaw 提前开始旋转。
```

未来 Gate：

```text
尽量保持在画面中央区域。
```

但：

```text
位置轨迹
```

不因为视觉偏差发生明显 Gate 吸附。

---

# 53. Z 成功标准

视觉短暂丢失：

```text
Z 不继续无限下降。
```

错误 Gate：

```text
不能成为 Hard Anchor。
```

高速姿态变化：

```text
不能导致 Gate Z 大幅漂移。
```

---

# 54. 最终控制链

```text
                 YOLO Pose
                    ↓
             4 Gate Keypoints
                    ↓
             Stereo Triangulate
                    ↓
          4 × Camera 3D Points
                    ↓
             Gate Geometry
        center / normal / size
                    ↓
           Timestamp Synced Pose
                    ↓
      Full Quaternion World Transform
                    ↓
             Persistent Gate Map
                    ↓
              Reprojection
                    ↓
        Data Association / Tracking
                    ↓
             Ordered Gate Chain
               │             │
               │             └────→ Yaw FOV Correction
               │
               ↓
          3D Reference Curve
               ↓
      Continuous vx / vy / vz
               ↓
          Speed Scheduler
               ↓
          World → Body
               ↓
          vel_body_cmd
```

---

# 55. 与当前方案相比的关键升级

之前：

```text
YOLO bbox
↓
双目
↓
Gate Center
↓
World XYZ
```

现在：

```text
YOLO Pose 四角
↓
四角双目三角化
↓
几何质量验证
↓
时间同步姿态
↓
完整 Quaternion
↓
World Gate
↓
Persistent Map
↓
反投影验证
↓
Gate Chain
```

即：

> **从“检测到一个矩形”升级成“维护一个有几何约束、有时间约束、有世界坐标一致性的 Gate 地图”。**

---

# 56. 本阶段最终结论

当前 Z 下坠问题已经不能只看作：

```text
飞控高度控制问题。
```

更可能是：

```text
高速转弯
↓
Gate FOV 变化
↓
检测 / 关联变化
↓
姿态时间不同步
↓
Gate World Z 漂移
↓
错误 Z Anchor
↓
Z Path 下沉
```

因此这阶段真正应该优先解决：

```text
Gate 四角几何
+
时间同步
+
完整 Quaternion
+
Persistent Gate Map
+
World → Image 反投影
```

而不是继续单独增加：

```text
Kz
```

或：

```text
高度补偿器。
```

完成以后，10m/s 高速状态下 Gate Z 的稳定性和转弯后的持续可见性都会有明显提升，也会为后续 LiDAR 避障和局部规划提供更可靠的三维 Gate 地图。
