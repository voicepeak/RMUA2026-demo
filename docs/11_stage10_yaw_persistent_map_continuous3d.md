# RMUA 2026：转弯丢 Gate、Z 下坠与连续三维跟踪修复方案

## 0. 当前问题

最近两轮测试暴露的核心问题不是单独的 Z 控制，而是一整条链路：

```text
转弯开始
→ 无人机位置沿 XY 路径改变
→ yaw 没有提前跟随未来路径
→ 前视双目仍朝旧方向
→ 下一批 Gate 离开视野
→ YOLO / 双目 Gate Chain 中断
→ Z 参考失去可靠未来锚点或继续错误外推
→ 无人机下降
→ 从坡下离开合法区域并结束
```

同时，上一版“高度误差大 → XY 速度降到 0 → 原地爬升 → 再继续”的补偿策略不适合当前竞速任务，因为它会让无人机停在门前，破坏连续轨迹。

本阶段目标是：

> **让相机始终尽量看向未来赛道，让视觉 Gate 形成持久世界坐标地图，并让无人机沿连续三维轨迹边前进边升降。**

---

# 1. 总体方案

新的主链路：

```text
XY Route / Gate Chain
        ↓
Future Path Direction
        ↓
Yaw Controller
        ↓
相机尽量始终看向未来赛道
        ↓
YOLO + Stereo
        ↓
Gate 3D Observation
        ↓
完整姿态补偿
(IMU / Quaternion)
        ↓
Persistent Gate Map
        ↓
Ordered Gate Chain
        ↓
Continuous 3D Reference Curve
        ↓
vx + vy + vz 连续控制
        ↓
vel_body_cmd
```

原则：

```text
视觉负责发现 Gate；
Gate Map 负责记住 Gate；
Route / Gate Chain 负责规划；
Z 不依赖某一帧视觉；
正常上坡时不能停车。
```

---

# 2. Yaw 必须跟随未来路径

当前若一直：

```text
yaw_rate = 0
```

无人机虽然仍能通过 body XY 速度侧着转弯，但机头不会朝新路线方向旋转，前视相机也不会转过去。

因此需要增加真正的 Yaw Path Following。

当前位置：

```text
P = [x, y]
```

取未来路径前方一个点：

```text
Q = [xq, yq]
```

期望 yaw：

\[
\psi_{target}=atan2(y_q-y, x_q-x)
\]

误差：

\[
e_\psi=wrap(\psi_{target}-\psi)
\]

控制：

\[
yaw_{rate}=K_{yaw}e_\psi
\]

并限幅：

```text
|yaw_rate| <= yaw_rate_max
```

---

# 3. Yaw 不要只盯最近一扇门

不推荐：

```text
yaw_target = nearest_gate_center
```

因为连续多个 Gate 会让机头不断小幅摆动。

推荐：

> **使用未来 Gate Chain / Reference Curve 的切线方向作为 yaw_target。**

优先级：

```text
1. 有可靠 Future Gate Chain
   → 看 Gate Chain tangent

2. Future Gate 暂时不足
   → 看 XY Route tangent

3. Route 也暂时异常
   → 保持当前运动方向
```

---

# 4. Yaw 前视距离要比位置前视更远

建议第一版：

```text
Position Look-ahead:
8 ~ 15 m

Yaw Look-ahead:
20 ~ 30 m
```

这样无人机还没有进入弯道时，机头已经开始提前转向后续路径。

效果：

```text
未来 Gate 更长时间停留在前视 FOV 内
```

首轮参数：

```text
yaw_lookahead = 20 ~ 25 m
yaw_rate_max = 0.5 ~ 1.0 rad/s
```

---

# 5. 视觉仍然是 Gate 的主要语义来源

当前 Gate 识别继续采用：

```text
YOLO
+
双目
```

职责分开：

```text
YOLO：
Gate 是什么、2D 在哪里

Stereo：
Gate 的相对 3D 坐标

IMU / Pose：
相机当前姿态

Persistent Gate Map：
Gate 的世界坐标记忆

Route：
Gate 大致应该出现在哪个方向
```

但视觉不能继续直接变成每一帧的控制输入。

---

# 6. 建立 Persistent Gate Map

一个 Gate 如果已经满足：

```text
连续多帧检测
双目结果稳定
World XYZ 稳定
顺序合理
```

就保存为 Persistent Gate，例如：

```yaml
gate_id: 6
x: ...
y: ...
z: ...
path_s: ...
confidence: ...
support: ...
valid: true
last_seen: ...
```

以后哪怕 YOLO 短暂丢失：

```text
Gate 不从地图中消失。
```

---

# 7. Perception 与 Planning 解耦

禁止：

```text
YOLO 当前帧
→ 当前 Gate Z
→ 直接控制 vz
```

改为：

```text
YOLO + Stereo
      ↓
Gate Observation
      ↓
Persistent Gate Map
      ↓
Gate Chain
      ↓
3D Planner
      ↓
Control
```

于是：

```text
视觉偶尔丢 0.5 ~ 1 s
```

不等于：

```text
无人机丢掉 3D 路径。
```

---

# 8. Gate 世界坐标不能只用 yaw 变换

这是当前需要重点检查的一项。

Camera → Body → World 必须使用完整姿态：

```text
roll
pitch
yaw
```

或者直接使用 quaternion。

计算链：

\[
P_B=R_{BC}P_C+t_{BC}
\]

\[
P_W=R_{WB}P_B+t_{WB}
\]

其中：

```text
R_WB
```

必须由完整 quaternion 得到。

不能只使用 yaw。

---

# 9. 为什么 pitch 可能造成“Gate Z 越来越低”

高速飞行或转弯时，无人机会产生 pitch / roll。

前视相机固定在机体上，因此相机光轴也会随姿态变化。

如果坐标转换没有完整补偿姿态：

```text
前方距离
```

可能被错误投影成：

```text
World Z
```

表现为：

```text
飞机加速 / 转弯
↓
Pitch 改变
↓
同一 Gate World Z 漂移
↓
Gate Chain 被错误拉低
↓
无人机继续下降
```

---

# 10. IMU 正式加入 Gate 3D 链路

建议使用：

```text
/airsim_node/drone_1/imu/imu
```

重点使用：

```text
orientation quaternion
angular velocity
```

调试阶段可与：

```text
pose_gt orientation
```

做对照。

重点测试：

> 同一固定 Gate 在无人机转弯、加速、pitch 变化期间，其 World Z 是否基本不变。

---

# 11. Gate Map 更新不要直接覆盖

新观测不能直接：

```text
old_xyz = new_xyz
```

建议使用：

```text
Median
EMA
或 Kalman Filter
```

例如：

\[
G_{new}=(1-\beta)G_{old}+\beta G_{obs}
\]

建议：

```text
远距离：beta 小
近距离：beta 大
```

参考：

```text
beta = 0.1 ~ 0.3
```

---

# 12. 删除“门口停车爬升”策略

旧策略：

```text
Z Error 大
↓
XY = 0
↓
原地爬升
↓
高度恢复
↓
重新向前
```

这一策略从正常控制中删除。

保留完全停车能力的唯一场景：

```text
Emergency
```

例如已经非常接近非法边界，继续前进必然失败。

---

# 13. 改成连续三维速度控制

规划结果必须是：

```text
P(s) = [x(s), y(s), z(s)]
```

控制结果同时存在：

```text
vx
vy
vz
```

上坡行为：

```text
      ↗
    ↗
  ↗
→
```

而不是：

```text
→ → STOP
      ↑
      ↑
→ → →
```

---

# 14. 高度误差只用于连续减速，不用于普通停车

建议第一版：

```text
|z_error| < 0.3 m
→ XY 速度保持

0.3 ~ 0.6 m
→ XY × 0.8

0.6 ~ 1.0 m
→ XY × 0.6

> 1.0 m
→ XY × 0.4
```

仍保持一定前进速度，例如：

```text
最低 2 ~ 4 m/s
```

具体值后续实测。

---

# 15. Z Feedforward 保留，但不触发 Stop

Z 控制仍推荐：

\[
v_z=v_{z,fb}+v_{z,ff}
\]

反馈：

\[
v_{z,fb}=K_z(z-z_{ref})
\]

前馈：

\[
v_{z,ff}=-\frac{dz}{ds}v_s
\]

但是：

```text
Z Error / Z Slope
```

只决定：

```text
速度缩放和 vz
```

不能直接进入：

```text
HOLD_FOR_Z
```

---

# 16. 没有未来 Gate 时，禁止 Z 无限外推

这是当前必须新增的保护。

如果未来仍有 Persistent Gate：

```text
继续正常 3D Path。
```

如果：

```text
未来 Persistent Gate 已用完
+
YOLO 暂时没有发现新的 Gate
```

进入：

```text
NO_FUTURE_GATE
```

此时：

```text
XY：
继续沿 XY Route

Yaw：
继续朝 Route Future Tangent

Z：
停止长期 slope extrapolation
```

---

# 17. Z 外推设置 Horizon

允许短距离预测，例如：

```text
1 ~ 2 s
```

或者：

```text
10 ~ 20 m
```

超过之后：

```text
dz/ds → 0
```

最终：

```text
Hold Last Reliable Z
```

而不是：

```text
按照最后一次下降斜率无限下降。
```

---

# 18. Gate Chain 仍然使用多门前视

维持前一版策略：

```text
G0
G1
G2
G3
```

必须满足：

```text
path_s > current_s
```

且：

```text
G0.s < G1.s < G2.s < G3.s
```

并结合：

```text
双目深度
bbox 尺寸
中心曲线
历史 Track
```

保证这些矩形真的是前方连续 Gate。

---

# 19. Yaw 与 Gate Chain 联动

有可靠 Gate Chain：

```text
Gate Chain
↓
拟合未来中心曲线
↓
取前方 20~30m 曲线切线
↓
yaw_target
```

Gate 暂时不足：

```text
XY Route tangent
↓
yaw_target
```

于是即使 YOLO 短暂丢失：

```text
机头仍然继续看向赛道未来方向。
```

这有利于快速重新捕获 Gate。

---

# 20. YOLO 不直接决定 Gate ID

YOLO 只输出：

```text
Gate Detection
```

Gate ID / Gate 顺序由：

```text
Persistent Track
+
Route Progress
+
Stereo Depth
+
历史 Gate Queue
```

共同确定。

避免转弯时：

```text
画面中 Gate 位置变化
→ YOLO Detection 顺序变化
→ Gate ID 串掉
```

---

# 21. Gate Track 推荐字段

```text
track_id
world_xyz
camera_xyz
path_s
confidence
support
last_seen
sigma_x
sigma_y
sigma_z
hard_anchor
```

其中：

```text
sigma_z
```

特别重要。

高速转弯时如果：

```text
sigma_z 突然变大
```

该 Gate 不应该立即更新 3D Reference。

---

# 22. Gate Z 进入三维路径前的检查

必须同时满足：

```text
1. 多帧稳定
2. Route s 顺序正确
3. World Z 方差合理
4. 与前后 Gate Z 趋势不过分冲突
5. Camera → World 姿态补偿正常
```

任何一项异常：

```text
soft_anchor / suspect
```

而不是：

```text
hard_anchor
```

---

# 23. LiDAR 暂时作为辅助节点预留

可使用：

```text
/airsim_node/drone_1/lidar
```

当前先不让 LiDAR 主导 Gate 识别。

后续用途：

```text
障碍物检测
道路 / 地面距离
前方几何验证
局部避障
YOLO Gate ROI 几何辅助
```

---

# 24. 后视双目暂时不进入主控制

后视双目以后可以用于：

```text
Gate 穿越确认
Recovery
回头重新定位
```

当前主感知链继续保持：

```text
前视 YOLO + Stereo
```

避免一次引入过多变量。

---

# 25. 推荐软件结构

视觉：

```text
rmua_gate_vision/
├── yolo_gate_detector.py
├── stereo_gate_estimator.py
├── gate_world_transform.py
├── gate_tracker.py
└── gate_map.py
```

控制：

```text
route_follower/
├── gate_chain.py
├── reference_3d.py
├── yaw_controller.py
├── z_controller.py
├── speed_scheduler.py
└── route_follower.py
```

以后再接：

```text
local_planner/
obstacle_avoidance/
```

---

# 26. 推荐调试顺序

## Test 1：只验证 Yaw

速度先降到：

```text
3 ~ 4 m/s
```

观察转弯时：

```text
Yaw 是否提前变化；
下一批 Gate 是否更长时间保持在画面前方。
```

---

## Test 2：验证同一 Gate 的 World Z

锁定同一个 Gate Track。

让无人机：

```text
转弯
加速
减速
产生 pitch
```

记录：

```text
Gate World X/Y/Z
roll/pitch/yaw
```

要求：

```text
Gate World Z 基本稳定。
```

如果不稳定：

> **先修坐标变换，不要继续调 Z Controller。**

---

## Test 3：视觉短时丢失

人为停止 YOLO 更新约：

```text
1 s
```

检查：

```text
Persistent Gate Map 是否继续提供未来 Gate；
Z 是否保持合理；
是否还会一直往下外推。
```

---

## Test 4：连续上坡

检查：

```text
无人机边向前边爬升
```

而不是：

```text
门口停车后再上升。
```

---

## Test 5：逐渐恢复高速

顺序：

```text
4 m/s
→ 6 m/s
→ 8 m/s
→ 10 m/s
```

每一级检查：

```text
Yaw 是否跟得上；
Gate Track 是否连续；
Gate World Z 是否稳定；
Z Path 是否连续。
```

---

# 27. 推荐日志

至少记录：

```text
time

roll
pitch
yaw

yaw_target
yaw_error
yaw_rate_cmd

gate_track_id
gate_camera_xyz
gate_world_xyz
gate_world_z_sigma

visible_gate_count
persistent_gate_count
future_gate_count

path_s
z_ref
z_actual
vz_cmd

vision_state
gate_map_state
```

---

# 28. 最值得看的四张图

### 图 1：Yaw

```text
yaw_actual
yaw_target
```

看：

```text
转弯是否提前转头。
```

### 图 2：固定 Gate World Z

```text
gate_world_z
pitch
roll
```

看：

```text
姿态变化是否导致 Gate Z 漂移。
```

### 图 3：Gate 数量

```text
visible_gate_count
persistent_gate_count
future_gate_count
```

看：

```text
视觉短暂丢失时 Gate Map 是否仍然连续。
```

### 图 4：Z

```text
z_ref
z_actual
vz_cmd
```

看：

```text
是否还存在“丢 Gate → Z 一路下降”。
```

---

# 29. 本阶段验收标准

- 转弯时 yaw 会提前朝未来路径方向变化；
- 前视相机不再长期固定看旧方向；
- YOLO 短暂丢 Gate 不会让 3D Path 立刻消失；
- 已确认 Gate 会保存在 Persistent Gate Map；
- Camera → World 使用完整 quaternion；
- 高速 pitch / roll 变化时，同一 Gate World Z 基本稳定；
- 没有可靠未来 Gate 时不会无限向下外推；
- Z 外推存在明确 horizon；
- 上坡时保持连续前进；
- 正常控制不再出现“门口停车再爬升”；
- Gate Chain 和 XY Route 都可以为 Yaw 提供未来方向；
- 后续仍可接 LiDAR / Local Planner 做避障。

---

# 30. 最终控制逻辑

```text
              XY Route
                 │
                 │
YOLO + Stereo    │
      ↓          │
Gate Observation │
      ↓          │
Full Quaternion  │
Transform        │
      ↓          │
Persistent Gate Map
      ↓
Ordered Gate Chain
      │
      ├────────────→ Yaw Preview
      │                 ↓
      │             yaw_rate
      │
      ↓
Continuous 3D Reference
      ↓
Dynamic Look-ahead
      ↓
vx + vy + vz
      ↓
Continuous Speed Scheduler
      ↓
World → Body
      ↓
vel_body_cmd
```

---

# 31. 一句话总结

这一版不再把问题理解成：

```text
“Z 不够就停车补高度”。
```

而是改成：

> **机头提前看向未来赛道，YOLO+双目不断更新 Gate，IMU 保证 Gate 世界坐标不随姿态漂移，Persistent Gate Map 保证短暂丢视觉也不会丢路线，无人机沿连续三维曲线边前进边升降。**

这应该作为下一版高速连续穿 Gate 控制系统的主方向。
