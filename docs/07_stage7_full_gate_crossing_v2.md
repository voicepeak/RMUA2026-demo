# RMUA 2026 Stage 7 v2：全 Gate 连续穿越、强 Gate 对准与避障预留方案

## 0. 本阶段目标

当前系统已经完成：

- XY 赛道路径跟踪；
- OpenCV Gate 检测；
- 双目三角化；
- Gate Camera → Body → World 坐标变换；
- 真实 Gate XYZ 采集；
- Gate 按路径进度 `s` 排序；
- 基础 Gate Manager；
- 部分 Gate 已经能够实际穿越。

现在的核心目标不是继续提高视觉复杂度，而是：

> **让无人机优先对准每一道 Gate 的正中心，并稳定、连续地穿过所有 Gate。**

同时考虑到后续一定会加入障碍物避障，本阶段必须提前预留：

```text
Gate Guidance
与
Obstacle Avoidance
```

之间的仲裁接口。

本阶段：

```text
避障功能先留空
```

但架构上不能把 Gate 吸附直接写死到最终速度输出。

---

# 1. 当前问题重新定义

目前真正的问题有三个：

```text
① Z 轨迹必须跟随真实 Gate 高度变化；
② Gate 对飞行控制的吸附还不够强；
③ 后续避障加入后，不能让 Gate 吸附覆盖避障命令。
```

因此本阶段的设计原则调整为：

> **Route 负责把飞机送到 Gate 附近，Gate 负责精确对准和穿越，未来 Obstacle Avoidance 拥有更高安全优先级。**

---

# 2. 控制优先级

推荐从现在开始明确控制层级：

```text
最高优先级：
Emergency / Safety

第二优先级：
Obstacle Avoidance
（本阶段预留，不实现）

第三优先级：
Gate Alignment / Gate Crossing

第四优先级：
Route Following
```

也就是说未来不允许出现：

```text
Gate Center 在前方
+
障碍物也在前方
+
Gate Attraction 仍然强行让飞机撞过去
```

正确行为应该是：

```text
先避障
↓
重新获得安全通道
↓
再恢复 Gate 捕获
↓
重新对准 Gate
↓
穿门
```

---

# 3. 总体架构

```text
                    XY ROUTE
                       ↓
                Route Follower
                       ↓
                v_route_desired
                       │
                       │
REAL GATE XYZ ─────→ Gate Manager
                       ↓
                  Gate Guidance
                       ↓
                 v_gate_desired
                       │
                       │
LiDAR / Camera ─→ Obstacle Avoidance
                 （当前预留）
                       ↓
                 v_avoid_desired
                       │
                       ↓
                Command Arbiter
                 控制仲裁器
                       ↓
                v_final_world
                       ↓
                World → Body
                       ↓
                 vel_body_cmd
                       ↓
                      UAV
```

关键变化：

> `Route Follower` 和 `Gate Guidance` 都不再直接发布最终速度。

它们只输出：

```text
desired velocity
```

最终速度统一经过：

```text
Command Arbiter
```

决定。

这样以后接避障不需要重写 Gate 控制。

---

# 4. Gate 必须成为局部高优先级目标

当前比赛主要得分来自按顺序穿过 Gate。

所以靠近下一 Gate 后：

```text
Route 只负责把飞机送到附近
```

而：

```text
Gate Center
```

应该逐步成为主要控制目标。

目标不是：

```text
尽量靠近 Gate
```

而是：

> **进入 Gate 前尽可能让飞机位于 Gate 正中心线上。**

---

# 5. Gate 控制状态机

推荐状态：

```text
PATH
 ↓
GATE_CAPTURE
 ↓
GATE_ALIGN
 ↓
GATE_CROSS
 ↓
GATE_EXIT
 ↓
PATH
```

如果穿门失败：

```text
GATE_RECOVER
```

后续加入避障后可能增加：

```text
AVOID
```

但当前只预留。

---

# 6. PATH

距离下一 Gate 较远时：

```text
Route Follower
```

占主导。

主要目标：

```text
沿已有 XY Route 前进
+
跟随 Gate 构造出的 Z Profile
```

此时：

```text
Gate 权重很低或为 0。
```

---

# 7. GATE_CAPTURE

当距离 Gate 小于：

```text
D_capture_start
```

例如：

```text
20 ~ 25 m
```

开始逐渐增加 Gate 控制权。

此时：

```text
Route
+
Gate
```

共同决定期望速度。

推荐：

```text
D_capture_start = 22 m
D_full_control  = 10 m
```

定义：

\[
\alpha_{gate}
=
clamp\left(
\frac{D_{capture}-D}
{D_{capture}-D_{full}},
0,
1
\right)
\]

于是：

```text
D > 22m
alpha = 0

D ≈ 16m
alpha ≈ 0.5

D < 10m
alpha = 1
```

---

# 8. Gate XY 吸附必须更灵敏

以前可能采用：

```text
Route Velocity × (1-alpha)
+
Gate Velocity × alpha
```

但 Gate 权重提升较慢。

现在建议：

```text
Gate 越近
→ Gate XY 增益越大
→ Route 权重越小
```

推荐首轮参数：

```text
>15 m:
gate_k_xy = 0.8 ~ 1.0

10~15 m:
gate_k_xy = 1.2 ~ 1.5

<10 m:
gate_k_xy = 1.8 ~ 2.2
```

因此：

```text
远距离：
动作柔和

近距离：
强力吸向门中心
```

---

# 9. 速度与灵敏度反向变化

Gate 越近：

```text
控制增益越大
```

但：

```text
前进速度越低
```

推荐：

```text
>15m:
speed = 2.0 ~ 2.5 m/s

10~15m:
speed = 1.3 ~ 1.7 m/s

<10m:
speed = 0.8 ~ 1.2 m/s
```

这样：

```text
远处快速靠近
近处慢速精确对准
```

比单纯把 `gate_k` 调很大稳定得多。

---

# 10. GATE_ALIGN：Gate 完全接管

当：

```text
D_gate < D_full_control
```

例如：

```text
< 10 m
```

进入：

```text
GATE_ALIGN
```

此时：

```text
alpha_gate = 1
```

即：

> **Route 不再决定当前局部目标。**

控制目标直接变成：

```text
Gate Center XYZ
```

但不能简单把 Gate Center 当成停车点。

真正需要消除的是：

```text
门平面内误差
```

---

# 11. 将 Gate 误差拆成两部分

设：

```text
Gate Center = G
Gate Normal = n
Drone Position = P
```

定义：

\[
r=P-G
\]

沿 Gate 法向方向的误差：

\[
e_n=r\cdot n
\]

门平面内误差：

\[
e_{plane}=r-e_n n
\]

真正要强力压小的是：

```text
e_plane
```

因为它表示：

```text
偏左 / 偏右
+
偏高 / 偏低
```

也就是：

> 飞机是否真正对着门正中心。

---

# 12. GATE_ALIGN 的目标

进入 ALIGN 后：

```text
目标 1：
横向误差足够小

目标 2：
Z 误差足够小

目标 3：
Gate Center 在当前飞行方向附近
```

建议初始阈值：

```text
align_xy_tol = 0.5 m
align_z_tol  = 0.3 ~ 0.4 m
```

如果没有达到：

```text
不进入高速 CROSS。
```

---

# 13. Gate Z 同样采用强吸附

正常路径阶段：

```text
z_ref = z_profile(s)
```

进入 Gate Capture：

```text
z_ref
=
(1-alpha_z) * z_profile
+
alpha_z * z_gate
```

进入 Gate Align：

```text
z_ref = z_gate
```

也就是说：

```text
最后 10m
```

高度不再参考普通赛道曲线，而直接以：

```text
Gate Center Z
```

为准。

---

# 14. Z Profile 仍然必须保留

虽然 Gate 附近会强制：

```text
z = gate.z
```

但不能等靠近 Gate 才调整高度。

真实 Gate 数据中存在明显变化。

例如：

```text
Gate 6:
z ≈ -7.18

Gate 7:
z ≈ -10.64
```

只有约 11m 路径距离。

因此必须继续使用：

```text
Gate Anchor
→ z_profile(s)
```

提前升降。

Gate Align 只是：

```text
最后精确收敛。
```

---

# 15. Z Profile

相邻 Gate：

```text
A = (sA, zA)
B = (sB, zB)
```

当前：

```text
s
```

计算：

\[
t=
\frac{s-s_A}{s_B-s_A}
\]

使用：

\[
h(t)=3t^2-2t^3
\]

得到：

\[
z_{profile}(s)
=
z_A+h(t)(z_B-z_A)
\]

这样：

```text
上一 Gate 穿过后
↓
马上开始向下一 Gate 高度调整
```

---

# 16. GATE_CROSS：不要追 Gate Center 到速度为 0

如果继续采用普通 P 控制：

```text
目标 = Gate Center
```

则：

```text
越靠近中心
→ 速度越小
```

可能发生：

```text
停在门中间。
```

因此进入：

```text
GATE_CROSS
```

以后控制方式必须改变。

---

# 17. Gate Crossing Velocity

设：

```text
Gate Normal = n
```

定义固定穿门速度：

```text
v_cross
```

例如：

```text
1.0 ~ 1.5 m/s
```

最终控制：

\[
v
=
v_{cross}n
-
K_{center}e_{plane}
\]

含义：

```text
v_cross * n
→ 一直往门后飞

-K_center * e_plane
→ 偏离门中心就拉回来
```

这样飞机会：

> **边保持正中心，边穿过去。**

而不是：

> 飞到中心然后停住。

---

# 18. GATE_EXIT

穿过 Gate Plane 后不要立即：

```text
gate_idx++
```

继续沿：

```text
Gate Normal
```

向前飞：

```text
2 ~ 3 m
```

直到：

```text
Exit Confirmed
```

才切换下一 Gate。

建议：

```text
exit_distance = 2.5 ~ 3.0 m
```

这样能避免：

```text
刚穿平面
↓
控制目标突然切下一门
↓
侧向拉走
```

---

# 19. Gate Pass 判定

推荐采用三条件：

```text
① Plane Crossed
② Inside Gate Aperture
③ Exit Confirmed
```

不能只判断：

```text
距离 Gate Center 很近
```

---

# 20. Gate Miss Recovery

如果：

```text
Gate Plane 已经被穿过
```

但：

```text
XY / Z 不在门洞允许范围
```

则：

```text
不能认为 PASSED。
```

进入：

```text
GATE_RECOVER
```

流程：

```text
穿门失败
 ↓
降低速度
 ↓
回到 Approach Point
 ↓
重新 ALIGN
 ↓
重新 CROSS
```

建议：

```text
max_gate_retry = 3
```

因为 Gate 必须按顺序通过。

---

# 21. Gate Approach / Exit 点

利用：

```text
Gate Center = G
Gate Normal = n
```

生成：

\[
P_{approach}
=
G-d_a n
\]

\[
P_{exit}
=
G+d_e n
\]

建议：

```text
d_a = 3 ~ 5 m
d_e = 2 ~ 3 m
```

---

# 22. 动态 Z 安全包络继续保留

旧的：

```text
z_safe_min = -4.9
```

不能继续作为全地图约束。

必须使用：

```text
z_center(s)
```

附近的局部安全 Corridor。

例如：

```text
corridor_half_height = 1.2 ~ 1.8 m
```

则：

```text
z_ceiling(s)
z_floor(s)
```

随着赛道一起变化。

---

# 23. Gate 模式中的 Z Corridor

进入 Gate Align / Cross：

```text
Dynamic Corridor
```

进一步收紧到：

```text
Gate Aperture Z
```

如果目前还没有 Gate 上下边缘真实尺寸：

```text
先用 gate_z ± gate_vertical_margin
```

以后视觉增加：

```text
Gate Height
```

后改成真实门洞高度。

---

# 24. 预留避障能力：必须现在加入接口，但不实现

这是本次更新最重要的架构预留。

未来可能出现：

```text
Gate Center
       ↓
强吸附

但同时：

Gate 前有动态悬浮车
```

显然不能：

```text
因为 Gate 权重 = 1
就继续撞向 Gate。
```

所以：

> **Gate Guidance 不允许直接输出最终 `vel_body_cmd`。**

---

# 25. 预留 Command Arbiter

增加：

```text
Command Arbiter
```

统一接受：

```text
v_route_desired
v_gate_desired
v_avoid_desired
```

当前阶段：

```text
v_avoid_desired = None
avoidance_active = false
```

因此：

```text
Arbiter
```

暂时等价于：

```text
Route / Gate 控制切换。
```

未来避障只需接入：

```text
Obstacle Avoidance
→ Arbiter
```

---

# 26. 当前 Arbiter 逻辑

当前：

```python
if gate_state in [ALIGN, CROSS, EXIT]:
    v_desired = v_gate

elif gate_state == CAPTURE:
    v_desired = blend(v_route, v_gate)

else:
    v_desired = v_route
```

未来扩展成：

```python
if emergency:
    v_final = v_emergency

elif avoidance_active:
    v_final = v_avoid

elif gate_state in [ALIGN, CROSS, EXIT]:
    v_final = v_gate

elif gate_state == CAPTURE:
    v_final = blend(v_route, v_gate)

else:
    v_final = v_route
```

现在先把：

```text
avoidance_active
```

接口留着。

---

# 27. 未来避障时 Gate 不应该被完全丢弃

虽然：

```text
Obstacle Avoidance
```

拥有更高安全优先级，

但未来也不应该：

```text
一避障就忘掉 Gate。
```

建议以后设计成：

```text
Gate = Mission Objective
Obstacle = Hard Constraint
```

也就是：

```text
“尽可能去 Gate”
```

但：

```text
“不能撞”
```

---

# 28. 当前只预留以下接口

现在新增数据结构：

```text
AvoidanceCommand
```

暂时只定义：

```text
active: bool
velocity_world: [vx, vy, vz]
confidence:
timestamp:
```

当前：

```text
active = false
```

不实现任何 LiDAR / 动态障碍逻辑。

---

# 29. Gate State 在避障后必须可恢复

未来如果：

```text
GATE_ALIGN
```

过程中进入：

```text
AVOID
```

避障结束后不能直接：

```text
继续 CROSS。
```

应该：

```text
AVOID
 ↓
重新评估 Gate
 ↓
回到 GATE_CAPTURE / GATE_ALIGN
 ↓
重新对准
 ↓
CROSS
```

所以当前 Gate 状态机最好预留：

```text
previous_gate_state
```

以及：

```text
resume_gate_alignment()
```

接口。

---

# 30. 推荐状态机

当前：

```text
PATH
 ↓
CAPTURE
 ↓
ALIGN
 ↓
CROSS
 ↓
EXIT
 ↓
PATH
```

失败：

```text
RECOVER
```

未来：

```text
任何状态
 ↓
AVOID
 ↓
CAPTURE / ALIGN
```

而不是：

```text
AVOID
↓
直接恢复原速度
```

---

# 31. Gate Detector 后续深度学习升级不影响本方案

当前：

```text
OpenCV Gate Detector
```

未来：

```text
Deep Learning Gate Detector
```

只改变：

```text
Gate Observation
```

来源。

保持统一输出：

```text
Gate Center XYZ
Gate Normal
Gate Width
Gate Height
Confidence
```

后面的：

```text
Gate Manager
Z Profile
Gate Guidance
Command Arbiter
```

全部不变。

---

# 32. 建议的新软件结构

```text
route_follower/
├── route_follower.py
├── altitude_profile.py
├── gate_manager.py
├── gate_guidance.py
├── command_arbiter.py
└── avoidance_interface.py
```

其中：

```text
avoidance_interface.py
```

当前可以只有：

```python
class AvoidanceCommand:
    active = False
```

后面再真正实现。

---

# 33. Gate Guidance 模块职责

输入：

```text
current_pose
current_velocity
next_gate
gate_state
z_profile
```

输出：

```text
v_gate_world
gate_state
gate_error_xy
gate_error_z
```

不要直接发布 ROS 控制命令。

---

# 34. Route Follower 模块职责

输入：

```text
current_pose
XY route
```

输出：

```text
v_route_world
path_progress
```

同样不直接发布最终控制。

---

# 35. Command Arbiter 模块职责

输入：

```text
v_route_world
v_gate_world
v_avoid_world
gate_state
avoidance_active
safety_state
```

输出：

```text
v_final_world
```

只有这里才进入：

```text
World → Body
↓
vel_body_cmd
```

---

# 36. 推荐首轮参数

```text
Gate Capture Start:
22 m

Gate Full Control:
10 m
```

```text
gate_k_xy_far:
0.9

gate_k_xy_mid:
1.4

gate_k_xy_near:
2.0
```

```text
Gate Speed Far:
2.0 m/s

Gate Speed Mid:
1.4 m/s

Gate Align Speed:
0.8 ~ 1.0 m/s

Gate Cross Speed:
1.2 m/s
```

```text
Align XY Tol:
0.5 m

Align Z Tol:
0.35 m
```

```text
Approach Distance:
4 m

Exit Distance:
3 m
```

```text
Max Retry:
3
```

这些为首轮测试值。

---

# 37. Gate 穿越测试重点

## Test A：单 Gate

只验证：

```text
远处捕获
↓
靠近中心
↓
ALIGN
↓
正中心 CROSS
↓
EXIT
```

要求：

```text
不要考虑速度成绩。
```

---

## Test B：两个连续 Gate

验证：

```text
Gate 1 EXIT
↓
Z Profile 自动朝 Gate 2 调整
↓
Gate 2 CAPTURE
```

---

## Test C：明显高度变化 Gate

重点测试：

```text
Gate 6 → Gate 7
```

观察：

```text
是否提前调整 Z
是否在 Gate 前已经基本居中
```

---

## Test D：人为制造偏差

飞到 Gate 前故意：

```text
偏左
偏右
偏高
偏低
```

观察：

```text
Gate 吸附是否能够迅速拉回中心。
```

---

# 38. 建议日志

每秒打印：

```text
STATE
gate_idx
distance_to_gate

alpha_gate

gate_center
gate_normal

plane_error_xy
z_error

gate_k_xy
gate_speed

v_route
v_gate
v_final

avoidance_active
```

当前：

```text
avoidance_active = false
```

但日志字段从现在就保留。

---

# 39. 当前阶段验收标准

认为当前 Stage 7 完成，需要：

- Gate 在 20m 左右开始明显影响轨迹；
- 10m 内 Gate 完全接管；
- 飞机明显优先朝 Gate 正中心靠近；
- ALIGN 后 XY / Z 能稳定进入容差；
- CROSS 时不会停在 Gate Center；
- 能沿 Gate Normal 穿过去；
- EXIT 后才切换下一 Gate；
- 穿门失败能够 RECOVER；
- 动态 Z Profile 正常工作；
- 全局固定 `z_safe_min=-4.9` 已退出主控制；
- Command Arbiter 已建立；
- 避障接口已经预留；
- 当前即使不实现避障，也不会影响现有控制；
- 后续加入避障不需要修改 Route / Gate Guidance 的主体结构。

---

# 40. 当前最终架构

```text
                         MISSION
                            ↓
                       Next Gate
                            ↓
                      Gate Manager
                            │
              ┌─────────────┴─────────────┐
              ↓                           ↓
          XY Route                   Gate XYZ
              ↓                           ↓
        Route Follower              Gate Guidance
              ↓                           ↓
        v_route_world              v_gate_world
              │                           │
              └─────────────┬─────────────┘
                            │
                 Obstacle Avoidance
                  （当前留空接口）
                            │
                            ↓
                     Command Arbiter
                            ↓
                     v_final_world
                            ↓
                      World → Body
                            ↓
                      vel_body_cmd
                            ↓
                           UAV
```

---

# 41. 当前最终策略

本阶段比赛策略明确调整为：

```text
远离 Gate：
沿路线快速推进

接近 Gate：
逐渐增强 Gate 吸附

10m 内：
Gate Center 完全接管

门前：
先把 XY + Z 对准正中心

穿门：
保持中心修正
+
沿 Gate Normal 主动向前

穿完：
继续 Exit 一段距离

确认：
再切下一 Gate
```

而不是：

```text
沿 Route 擦着 Gate 边缘过去。
```

当前优先级是：

> **Gate Pass Rate > 轨迹平滑 > 速度。**

---

# 42. 避障部分当前明确留空

本阶段不实现：

```text
LiDAR Obstacle Detection
Dynamic Obstacle Tracking
Local Planner
Velocity Obstacle
MPC
EGO-Planner
```

只保留：

```text
ObstacleAvoidanceCommand
Command Arbiter
AVOID state hook
```

未来讨论避障时再决定：

```text
如何在避障和 Gate Mission Objective 之间重新规划。
```

这样现在不会把问题复杂化，也不会让后续功能把当前 Gate 逻辑推翻。

---

# 43. 一句话总结

当前方案从：

```text
“路线带着飞机穿门”
```

升级成：

```text
“路线把飞机送到门附近，
Gate Center 接管并把飞机吸到正中心，
沿 Gate Normal 穿过去。”
```

同时预留：

```text
“未来遇到障碍时，
安全避障可以暂时覆盖 Gate 吸附，
避障完成后再重新捕获 Gate。”
```

这应该作为下一阶段连续穿门的主控制框架。
