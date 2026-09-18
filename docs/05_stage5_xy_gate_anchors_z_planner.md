# RMUA 2026：基于现有 XY 路径与 Gate 锚点的 Z 轴规划方案

## 0. 当前状态

目前系统可以认为已经具备：

```text
① XY 路径
② XY Path Follower
③ Look-ahead
④ Cross-track Correction
⑤ Boundary Guard
⑥ vel_body_cmd 控制
⑦ Gate Manager / Gate 参考机制
```

当前主要缺失的是：

> **一条真正随赛道变化的 Z 轴参考轨迹。**

也就是说，现在已有：

```text
x_ref(s)
y_ref(s)
```

但还没有可靠的：

```text
z_ref(s)
```

因此当前阶段的目标不是重新做一套完整三维路径规划，而是在现有 XY 路径基础上：

> **利用 Gate 作为高度关键锚点，生成沿赛道连续变化的 Z Reference。**

---

# 1. 为什么 Gate 应该成为 Z 轴核心参考

RMUA 的检测门不是普通视觉目标。

它同时承担：

```text
路线顺序约束
+
计分约束
+
空间位置约束
```

因此对于比赛来说：

> **真正重要的不是“飞在一个看起来合理的高度”，而是能够按顺序从 Gate 内部穿过去。**

只要：

```text
XY 路径正确
```

但：

```text
Z 高度错误
```

就可能：

```text
从 Gate 上方飞过
从 Gate 下方飞过
```

最终：

```text
Gate 不触发
```

所以 Gate 应当成为 Z 轴中的：

> **强约束锚点（Altitude Anchor）。**

---

# 2. 当前阶段不需要完整 3D 地图

目前已经有一条可用的 XY Route：

```text
P0
 ↓
P1
 ↓
P2
 ↓
...
 ↓
Goal
```

当前不需要把整个地图重建成：

```text
完整三维可飞空间
```

而只需要在这条 XY 路线上再增加一个函数：

```text
z_ref = f(s)
```

其中：

```text
s
```

表示飞机沿当前 XY 路线已经前进到哪里。

整个系统就从：

```text
XY Path
```

升级为：

```text
XY Path
+
Altitude Profile
```

---

# 3. 核心思想：Gate 提供 Z Anchor

假设路线中有：

```text
Gate 1
Gate 2
Gate 3
...
```

每个 Gate 提供：

```text
Gate_i = [x_i, y_i, z_i]
```

将 Gate 投影到当前 XY Path 上，可以得到它对应的路径进度：

```text
s_i
```

于是每一个 Gate 都变成一个：

```text
(s_i, z_i)
```

高度锚点。

例如：

```text
Gate 1:
s = 80 m
z = -2.2

Gate 2:
s = 150 m
z = -3.0

Gate 3:
s = 230 m
z = -2.5
```

然后构造：

```text
z_ref(s)
```

即可。

---

# 4. 系统总体结构

```text
                XY Reference Path
                       ↓
               Path Progress s
                       │
          ┌────────────┴────────────┐
          │                         │
          ↓                         ↓
      XY Follower              Gate Manager
          │                         │
          │                  Gate XYZ / Anchor
          │                         │
          │                         ↓
          │                  Altitude Profile
          │                         │
          ↓                         ↓
   vx_world, vy_world            z_ref(s)
          │                         │
          │                         ↓
          │                    Z Controller
          │                         │
          │                       vz
          └────────────┬────────────┘
                       ↓
              Velocity Command
                       ↓
               World → Body
                       ↓
                vel_body_cmd
                       ↓
                      UAV
```

---

# 5. Gate 当前怎么使用

当前阶段不要求深度学习 Gate Detector。

现在只要求 Gate Manager 能得到：

```text
Gate Center XYZ
```

这些 XYZ 可以来自：

```text
人工记录
```

或者：

```text
现有 Gate 配置
```

后面真正加入深度学习以后，只需要把：

```text
Gate Center XYZ 的来源
```

替换掉。

也就是说：

```text
当前：
人工/配置 Gate XYZ
        ↓
Gate Manager
```

以后：

```text
Camera
 ↓
Deep Learning Detector
 ↓
Depth / Stereo
 ↓
Gate Center XYZ
 ↓
Gate Manager
```

后面的 Z Planner 不需要重写。

---

# 6. Gate 不只是普通 Waypoint

普通道路路径点：

```text
ROAD_GUIDE
```

主要作用：

```text
让飞机留在赛道中
```

不要求非常精确经过。

Gate：

```text
GATE_ANCHOR
```

则必须：

```text
XY 接近 Gate Center
+
Z 接近 Gate Center
+
正确穿过 Gate Plane
```

所以 Gate 优先级应该高于普通 Route Point。

---

# 7. 最简单的 Z Profile

假设相邻两个 Gate：

```text
Gate A
(sA, zA)

Gate B
(sB, zB)
```

飞机当前路径进度：

```text
s
```

其中：

```text
sA <= s <= sB
```

定义：

\[
t=
\frac{s-s_A}
{s_B-s_A}
\]

然后：

\[
z_{ref}
=
(1-t)z_A+t z_B
\]

也就是：

```text
Gate A
zA
 \
  \
   \
    \
     Gate B
     zB
```

飞机在两个 Gate 之间逐渐改变高度。

---

# 8. 不要到 Gate 前才突然变高度

错误方式：

```text
一路保持 z=-2
↓
距离 Gate 2m
↓
突然改成 z=-4
```

这样会造成：

```text
垂直速度突然变大
飞机姿态波动
Gate 对准时间不足
```

正确方式：

```text
通过上一 Gate 后
↓
马上开始朝下一 Gate 高度平滑调整
↓
到 Gate 前已经基本达到正确 Z
```

因此 Gate 不只是一个：

```text
最后一刻修正点
```

而是：

> **下一段 Z 轨迹的目标锚点。**

---

# 9. 推荐使用 Smoothstep 插值

第一版线性插值可以直接使用。

稳定后建议改成：

\[
h(t)=3t^2-2t^3
\]

然后：

\[
z_{ref}
=
z_A+h(t)(z_B-z_A)
\]

这样：

```text
刚离开 Gate A
Z 变化较慢

中间段
Z 正常变化

靠近 Gate B
再次减缓
```

比纯线性更适合无人机。

---

# 10. Start 到第一个 Gate

第一个 Gate 前没有上一 Gate。

所以增加：

```text
START_ANCHOR
```

其坐标：

```text
s = 0
z = 起始实际高度
```

形成：

```text
Start
 ↓
Gate 1
 ↓
Gate 2
 ↓
Gate 3
 ...
```

对应：

```text
(start_s, start_z)
(gate1_s, gate1_z)
(gate2_s, gate2_z)
...
```

这样从起飞开始就有连续的 Z Reference。

---

# 11. 最后一个 Gate 到 Goal

最后一个 Gate 以后可以增加：

```text
GOAL_ANCHOR
```

如果 Goal 没有明确高度需求：

```text
Goal Z = 最后 Gate Z
```

先保持高度即可。

后续如果终点有实际三维高度要求，再加入：

```text
goal_z
```

---

# 12. Gate Approach 模式仍然需要保留

虽然：

```text
z_ref(s)
```

会提前把飞机带到 Gate 高度，

但 Gate 附近仍然建议进入：

```text
GATE_APPROACH
```

因为 Gate 是计分关键点。

状态机：

```text
PATH_FOLLOWING
      ↓
接近下一 Gate
      ↓
GATE_APPROACH
      ↓
GATE_CROSSING
      ↓
GATE_EXIT
      ↓
下一 Gate
```

---

# 13. PATH_FOLLOWING 模式

距离下一 Gate 较远：

```text
> D_gate
```

例如：

```text
10 ~ 20 m
```

主要使用：

```text
XY Path Follower
+
z_ref(s)
```

让无人机自然沿路径前进。

---

# 14. GATE_APPROACH 模式

接近 Gate 后：

```text
XY Target
```

从普通 Look-ahead Point 逐渐向：

```text
Gate Center XY
```

收紧。

同时：

```text
Z Target
```

逐渐强制靠近：

```text
Gate Center Z
```

也就是说：

```text
Route 提供大方向
Gate 提供最终精确约束
```

---

# 15. Gate Z Blend

距离 Gate 较远：

```text
z_ref = z_route
```

靠近 Gate：

```text
z_ref
=
(1-alpha) * z_route
+
alpha * z_gate
```

其中：

```text
alpha = 0
远离 Gate

alpha → 1
接近 Gate
```

例如：

```text
距离 > 15m
alpha = 0

15m → 5m
alpha 从 0 → 1

距离 < 5m
alpha = 1
```

这样能保证：

```text
正常情况下按平滑 Z Profile 飞
```

同时：

```text
真正穿 Gate 前精确锁定 Gate Z
```

---

# 16. Gate 三点模型

每个 Gate 最好不要只有：

```text
Gate Center
```

而是：

```text
Approach Point
      ↓
Gate Center
      ↓
Exit Point
```

示意：

```text
Approach      Gate        Exit
   ● ----------□---------- ●
```

三个点高度均以：

```text
Gate Z
```

为核心。

这样飞机不会：

```text
飞到门中央后开始减速
```

而是：

```text
提前对准
↓
穿门
↓
继续向前
```

---

# 17. 为什么 Gate 是“高分必要约束”

当前路线规划评价不能只看：

```text
飞了多远
```

还要看：

```text
穿了多少 Gate
```

所以路径规划的优先级应该是：

```text
第一优先级
不出界

第二优先级
按顺序穿 Gate

第三优先级
尽可能持续向前

第四优先级
提高速度
```

Z Planner 必须围绕：

```text
Gate Pass Rate
```

设计，而不是单纯追求高度平滑。

---

# 18. Z Controller

已经得到：

```text
z_ref(s)
```

以后，根据当前：

```text
z_actual = pose_gt.z
```

产生：

```text
vz
```

注意当前模拟器中实测约定为：

```text
pose_gt.z：
NED，向下为正

vel_body_cmd.vz：
向上为正
```

因此当前仓库中适合使用：

\[
e_z=z_{actual}-z_{ref}
\]

然后：

\[
v_z=K_z e_z
\]

再限幅：

\[
|v_z|\leq v_{z,max}
\]

---

# 19. 推荐参数初值

仅作为开发起点：

```text
Kz:
0.3 ~ 0.6
```

```text
vz_max:
0.5 ~ 1.0 m/s
```

```text
Gate Approach Distance:
10 ~ 20 m
```

```text
Gate Precision Distance:
约 5 m
```

```text
Gate Crossing Speed:
1 ~ 2 m/s
```

实际以模拟器测试结果调整。

---

# 20. 增加 Z Reference Rate Limit

即使 Gate 高度相差较大，也不要让：

```text
z_ref
```

每个控制周期变化过快。

限制：

```text
|dz_ref/dt| <= z_ref_rate_max
```

例如：

```text
z_ref 每秒最多改变 0.5 ~ 1 m
```

避免：

```text
Gate 数据突然跳变
```

导致：

```text
vz 瞬间打满
```

---

# 21. Gate 数据异常保护

Gate 是重要参考，但不能盲目信任异常数据。

如果：

```text
|z_gate - z_current|
```

突然出现极大值，例如：

```text
几十米
```

则认为：

```text
Gate Z 数据异常
```

这一帧不参与 Z Reference。

逻辑：

```python
if abs(z_gate - z_current) > gate_z_max_jump:
    ignore_gate_z = True
```

这对于当前占位数据尤其重要。

---

# 22. 当前 Gate 数据必须分成两种状态

建议 Gate 配置增加：

```text
valid
```

字段。

例如：

```yaml
- id: 1
  x: ...
  y: ...
  z: ...
  valid: true
```

如果 Gate 尚未实测：

```yaml
valid: false
```

那么它：

```text
不能参与 Z Profile
```

这样可以防止 placeholder Gate Z 把飞机带到错误高度。

---

# 23. 深度学习 Gate Detector 后续怎么接

以后深度学习模块只负责输出：

```text
Gate Observation
```

例如：

```text
gate_id
center_u
center_v
confidence
```

再通过：

```text
Stereo / Depth
```

获得：

```text
gate_depth
```

然后：

```text
Camera XYZ
↓
TF
↓
World XYZ
```

最终送给：

```text
Gate Manager
```

Gate Manager 更新：

```text
Gate Anchor XYZ
```

后面的：

```text
Altitude Profile
Z Controller
Gate Approach
Gate Crossing
```

都不需要修改。

---

# 24. 因此现在和未来的接口应该统一

现在：

```text
人工 Gate XYZ
      ↓
Gate Manager
```

未来：

```text
Deep Learning
      ↓
Gate XYZ
      ↓
Gate Manager
```

后面统一：

```text
Gate Manager
      ↓
Altitude Planner
```

这样当前工作不会浪费。

---

# 25. 推荐代码模块

当前新增一个：

```text
altitude_planner.py
```

职责：

```text
读取 path progress
读取 gate anchors
生成 z_ref
```

不要让：

```text
route_follower.py
```

同时负责所有 Z 逻辑。

推荐：

```text
Route Follower
   ↓
path_progress

Gate Manager
   ↓
gate anchors

两者
   ↓
Altitude Planner
   ↓
z_ref
```

然后：

```text
Z Controller
↓
vz
```

---

# 26. Altitude Planner 输入输出

输入：

```text
current_path_progress
next_gate
gate_list
current_z
route_state
```

输出：

```text
z_ref
next_gate_z
gate_z_blend_alpha
altitude_mode
```

其中：

```text
altitude_mode
```

可以是：

```text
ROUTE
GATE_APPROACH
GATE_CROSSING
GATE_EXIT
```

---

# 27. 核心伪代码

```python
s = path_progress

gate_a = previous_valid_gate(s)
gate_b = next_valid_gate(s)

if gate_a is None:
    gate_a = start_anchor

if gate_b is None:
    gate_b = goal_anchor

# --------------------------------
# 1. 根据两个 Anchor 生成基础 Z
# --------------------------------

t = (
    (s - gate_a.s)
    /
    (gate_b.s - gate_a.s)
)

t = clamp(t, 0.0, 1.0)

smooth_t = 3*t*t - 2*t*t*t

z_route = (
    gate_a.z
    +
    smooth_t * (gate_b.z - gate_a.z)
)

# --------------------------------
# 2. 接近 Gate 时加强 Gate 约束
# --------------------------------

distance_to_gate = distance_xy(
    current_position,
    gate_b.position
)

alpha = gate_blend(distance_to_gate)

z_ref = (
    (1-alpha) * z_route
    +
    alpha * gate_b.z
)

# --------------------------------
# 3. Z Reference 变化率限制
# --------------------------------

z_ref = rate_limit(
    previous_z_ref,
    z_ref,
    max_rate
)

# --------------------------------
# 4. Z Controller
# --------------------------------

z_error = current_z - z_ref

vz = Kz * z_error

vz = clamp(
    vz,
    -vz_max,
    vz_max
)
```

---

# 28. 如果两个 Gate 中间距离很远怎么办

不用额外猜高度。

先采用：

```text
Gate A Z
↓
平滑插值
↓
Gate B Z
```

因为对于比赛目标来说：

```text
最终需要正确经过 Gate B
```

最重要。

如果后续发现两 Gate 之间：

```text
赛道本身还有明显上下起伏
```

再增加：

```text
ALTITUDE_GUIDE Anchor
```

即可。

例如：

```yaml
- type: ALTITUDE_GUIDE
  s: ...
  z: ...
```

它不计分，只帮助构造更真实的 Z Profile。

---

# 29. 数据层级

建议将 Z Anchor 分成：

```text
Level 1：
GATE
最高优先级

Level 2：
ALTITUDE_GUIDE
用于赛道中间高度调整

Level 3：
START / GOAL
边界条件
```

如果冲突：

```text
Gate > Altitude Guide > Default
```

---

# 30. 当前阶段不需要做的事情

先不要：

```text
完整 3D A*
```

不要：

```text
3D Occupancy Planning
```

不要：

```text
深度学习 Gate Detector
```

不要：

```text
复杂 MPC 高度控制
```

不要：

```text
全地图三维重建
```

当前只做：

```text
现有 XY Route
+
Gate Z Anchors
+
Altitude Profile
+
Z Controller
```

这是最小增量方案。

---

# 31. 开发顺序

## Step 1

保留现有稳定 XY Path。

不要改 XY Controller。

---

## Step 2

整理已有 Gate：

```text
Gate ID
Gate X
Gate Y
Gate Z
Valid
```

只允许真实 Gate：

```text
valid=true
```

参与 Z Planner。

---

## Step 3

把每个 Gate 投影到 XY Path。

得到：

```text
Gate ID
Path Progress s
Gate Z
```

---

## Step 4

生成：

```text
z_ref(s)
```

先使用线性插值。

---

## Step 5

接入现有 Z P Controller。

验证：

```text
Z 能随 s 改变
```

而不是：

```text
固定高度
```

---

## Step 6

改用：

```text
Smoothstep
```

让 Z 变化更平滑。

---

## Step 7

加入：

```text
Gate Approach
```

靠近 Gate 后：

```text
z_ref → Gate Z
```

---

## Step 8

加入：

```text
Approach → Center → Exit
```

确保真正穿门。

---

## Step 9

连续测试多 Gate。

重点看：

```text
Gate Pass Rate
```

---

## Step 10

最后再接：

```text
Deep Learning Gate Detector
```

替换 Gate XYZ 来源。

---

# 32. 本阶段验收标准

本阶段完成的标准：

- XY 路径仍然稳定；
- Z 不再使用固定常数；
- 能计算当前 XY Path Progress；
- 已建立有效 Gate Anchor 列表；
- 每个 Gate 有对应 `s_i` 和 `z_i`；
- 能根据相邻 Gate 生成连续 `z_ref(s)`；
- Z 能随路线平滑升降；
- 靠近 Gate 后可以精确收敛到 Gate Z；
- Gate Z 异常不会直接控制飞机；
- 可以连续穿过多个 Gate；
- 穿 Gate 后能恢复正常路径跟踪；
- 后续深度学习识别模块只需提供 Gate XYZ，不需要改 Z Planner。

---

# 33. 最终架构

```text
                     XY PATH
                        ↓
                 Path Progress s
                        │
       ┌────────────────┴────────────────┐
       │                                 │
       ↓                                 ↓
 XY Path Follower                    Gate Manager
       │                                 │
       │                             Gate XYZ
       │                                 │
       │                          Gate Anchor(s,z)
       │                                 │
       │                                 ↓
       │                          Altitude Planner
       │                                 │
       │                              z_ref(s)
       │                                 │
       │                                 ↓
       │                            Z Controller
       │                                 │
       ↓                                 ↓
 vx_world / vy_world                    vz
       │                                 │
       └───────────────┬─────────────────┘
                       ↓
                Velocity Command
                       ↓
                 World → Body
                       ↓
                  vel_body_cmd
                       ↓
                      UAV
```

未来只在：

```text
Gate Manager
```

前增加：

```text
Camera
↓
Deep Learning
↓
Stereo / Depth
↓
Gate XYZ
```

即可。

---

# 34. 当前最重要的结论

当前系统不需要立刻获得完整三维地图。

已经有：

```text
稳定 XY Path
```

那么 Z 轴最经济的升级方式就是：

> **利用 Gate 作为高度锚点，在现有 XY Path 上建立一条一维高度曲线 `z_ref(s)`。**

即：

```text
XY Path
+
Gate XYZ
=
当前阶段的 3D Route
```

其中 Gate 是计分必经点，因此：

```text
正常赛道：
平滑插值

靠近 Gate：
Gate 优先

穿过 Gate：
保持 Gate Center 高度

离开 Gate：
转向下一 Gate 的高度
```

后面深度学习的任务只是：

> **把 Gate XYZ 获取得更自动、更准确。**

而不会改变当前规划框架。
