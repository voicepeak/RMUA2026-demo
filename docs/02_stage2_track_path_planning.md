# RMUA 2026 第二阶段：赛道约束下的路径规划方案

## 0. 我们刚刚完成了什么

上一阶段的目标是做出一个最小可运行的 `Start → Goal` 闭环。

已经确定的基本控制链路为：

```text
pose_gt
  ↓
当前位置

end_goal
  ↓
目标位置

当前位置 + 目标位置
  ↓
位置误差
  ↓
P 控制
  ↓
世界坐标系期望速度
  ↓
世界坐标系 → Body 坐标系
  ↓
vel_body_cmd
  ↓
无人机运动
```

也就是说，目前的算法本质上是：

> 无论目标在哪里，都沿着“当前位置到终点”的方向直接飞过去。

这个方案能够证明：

- ROS 数据可以正常获取；
- `pose_gt` 可以提供当前位置；
- `end_goal` 可以提供当前路径终点；
- P 控制可以将位置误差转成速度；
- 世界坐标系速度可以转换为 Body 坐标系速度；
- `vel_body_cmd` 可以驱动无人机运动；
- 整个基本运动闭环已经建立。

因此，当前主要矛盾已经不再是“飞机会不会动”，而是：

> **飞机应该沿什么路线运动。**

---

# 1. 当前出现的问题

目前控制逻辑类似：

```text
当前位置 P
    ↓

直接指向 end_goal G

    ↓

沿 PG 直线飞行
```

问题是 RMUA 的比赛路线并不是“起点到终点之间任意飞行”。

官方规则规定，每一段路径均为：

```text
起点
 ↓
对应道路
 ↓
中央交通枢纽
 ↓
目标道路
 ↓
终点
```

例如：

```text
1 → 3
```

真正的合法路线应近似理解为：

```text
1号点
  ↓
1号道路
  ↓
中央交通枢纽
  ↓
3号道路
  ↓
3号点
```

而不是：

```text
1号点
  ─────────→
        3号点
```

如果直接从 1 号点朝 3 号终点做直线追踪，这条直线很可能穿出道路区域。

这就是目前飞机飞出边界的根本原因。

---

# 2. 规则对路径规划的直接约束

根据 RMUA 2026 规则：

- 道路宽约 10 m；
- 道路飞行空间高度约 5 m；
- 每段路径需要经过中央交通枢纽；
- 检测门需要按照顺序触发；
- 道路和中央交通枢纽外存在静止力场；
- 飞出合法区域后动力会受到明显影响；
- 在静止力场中累计停留超过 2 s 会导致任务失败；
- 排名首先比较成功触发的检测门数量；
- 只有检测门数量相同时才比较时间。

因此第二阶段的算法目标不能再是：

> 用最短距离到达 `end_goal`。

而应该变成：

> **优先保证无人机始终留在合法赛道中，并沿正确方向持续推进，从而尽可能多地触发检测门。**

当前阶段：

```text
“活着并继续往前”
```

比：

```text
“飞得最快”
```

更重要。

---

# 3. 第二阶段的核心目标

本阶段只解决一个问题：

> **给无人机生成一条始终位于合法赛道中的参考路径，并让现有的速度控制器沿着这条路径运动。**

暂时仍然不解决：

- 动态汽车预测；
- 完整三维 SLAM；
- 工厂巡检；
- VIO / LIO；
- 高级轨迹优化；
- PWM 底层控制；
- 极限竞速。

第二阶段的最小目标为：

```text
Start
 ↓
合法道路中心附近
 ↓
中央交通枢纽
 ↓
目标道路中心附近
 ↓
Goal
```

并且全过程尽量不出道路边界。

---

# 4. 新系统和旧系统最大的区别

旧系统：

```text
当前位置
  ↓
end_goal
  ↓
直接追终点
```

新系统：

```text
当前位置
  ↓
合法参考路径
  ↓
当前应该追踪的局部目标
  ↓
P 控制
  ↓
vel_body_cmd
```

也就是说：

> `end_goal` 不再直接交给控制器。

`end_goal` 主要用于：

- 判断当前是哪一段路线；
- 确定最终目标；
- 生成或选择正确的赛道路径。

真正交给底层控制器的是：

```text
local_target
```

而不是：

```text
end_goal
```

---

# 5. 推荐总体结构

```text
/initial_pose
/end_goal
/pose_gt
    │
    ▼
┌──────────────────────┐
│     Route Manager     │
│      路线管理器        │
│                      │
│ Start → Hub → Goal   │
└──────────┬───────────┘
           │
           ▼
     Reference Path
       参考路径
           │
           ▼
┌──────────────────────┐
│    Path Follower      │
│     路径跟踪器         │
│                      │
│ 最近点                │
│ 横向误差              │
│ Look-ahead目标点      │
└──────────┬───────────┘
           │
           ▼
      local_target
           │
           ▼
┌──────────────────────┐
│  Velocity Controller │
│      速度控制器        │
│                      │
│ P控制                 │
│ 限速                  │
│ World → Body          │
└──────────┬───────────┘
           │
           ▼
     vel_body_cmd
           │
           ▼
          UAV
```

后面再加入：

```text
LiDAR
  ↓
Obstacle Avoidance
  ↓
修改 local_target
```

但当前阶段先不加入。

---

# 6. 第一版路线不需要复杂规划算法

现在完全没有必要马上使用：

- A*
- RRT*
- EGO-Planner
- FAST-Planner
- MPC

因为比赛道路具有很强的先验结构。

每段路线都明确是：

```text
起点道路
 ↓
中央枢纽
 ↓
终点道路
```

所以第一版路径规划完全可以使用：

> **预先建立的道路中心线 + 路径点 Waypoint。**

---

# 7. 最简单可行的路线表达

例如 1 → 3 路线：

```text
P0  起点区域
 ↓
P1  1号道路中心
 ↓
P2  1号道路中段
 ↓
P3  中央枢纽入口
 ↓
P4  中央枢纽中心
 ↓
P5  3号道路出口
 ↓
P6  3号道路中段
 ↓
P7  3号终点区域
```

表示成：

```text
route_1_3:

P0 = [x0, y0, z0]
P1 = [x1, y1, z1]
P2 = [x2, y2, z2]
P3 = [x3, y3, z3]
P4 = [x4, y4, z4]
P5 = [x5, y5, z5]
P6 = [x6, y6, z6]
P7 = [x7, y7, z7]
```

程序不再一次追最终 Goal。

而是：

```text
追 P1
 ↓
到 P1
 ↓
追 P2
 ↓
到 P2
 ↓
追 P3
 ↓
...
 ↓
最终 Goal
```

这就是最简单的路径规划。

---

# 8. Waypoint 从哪里得到

第一版不需要自动识别地图。

最稳妥的方法是：

> **利用本地模拟器和 `pose_gt`，人工采集道路中心线坐标。**

可以做一个简单的 waypoint recorder。

手动或低速控制飞机停在道路中心位置，然后记录：

```text
x
y
z
```

例如：

```text
1号起点
1号道路中段
枢纽入口
枢纽中心
3号道路出口
3号道路中段
3号终点
```

保存为：

```yaml
route_1_3:
  - [x0, y0, z0]
  - [x1, y1, z1]
  - [x2, y2, z2]
  - [x3, y3, z3]
  - [x4, y4, z4]
  - [x5, y5, z5]
  - [x6, y6, z6]
```

后续再逐步把这些人工点替换为自动地图规划。

---

# 9. 不建议只用很少的几个 Waypoint

理论上：

```text
Start
 ↓
Hub
 ↓
Goal
```

三个点可能就能工作。

但第一版建议多放一些中间点。

原因是：

- 可以限制无人机始终靠近道路中心；
- 防止坐标误差导致切角；
- 防止在枢纽入口附近走出边界；
- 后期可以给不同区段设置不同速度；
- 更容易调试。

建议初始间隔可以从：

```text
10 ~ 30 m
```

量级开始测试。

道路越窄、转向越明显：

```text
Waypoint 越密
```

直线长道路：

```text
Waypoint 可以更稀
```

---

# 10. 不要简单“到一个点再追下一个点”

如果完全采用：

```text
到 P1
 ↓
停一下
 ↓
转向 P2
 ↓
再飞
```

飞机会出现：

```text
走一步
停一下
走一步
停一下
```

甚至会在 waypoint 附近振荡。

更好的方式是：

> **Look-ahead 路径跟踪。**

---

# 11. Look-ahead 的基本思想

假设参考路径是：

```text
───────────────→
```

无人机当前位置：

```text
        UAV
         ●
```

先在参考路径上找到离无人机最近的点：

```text
        UAV
         ●
         |
         |
---------C------------→
```

然后不要追 C。

而是在路径前方一定距离选择：

```text
Look-ahead Point Q
```

例如：

```text
        UAV
         ●
         |
---------C------Q-----→
```

让无人机追 Q。

这样飞机会自然沿道路向前，而不是反复追离散 waypoint。

---

# 12. 路径最近点的计算

设当前所在路径段：

```text
A → B
```

无人机位置：

```text
P
```

定义：

\[
d=B-A
\]

则 P 在直线 AB 上的投影参数：

\[
t=
\frac{(P-A)\cdot(B-A)}
{\|B-A\|^2}
\]

再限制：

\[
0 \leq t \leq 1
\]

则路径最近点：

\[
C=A+t(B-A)
\]

这里：

```text
C
```

就是当前无人机在路径中心线上的投影点。

---

# 13. 横向偏差

定义：

\[
e_{cross}=P-C
\]

它表示：

> 无人机偏离道路中心线多少。

横向偏差大小：

\[
d_{cross}=\|P-C\|
\]

这个量以后非常重要。

因为现在我们的目标已经不是：

```text
离最终 Goal 多远
```

而是同时关心：

```text
沿道路前进了多少
```

和：

```text
离道路中心偏了多少
```

---

# 14. Look-ahead Point

路径方向单位向量：

\[
u=
\frac{B-A}{\|B-A\|}
\]

选择一个前视距离：

\[
L
\]

例如初始测试：

```text
L = 3 ~ 8 m
```

则：

\[
Q=C+Lu
\]

其中：

```text
Q
```

就是当前控制器要追的局部目标。

因此新的 Goal Follower 变成：

```text
不是追最终 end_goal

而是追：

local_target = Q
```

---

# 15. 为什么 Look-ahead 比普通 Waypoint 好

普通 Waypoint：

```text
当前点 → P1
到达 → P2
到达 → P3
```

容易出现：

- 到点减速；
- 到点振荡；
- 切换目标突变；
- 路线不连续。

Look-ahead：

```text
路径在前方不断滑动一个目标点
```

无人机始终追一个位于自己前方的点。

运动会更加连续：

```text
UAV → → → → → Goal
```

这和汽车 Pure Pursuit 的思想类似。

---

# 16. 新的速度控制目标

上一版：

\[
e=G-P
\]

现在改为：

\[
e=Q-P
\]

其中：

```text
Q = 当前 Look-ahead Point
```

然后：

\[
v_W=K_p(Q-P)
\]

再进行：

```text
速度限幅
 ↓
World → Body
 ↓
vel_body_cmd
```

所以你们之前的控制算法基本不需要推倒重写。

只需要：

> 把原来的 `end_goal` 换成 `local_target`。

---

# 17. 增加“回到中心线”的能力

仅仅追 Look-ahead 点还不够。

建议增加一个：

```text
Centerline Correction
中心线纠偏
```

定义中心线修正方向：

\[
v_{center}
=
K_{cross}(C-P)
\]

前进速度：

\[
v_{forward}
=
v_f u
\]

最后：

\[
v_W
=
v_{forward}
+
v_{center}
\]

也就是：

```text
一部分速度负责向前走
+
一部分速度负责回到道路中心
```

这会比单纯追终点稳定很多。

---

# 18. 一个直观例子

假设道路：

```text
────────────────────────→
```

无人机偏到了道路右侧：

```text
────────────────────────→

            ● UAV
```

此时：

```text
v_forward
```

仍然让飞机往前。

同时：

```text
v_center
```

把飞机拉回中心线。

于是：

```text
            ↖
             ● UAV
────────────────────────→
```

最终形成：

```text
     UAV → → → → →
────────────────────────→
```

这就是第二阶段真正需要的控制逻辑。

---

# 19. Boundary Guard：边界保护

道路宽度约：

```text
10 m
```

但第一版不要真的允许飞机跑到中心线 ±5 m。

应该人为留下安全余量。

例如初始测试可以定义：

```text
Soft Boundary = 2.0 ~ 2.5 m
Hard Boundary = 3.0 ~ 3.5 m
```

这些值不是赛事规则值，而是工程调试参数。

---

## 正常区域

如果：

\[
d_{cross}<2m
\]

正常飞行：

```text
高速向前
+
轻微中心线修正
```

---

## 警戒区域

如果：

```text
2m < d_cross < 3m
```

则：

```text
降低前进速度
+
增强中心线纠偏
```

例如：

```text
v_forward × 0.5
K_cross × 2
```

---

## 危险区域

如果：

```text
d_cross > 3m
```

则：

```text
大幅降低甚至取消前进速度
```

优先：

```text
返回中心线
```

即：

```text
v_forward ≈ 0
v_center = 较大
```

这样就形成：

```text
正常：
往前跑

偏了：
边跑边纠正

太偏：
先回来，再继续
```

---

# 20. 高度也必须作为“赛道约束”

道路合法飞行空间高度有限。

所以 Z 轴不能继续简单追最终 Goal 的高度。

建议每一个路径点都保存：

```text
[x, y, z_ref]
```

然后：

\[
v_z=K_z(z_{ref}-z)
\]

使无人机始终保持在道路中间相对安全的高度。

注意：

RMUA 世界坐标采用 NED 描述。

因此真正代码中：

```text
z 正负方向
```

必须通过本地模拟器实际测试确认后再固定。

不要只凭直觉写：

```text
z = +2.5
```

或者：

```text
z = -2.5
```

第一版先人工测出一个安全 `z_ref`。

---

# 21. 路线状态机

对于 1 → 3，可以先设计：

```text
START_EXIT
    ↓
ROAD_1
    ↓
HUB_ENTRY
    ↓
HUB
    ↓
HUB_EXIT_3
    ↓
ROAD_3
    ↓
GOAL_3
```

程序维护：

```text
current_segment
```

每完成一段后：

```text
current_segment++
```

例如：

```python
if progress_on_segment > 0.95:
    current_segment += 1
```

这样比单纯判断：

```text
距离 waypoint < 1 m
```

更加平滑。

---

# 22. 如何判断一段路径是否完成

对于当前线段：

```text
A → B
```

前面已经计算：

\[
t=
\frac{(P-A)\cdot(B-A)}
{\|B-A\|^2}
\]

如果：

```text
t ≈ 1
```

说明已经接近线段末端。

例如：

```text
t > 0.9
```

就可以提前切换到下一段。

这样无人机不用精确飞到每一个离散 waypoint。

---

# 23. 新版控制流程

完整流程：

```text
启动
 ↓
读取 route
 ↓
读取 pose_gt
 ↓
读取 end_goal
 ↓
确定当前 route
 ↓
确定 current_segment
 ↓
找到当前路径最近点 C
 ↓
计算横向偏差 d_cross
 ↓
生成 Look-ahead Point Q
 ↓
计算前进速度
 ↓
计算中心线修正速度
 ↓
执行 Boundary Guard
 ↓
执行高度控制
 ↓
得到世界系速度
 ↓
World → Body
 ↓
发送 vel_body_cmd
 ↓
无人机运动
 ↓
重新读取 pose
 ↓
循环
```

---

# 24. 核心伪代码

```python
while running:

    P = get_current_position()
    yaw = get_current_yaw()

    A = route[current_segment]
    B = route[current_segment + 1]

    # --------------------------------
    # 1. 当前路径方向
    # --------------------------------

    d = B - A

    segment_length = norm(d)

    u = d / segment_length

    # --------------------------------
    # 2. 找到路径最近点
    # --------------------------------

    t = dot(P - A, d) / dot(d, d)

    t = clamp(t, 0.0, 1.0)

    C = A + t * d

    # --------------------------------
    # 3. 横向误差
    # --------------------------------

    e_cross = C - P

    d_cross = norm_xy(e_cross)

    # --------------------------------
    # 4. Look-ahead
    # --------------------------------

    Q = C + lookahead_distance * u

    # 不允许超过当前路径段太远
    Q = limit_to_route(Q)

    # --------------------------------
    # 5. 前进速度
    # --------------------------------

    v_forward = forward_speed * u

    # --------------------------------
    # 6. 中心线修正
    # --------------------------------

    v_center = K_cross * e_cross

    # --------------------------------
    # 7. 边界保护
    # --------------------------------

    if d_cross < soft_boundary:

        forward_scale = 1.0

    elif d_cross < hard_boundary:

        forward_scale = 0.5

        v_center *= 2.0

    else:

        forward_scale = 0.0

        v_center *= 3.0

    # --------------------------------
    # 8. 世界系速度
    # --------------------------------

    v_world = (
        forward_scale * v_forward
        +
        v_center
    )

    # --------------------------------
    # 9. 高度控制
    # --------------------------------

    z_ref = interpolate_z(A, B, t)

    vz = Kz * (z_ref - P.z)

    vz = clamp(vz, -vmax_z, vmax_z)

    v_world.z = vz

    # --------------------------------
    # 10. 全局速度限幅
    # --------------------------------

    v_world = limit_velocity(v_world)

    # --------------------------------
    # 11. 世界系 → Body系
    # --------------------------------

    v_body = world_to_body(
        v_world,
        yaw
    )

    # --------------------------------
    # 12. 发送控制
    # --------------------------------

    send_vel_cmd(v_body)

    # --------------------------------
    # 13. 路径段切换
    # --------------------------------

    if t > 0.90:

        if current_segment < len(route) - 2:

            current_segment += 1

        else:

            if distance(P, final_goal) < goal_tolerance:

                send_zero_velocity()
                state = GOAL_REACHED
```

---

# 25. 推荐第一版参数

以下仅作为调试初始值，不是比赛标准参数。

```text
forward_speed:

初期：
1.5 ~ 2.0 m/s

稳定后：
3 ~ 5 m/s
```

```text
K_cross:

初期：
0.5 左右
```

```text
lookahead_distance:

初期：
3 ~ 5 m
```

```text
soft_boundary:

约：
2.0 m
```

```text
hard_boundary:

约：
3.0 m
```

```text
vmax_z:

约：
0.5 ~ 1.0 m/s
```

```text
segment_switch:

t > 0.90 ~ 0.95
```

实际数值必须通过模拟器反复调试。

---

# 26. 第一阶段暂时不要追求高速

当前算法优先级应该是：

```text
1. 不出界
2. 能持续前进
3. 能经过枢纽
4. 能进入目标道路
5. 能到终点
6. 再提高速度
```

不要反过来做：

```text
先 10 m/s
 ↓
然后想办法不撞
```

因为比赛排名首先看：

```text
检测门数量
```

所以：

> 稳定向前多走 300 m

往往比：

> 非常快地飞 100 m 后出界

更有价值。

---

# 27. 第一版路径规划的验收目标

第一阶段不要直接要求完整比赛。

先做：

## Test 1

```text
沿 1 号道路向中央枢纽飞
```

验收：

- 不出道路边界；
- 横向偏差稳定；
- 不出现明显振荡。

---

## Test 2

```text
1号道路
 ↓
中央交通枢纽
```

验收：

- 能顺利进入枢纽；
- 不在入口切角出界。

---

## Test 3

```text
中央交通枢纽
 ↓
3号道路
```

验收：

- 能正确选择 3 号出口；
- 能进入正确道路。

---

## Test 4

完整：

```text
1
 ↓
Hub
 ↓
3
```

验收：

- 全程不因边界冻结；
- 能进入 Goal 区域；
- 能重复执行。

---

# 28. 建议增加调试日志

每一帧至少记录：

```text
current_position
current_segment
t
cross_track_error
lookahead_point
world_velocity
body_velocity
distance_to_goal
```

例如：

```text
SEGMENT: ROAD_1_TO_HUB
PROGRESS: 0.42
CROSS_TRACK: 0.73 m
LOOKAHEAD: [123.4, 45.2, -2.1]
VEL_WORLD: [1.82, 0.31, 0.05]
```

这样一旦飞机飞出去，可以立刻判断：

```text
是路径错了？
还是坐标转换错了？
还是 K_cross 太小？
还是速度太快？
```

---

# 29. 强烈建议增加 Safety Guard

新增一个非常简单的保护条件：

```text
如果 cross_track_error 持续增大
```

则：

```text
降低前进速度
```

如果超过危险阈值：

```text
forward_speed = 0
```

只执行：

```text
返回中心线
```

例如：

```python
if d_cross > hard_boundary:

    v_forward = 0

    v_world = K_emergency * (C - P)
```

这样至少不会继续朝场外冲。

---

# 30. 下一步再加入 LiDAR

静态中心线路径只能保证：

```text
路线合法
```

不能保证：

```text
前面没有障碍
```

后续结构会升级成：

```text
Reference Path
     ↓

当前应该往前走
     ↓

LiDAR
     ↓
前方有没有障碍？
     │
     ├── 没有
     │     ↓
     │   沿中心线走
     │
     └── 有
           ↓
       局部绕障
           ↓
       回到中心线
```

因此后期不会推翻本方案。

而是在：

```text
Route Manager
```

和：

```text
Velocity Controller
```

中间加入：

```text
Local Planner
```

---

# 31. 后续完整结构

最终可以演化为：

```text
                Route Manager
                     ↓
              Reference Path
                     ↓
              Path Progress
                     ↓
LiDAR ─────→ Local Planner
                     ↓
                Local Target
                     ↓
            Boundary Constraint
                     ↓
            Velocity Controller
                     ↓
               vel_body_cmd
                     ↓
                    UAV
```

---

# 32. 当前阶段最重要的思想变化

第一版算法考虑的是：

```text
“Goal 在哪里？”
```

第二版开始必须考虑：

```text
“我允许在哪里飞？”
```

所以原来的问题：

\[
\min \text{distance to goal}
\]

要变成更接近：

\[
\text{沿合法路径持续增加 progress}
\]

同时约束：

\[
\text{cross track error 尽量小}
\]

即：

```text
目标：
尽可能向前走

约束：
不能跑出赛道
```

---

# 33. 当前最推荐的最小实现

不要立即上大型路径规划框架。

先做：

```text
① 人工采集 1→Hub→3 的中心线路径点

② 保存成 YAML

③ 写 Route Manager

④ 把原来的 end_goal follower
   改成 local_target follower

⑤ 增加路径投影

⑥ 增加 Look-ahead

⑦ 增加 cross-track correction

⑧ 增加 Boundary Guard

⑨ 在低速下完成 1→Hub→3

⑩ 稳定后再加 LiDAR 避障
```

这套方案是从你们现有算法升级过去改动最小的一条路线。

---

# 34. 和上一版代码的关系

上一版：

```text
end_goal
   ↓
P controller
   ↓
World → Body
   ↓
vel_body_cmd
```

这一版不需要推翻它。

只需要变成：

```text
end_goal
   ↓
Route Manager
   ↓
Reference Path
   ↓
Path Follower
   ↓
local_target
   ↓
原来的 P controller
   ↓
原来的 World → Body
   ↓
原来的 vel_body_cmd
```

因此：

> **已经写好的基本运动控制器继续保留。**

真正新增的是：

```text
Route Manager
+
Path Follower
+
Boundary Guard
```

三个模块。

---

# 35. 本阶段完成标准

满足以下条件即可认为第二阶段完成：

- 可以加载一条预定义赛道中心线；
- 能识别当前正在跟踪哪一段路径；
- 能计算当前路径投影点；
- 能计算横向偏差；
- 能生成 Look-ahead Point；
- 能沿中心线稳定前进；
- 偏离中心线后能够自动回来；
- 不会因为直接追 `end_goal` 而切出赛道；
- 能从起点道路进入中央枢纽；
- 能从中央枢纽进入目标道路；
- 能完成至少一整段 `Start → Hub → Goal`；
- 能重复运行而不是偶然成功。

完成后，再进入：

> **LiDAR 局部避障阶段。**

---

# 36. 参考资料

RoboMaster RMUA 2026 官方模拟器：

https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/tree/RMUA2026-01

RoboMaster RMUA 2026 基础开发仓库：

https://github.com/RoboMaster/IntelligentUAVChampionshipBase/tree/RMUA2026

RMUA 2026 规则内容要点：

- 世界系采用 NED；
- 道路宽约 10 m、高约 5 m；
- 每段路径需要经过中央交通枢纽；
- 检测门按顺序触发；
- 道路和中央交通枢纽外存在静止力场；
- 静止力场累计停留超过 2 s 会失败；
- 排名首先比较检测门数量，再比较时间。

实际比赛开发应始终以官方当赛季最新规则与模拟器版本为准。
