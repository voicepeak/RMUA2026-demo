# RMUA2026 无人机动态避障重构方案

## 0. 总要求

请基于当前仓库：

`https://github.com/voicepeak/RMUA2026-demo`

重新设计和实现车辆区域的无人机避障系统。

本次修改不是继续在现有避障逻辑上追加更多 `if/else`、恢复状态、侧移状态、后退状态或阈值补丁，而是要重新整理避障架构。

核心目标：

> 将目前的“空间路径规划 + ExecutionGuard 不断否决”的架构，改造成“动态障碍预测 + 时空轨迹规划 + ExecutionGuard 最终安全兜底”的架构。

现阶段优先使用已有 LiDAR，不要求首先引入双目摄像头，也不要求首先引入深度学习。

---

# 1. 当前问题判断

当前项目已经具备：

- LiDAR 点云；
- 基础点云聚类；
- 动态目标速度估计；
- 3D 局部路径搜索；
- `VelocityResponse`；
- `ExecutionGuard`；
- 制动距离和执行延迟建模；
- 路线中心线；
- 基础道路边界约束。

所以本次不要推翻所有已有代码。

当前真正的问题是：

```text
Planner:
只根据当前空间几何判断路径是否可行
        ↓
生成路线
        ↓
ExecutionGuard:
加入车辆未来运动、真实速度和制动距离检查
        ↓
发现未来可能碰撞
        ↓
COMMAND_BLOCKED
        ↓
停车 / 侧移 / 后退 / 重规划
        ↓
再次生成类似的空间路径
```

这会造成典型现象：

```text
看到空隙
→ 决定进入
→ Guard发现未来会撞
→ 刹车
→ 换另一个方向
→ 再被Guard否决
→ 左右摆动
→ 后退
→ 长时间无法穿过车辆区域
```

根本原因：

> Planner 与 ExecutionGuard 对“可行轨迹”的定义不一致。

因此新的 Planner 必须直接考虑：

```text
空间
+
时间
+
无人机动力学
+
动态车辆未来位置
```

---

# 2. 明确禁止继续走的老路

以下方案不得作为本次主要解决方案。

## 2.1 禁止继续堆避障状态机

不要继续新增：

```text
SIDE_SHIFT
SIDE_SHIFT_LEFT
SIDE_SHIFT_RIGHT
RETREAT
REJOIN
RECOVERY
RECOVERY_LEFT
RECOVERY_RIGHT
WAIT_CLEAR
FORCE_FORWARD
```

之类的状态去解决车辆段。

状态机可以存在，但只能负责：

```text
NORMAL
PLANNING
EMERGENCY_STOP
FAILSAFE
```

不能让状态机本身承担路径规划。

## 2.2 禁止继续靠调阈值解决结构问题

不要主要通过修改：

```text
clearance
stop_distance
side_offset
retreat_distance
blocked_timeout
recovery_timeout
速度阈值
障碍膨胀半径
```

来尝试“调通”。

这些参数只能在算法结构正确以后进行标定。

## 2.3 不要让 ExecutionGuard 继续充当 Planner

ExecutionGuard 应该只负责：

> 最后的安全否决。

正常情况下，Planner 输出的轨迹应该绝大多数能够直接通过 Guard。

如果运行中持续大量出现：

```text
COMMAND_BLOCKED
```

应认为 Planner 设计失败，而不是继续修改 Guard。

## 2.4 不要优先引入双目摄像头

当前已有 LiDAR。

双目摄像头解决的是：

```text
环境深度感知
```

当前主要瓶颈却是：

```text
动态障碍规划
```

因此第一阶段必须优先：

```text
LiDAR
→ Occupancy
→ Dynamic Tracking
→ Space-Time Planning
```

而不是：

```text
重新做 Stereo
→ Depth
→ PointCloud
→ 然后仍然使用旧 Planner
```

以后可以增加 Stereo / RGB 作为感知增强。

## 2.5 不要使用端到端深度学习控制

禁止第一阶段实现：

```text
RGB/LiDAR
    ↓
Neural Network
    ↓
vx vy vz
```

或者：

```text
RL
↓
无人机直接控制
```

当前问题必须保持可解释、可 debug。

深度学习未来只能作为：

- 车辆检测；
- 语义分割；
- 点云补全；
- 车辆轨迹预测；

等模块的增强。

## 2.6 不要直接搬 Fast-Planner / EGO-Planner / MADER 整套工程

这些项目用于参考设计思想。

当前项目已有：

- ROS 架构；
- 路线；
- LiDAR；
- 自己的控制接口；
- 自己的飞行动力学；
- ExecutionGuard。

优先复用当前代码。

不要为了使用某个论文代码把整个项目架构重写。

---

# 3. 新目标架构

目标架构：

```text
                 LiDAR PointCloud
                       │
          ┌────────────┴────────────┐
          ↓                         ↓
 Local Occupancy Map         Dynamic Tracker
 FREE/OCCUPIED/UNKNOWN       vehicle tracks
          │                         │
          │                         ↓
          │                  position / velocity
          │                  size / uncertainty
          │                  prediction(t)
          │                         │
          └────────────┬────────────┘
                       ↓
              Space-Time Planner
                (s,y,z,v,t)
                       ↓
               planned trajectory
                       ↓
              trajectory tracker
                       ↓
                VelocityResponse
                       ↓
               ExecutionGuard
                       ↓
                    VelCmd
```

原则：

> Planner 自己必须知道车辆未来什么时候在哪里。

而不能先规划一条空间路线，再让 Guard 判断未来会不会撞。

---

# 4. 坐标系

因为无人机已经沿既定赛道飞行，所以不要做完全自由的世界坐标四维/五维搜索。

使用路线坐标系：

```text
s = 沿 route 前进的距离
y = 相对于 route 中心线的横向偏移
z = 相对于参考高度的垂直偏移
```

规划状态：

```text
State:
    s
    y
    z
    v
    t
```

即：

```text
(s, y, z, v, t)
```

必要情况下可以额外包含：

```text
vy
vz
```

但第一版优先控制状态规模。

---

# 5. 第一模块：Local Occupancy Map

新建建议：

```text
local_occupancy.py
```

当前不能只依赖：

```text
nearest point distance
```

应该建立局部 voxel occupancy。

每个 voxel 有三种状态：

```text
FREE
OCCUPIED
UNKNOWN
```

## LiDAR ray casting

每根 LiDAR ray：

```text
sensor
  ●────────────────────X
  FREE FREE FREE FREE  OCCUPIED
```

规则：

- sensor 到 hit point 之间：`FREE`
- hit voxel：`OCCUPIED`
- 未观察区域：`UNKNOWN`

UNKNOWN 不允许默认认为完全安全。

## 推荐第一版参数

可结合现有代码调整，但初始可以：

```text
resolution:
0.25 ~ 0.5 m

forward:
25 ~ 30 m

lateral:
±4 m

vertical:
根据赛道实际范围
```

必须使用 rolling local map。

不需要第一阶段构建巨大世界地图。

---

# 6. 第二模块：Dynamic Tracker

建议新增：

```text
dynamic_tracker.py
```

现有 `lidar_scene.py` 可以复用或逐步迁移。

目标：

每辆动态车辆形成稳定 track。

数据结构建议：

```python
DynamicObstacle:
    track_id

    position
    velocity
    acceleration

    bbox_size

    timestamp
    age
    confidence

    position_covariance
    velocity_covariance
```

第一阶段不需要深度学习。

流程：

```text
LiDAR
 ↓
去地面 / ROI
 ↓
cluster
 ↓
bbox
 ↓
track association
 ↓
Kalman Filter
 ↓
DynamicObstacle
```

---

# 7. Kalman 跟踪

第一版可以使用：

```text
Constant Velocity Model
```

状态：

```text
[x, y, z, vx, vy, vz]
```

如果实际车辆加减速明显，再升级：

```text
Constant Acceleration
```

主要目标不是预测特别精确，而是做到：

```text
当前车辆在哪里
+
运动方向是什么
+
未来1~4秒大概在哪里
+
预测有多不确定
```

---

# 8. 动态障碍未来占据空间

每辆车辆必须能查询：

```python
obstacle.predict(t)
```

得到：

```text
position(t)
bbox(t)
uncertainty(t)
```

预测越远，不确定度越大。

因此实际 collision box 应随时间膨胀。

例如：

```text
t = 0.0

 ███

t = 1.0

█████

t = 2.0

███████
```

可以实现为：

```text
effective_bbox(t)
=
physical_bbox
+
base_margin
+
prediction_uncertainty(t)
```

不要用固定安全半径覆盖所有未来时刻。

---

# 9. 第三模块：Space-Time Planner

建议新增：

```text
st_lattice.py
```

这是本次重构最核心模块。

## 状态

第一版：

```python
Node:
    s
    y
    z
    v
    t
```

不要只搜索：

```text
(s,y,z)
```

必须存在：

```text
t
```

否则无法真正解决动态车辆。

---

# 10. Motion Primitive

不要任意从一个 voxel 跳到另一个 voxel。

候选 action 应具有物理意义。

例如：

```text
纵向：

减速
保持
加速

横向：

左移
保持
右移

垂直：

下降
保持
上升
```

组合后形成有限数量 motion primitives。

例如：

```text
KEEP

ACCELERATE
DECELERATE

LEFT
RIGHT

UP
DOWN

LEFT + DECELERATE
RIGHT + DECELERATE
...
```

不要让组合数量无限膨胀。

---

# 11. 必须使用真实无人机响应模型

当前已有：

```text
VelocityResponse
```

这是重要资产。

Planner 不应该假设：

```text
command == 实际速度
```

应该：

```text
command
 ↓
VelocityResponse
 ↓
预测未来0~N秒实际运动
 ↓
得到 trajectory(t)
```

Planner 判断的应该是：

> 实际无人机预计会走出的 trajectory。

而不是理想 waypoint 之间的直线。

---

# 12. Collision Check

对于一条候选 trajectory：

```text
P(t)
```

必须同时检查四类约束。

## 12.1 静态障碍

```text
occupancy(P(t)) != OCCUPIED
```

并满足：

```text
distance_to_static_obstacle > safety_margin
```

## 12.2 动态车辆

对于每辆：

```text
Car_i(t)
```

检查：

```text
DroneBBox(P(t))
intersection
CarBBox_i(t)
```

或者等价距离检查。

关键是：

```text
必须使用同一个 t。
```

不能拿：

```text
无人机未来位置
```

去和：

```text
车辆当前点云
```

比较。

## 12.3 道路边界

必须限制：

```text
y_min(s) < y < y_max(s)
```

以及：

```text
z_min(s) < z < z_max(s)
```

禁止 Planner 为了绕车飞出赛道合法区域。

## 12.4 动力学约束

至少包含：

```text
v <= vmax
a <= amax
vertical_speed <= vzmax
lateral_speed <= vymax
```

如果已有实测制动模型，必须使用实测值而不是理想值。

---

# 13. Planner 必须允许“等待”

这是当前架构非常容易遗漏的一点。

动态障碍环境中：

```text
绕过去
```

不是唯一策略。

必须允许：

```text
减速
等待
再通过
```

例如：

```text
车辆：

       ↓
──────────────
无人机  →
──────────────
```

最优轨迹可能是：

```text
6 m/s
↓
2 m/s
↓
等0.6秒
↓
从车辆后面穿过
↓
恢复6 m/s
```

而不是：

```text
左移
右移
升高
后退
```

因此 planner cost 需要允许：

```text
低速节点
甚至短暂停止节点
```

---

# 14. 第一版搜索范围

不要一开始追求巨大规划空间。

建议 MVP：

```text
spatial horizon:
20~25 m

time horizon:
3~4 s

ds:
1.0 m

dy:
0.5 m

dz:
0.5 m

dt:
0.2~0.25 s
```

速度离散：

```text
0
2
4
6
8 m/s
```

具体最大速度以当前项目配置为准。

---

# 15. Cost Function

第一版建议：

```text
J =
    progress_cost
  + time_cost
  + centerline_cost
  + altitude_cost
  + obstacle_proximity_cost
  + dynamic_obstacle_risk
  + acceleration_cost
  + jerk_cost
  + uncertainty_cost
```

主要目标优先级：

```text
第一：
不碰撞

第二：
能持续向前通过车辆区域

第三：
减少无意义左右摆动

第四：
减少急加速/急刹车

第五：
尽量靠近中心路线
```

绝不能为了：

```text
严格贴中心线
```

而导致车辆区域无法通过。

---

# 16. Dynamic Risk Cost

不要只使用：

```text
collision / no collision
```

还应该考虑接近风险。

例如：

```text
risk =
1 / distance_to_predicted_obstacle
```

再根据预测不确定性增加 cost。

这可以避免：

```text
理论上刚好擦过去
```

这种脆弱轨迹。

---

# 17. ExecutionGuard 的新职责

保留当前：

```text
execution_guard.py
```

不要删除。

但是职责重新定义。

它只负责检测：

```text
突发障碍
感知数据过期
planner异常
无人机实际速度偏离预测
车辆突然改变运动
轨迹执行误差
制动距离已经不足
```

然后：

```text
Emergency Brake / Stop
```

正常运行时：

```text
Planner trajectory
       ↓
ExecutionGuard
       ↓
PASS
```

应该是绝大多数情况。

如果：

```text
COMMAND_BLOCKED
```

长期频繁出现，应优先检查 Planner。

不要继续增强 recovery 状态机。

---

# 18. 不要直接删除现有 Planner

为了避免一次重构造成项目无法运行：

保留：

```text
lidar_navigation.py
```

旧 Planner。

新增：

```text
st_lattice.py
```

允许通过配置切换：

```text
planner_mode:
    legacy
    spacetime
```

开发完成后再默认：

```text
spacetime
```

方便 A/B 对比。

---

# 19. 推荐目录结构

目标可以整理成：

```text
route_follower/scripts/

lidar_navigation.py
    Planner入口和兼容层

lidar_scene.py
    点云基础处理

local_occupancy.py
    FREE/OCCUPIED/UNKNOWN voxel map

dynamic_tracker.py
    动态障碍tracking

st_lattice.py
    时空搜索

trajectory_collision.py
    静态+动态轨迹碰撞检查

velocity_response.py
    无人机执行模型

execution_guard.py
    最终安全保护
```

不要把所有功能继续塞进：

```text
lidar_navigation.py
```

---

# 20. 调试信息必须完善

这是本次修改的重要要求。

每个规划周期至少应该能够输出：

```text
Planner status

当前无人机：
position
velocity

动态车辆数量

每辆车：
track ID
position
velocity
predicted position @ 1s
predicted position @ 2s

候选轨迹数量

因静态障碍淘汰：
N

因动态障碍淘汰：
N

因道路边界淘汰：
N

因动力学淘汰：
N

最终轨迹 cost

ExecutionGuard：
PASS / BLOCKED
具体原因
```

禁止只打印：

```text
BLOCKED
```

却不知道原因。

---

# 21. 必须增加可视化

如果项目基于 ROS，可以发布 Marker。

至少显示：

```text
绿色：
FREE空间/最终轨迹

红色：
静态障碍

蓝色：
动态车辆当前位置

橙色：
车辆未来预测bbox

黄色：
候选无人机轨迹

紫色：
最终选中trajectory
```

需要能够肉眼判断：

```text
Planner认为车未来在哪里
无人机计划什么时候经过那里
```

这是动态避障 debug 的关键。

---

# 22. 第一阶段不要做的东西

第一版禁止主动扩展为：

- YOLO；
- 双目；
- Transformer；
- RL；
- Neural Motion Planner；
- 大规模全局建图；
- SLAM 重构；
- 多无人机协同；
- 复杂 MPC；
- 完整论文复现。

第一阶段唯一目标：

> 使用现有 LiDAR 和现有控制系统，让无人机能够稳定通过动态车辆区域。

---

# 23. 第二阶段再考虑视觉

只有第一阶段稳定以后，再考虑：

```text
Stereo Camera
      ↓
Dense Depth
      ↓
补充 LiDAR occupancy
```

或者：

```text
RGB
 ↓
YOLO
 ↓
车辆语义
 ↓
LiDAR 3D bbox
 ↓
Dynamic Tracker
```

视觉的职责是：

> 改善 perception。

不能用视觉掩盖 planner 的问题。

---

# 24. 推荐开发顺序

严格按以下顺序实施。

## Phase 1：先整理现有行为

先阅读：

```text
lidar_navigation.py
lidar_scene.py
execution_guard.py
velocity_response.py
当前配置
当前docs
```

输出当前实际 pipeline。

不要直接开始写代码。

确认哪些现有函数可以复用。

## Phase 2：Dynamic Tracker

首先实现：

```text
LiDAR cluster
→ persistent track
→ Kalman
→ prediction(t)
```

先不修改 Planner。

通过日志和可视化验证：

```text
车辆 ID 不乱跳
速度方向正确
未来1~2秒预测合理
```

## Phase 3：Occupancy

实现 rolling occupancy。

验证：

```text
FREE
OCCUPIED
UNKNOWN
```

以及 ray casting 是否正确。

## Phase 4：Space-Time Planner MVP

实现最简单：

```text
(s,y,z,v,t)
```

搜索。

先允许：

```text
forward
slow
wait
left/right
up/down
```

不要第一版追求平滑。

先追求：

> 能找到正确时机穿过车辆。

## Phase 5：VelocityResponse Integration

所有 candidate 必须通过：

```text
VelocityResponse
```

生成实际预测轨迹。

然后再做 collision checking。

## Phase 6：ExecutionGuard Integration

将 Planner 输出送到 Guard。

记录：

```text
planner认为safe
但guard拒绝
```

的情况。

如果频繁发生：

> 修 Planner。

不要继续增加 recovery。

## Phase 7：轨迹平滑和性能

通过车辆区以后再优化：

- jerk；
- acceleration；
- 路线贴合；
- 搜索速度；
- cache；
- pruning；
- heuristic。

---

# 25. Planner 搜索算法

第一版优先：

```text
A*
```

或者：

```text
Weighted A*
```

不需要第一版上复杂非线性优化。

heuristic 可以使用：

```text
remaining_s / vmax
```

以及：

```text
偏离目标中心线代价
```

动态障碍检测放在 edge expansion 中。

---

# 26. 搜索过程中必须检查整条 primitive

错误方式：

```text
只检查下一个Node安全
```

正确方式：

```text
Node A
 ↓
motion primitive
 ↓
sample t0
sample t1
sample t2
sample t3
...
 ↓
Node B
```

所有 sample 都必须：

```text
static safe
AND
dynamic safe
AND
road safe
AND
kinodynamic safe
```

否则 primitive 不可用。

---

# 27. Failure 行为

如果 Planner 找不到路径：

优先：

```text
安全减速
→ 停止
→ 等待下一轮预测
```

而不是马上：

```text
后退
```

只有在明确判断：

```text
前方长期完全堵塞
+
当前位置不能形成新的可行轨迹
```

时才允许有限后退。

Retreat 必须成为非常低频的最后方案。

不能再成为常规避障动作。

---

# 28. 防止左右振荡

对于：

```text
这一帧选择左
下一帧选择右
再下一帧又选择左
```

必须引入 trajectory commitment。

例如：

只要当前轨迹：

```text
仍然安全
```

就继续执行短时间：

```text
0.3~0.5 s
```

不要每帧完全重新选择不同拓扑路线。

如果新轨迹明显优于旧轨迹才切换。

可以加入：

```text
trajectory_switch_cost
```

防止左右振荡。

---

# 29. 必须明确区分三种原因

日志和内部状态必须区分：

```text
NO_PATH
```

没有合法空间路径。

```text
WAIT_FOR_DYNAMIC
```

空间上可以通过，但现在时机不合适，需要等车。

```text
EMERGENCY_BLOCK
```

当前轨迹已经进入紧急制动范围。

这三个状态不能全部处理成：

```text
BLOCKED
```

否则系统永远无法正确决策。

---

# 30. 最重要的验收测试

不要用：

```text
程序没报错
```

作为完成标准。

必须测试以下场景。

### Case A：静态障碍

前方一个静止车辆。

预期：

```text
提前规划绕过
不触发Emergency Guard
不频繁停车
```

### Case B：车辆横穿

车辆横穿无人机路线。

预期：

Planner能够选择：

```text
减速 / 等待
```

然后：

```text
从车后通过
```

而不是不停左右切换。

### Case C：连续两辆反向车辆

一辆向左，一辆向右。

预期：

Planner根据：

```text
time
```

选择真正存在的时隙。

不能只根据当前点云空隙决定。

### Case D：暂时完全堵塞

短时间没有可行路径。

预期：

```text
安全停车
WAIT_FOR_DYNAMIC
```

车辆离开后继续。

不能立即长距离后退。

### Case E：车辆突然改变速度

Planner原轨迹失效。

预期：

```text
重新规划
```

如果已经来不及：

```text
ExecutionGuard emergency brake
```

### Case F：静态窄通道

确认加入 time dimension 后：

静态避障能力没有退化。

---

# 31. KPI

至少记录：

```text
车辆段成功通过率

碰撞次数

Emergency Guard触发次数

COMMAND_BLOCKED次数

平均规划耗时

P95规划耗时

平均速度

停车次数

后退次数

左右轨迹切换次数

通过车辆段总耗时
```

目标不是：

```text
绝不减速
```

而是：

```text
安全
稳定
持续推进
可重复成功
```

---

# 32. 性能要求

当前是在线无人机规划。

不要写一个：

```text
几秒才能算一次
```

的 Planner。

第一阶段目标建议：

```text
规划频率：
至少 5~10 Hz

最好：
10 Hz 左右稳定运行
```

如果搜索空间过大：

优先：

```text
减少离散状态
限制 lateral范围
限制 vertical范围
减少 primitive
使用 pruning
使用 Weighted A*
```

而不是删除动态预测。

---

# 33. 代码修改原则

请做到：

```text
最小侵入
模块化
旧逻辑可回退
日志完善
参数集中配置
```

禁止：

```text
几千行全部重写到一个文件
大量魔法数字
硬编码特定车辆位置
硬编码某个仿真场景
针对某次录像写规则
```

必须能够适应车辆出现时间变化。

---

# 34. 最终预期结果

改造后正常流程应变成：

```text
LiDAR看到车辆
      ↓
Tracker发现：
car_1正在运动
      ↓
预测未来：
1s以后在这里
2s以后在那里
      ↓
Planner发现：
现在从左边走未来会相撞
      ↓
不选择左边
      ↓
计算：
减速0.5秒后存在安全时间窗口
      ↓
输出减速轨迹
      ↓
等待车辆通过
      ↓
Planner找到安全穿越轨迹
      ↓
正常通过
      ↓
ExecutionGuard始终PASS
```

而不是：

```text
先冲
→ Guard BLOCK
→ 左移
→ Guard BLOCK
→ 右移
→ Guard BLOCK
→ 后退
→ 再冲
```

---

# 35. 关于深度学习的最终规定

当前阶段：

```text
NO end-to-end DL avoidance
NO RL direct control
NO stereo-first rewrite
```

以后允许增加：

```text
YOLO：
车辆识别

Stereo：
Dense depth

Deep tracker：
目标关联

Trajectory prediction network：
复杂车辆行为预测
```

但 Planner 和 Safety Guard 必须仍然保持明确的几何和动力学安全约束。

---

# 36. 本次修改时，请先完成设计审查

在真正修改代码前，请先基于仓库当前代码输出：

```text
1. 当前避障调用链
2. 当前Planner入口
3. 当前Dynamic Scene入口
4. VelocityResponse在哪里进入
5. ExecutionGuard在哪里进入
6. 哪些代码保留
7. 哪些代码废弃
8. 哪些模块新增
9. 新旧模块调用关系
10. 第一阶段具体修改文件列表
```

确认设计后再开始代码修改。

但不要重新回到以前：

```text
继续堆side-shift / retreat / recovery状态
```

的设计。

核心原则始终是：

> 把动态障碍的未来运动提前放进 Planner。

而不是：

> Planner先随便规划，再依赖 Guard不断否决。

---

# 37. 实现优先级

如果工作量较大，请严格按优先级执行：

```text
P0
Dynamic Tracker + Prediction

P0
Space-Time Planner

P0
与 VelocityResponse 对齐

P0
动态collision checking

P1
Occupancy FREE/OCCUPIED/UNKNOWN

P1
可视化和debug信息

P1
Planner/Guard一致性验证

P2
性能优化

P3
Stereo / YOLO / Deep Learning
```

禁止因为：

```text
Stereo
YOLO
神经网络
```

看起来更先进，而推迟真正的 Planner 重构。

---

# 38. 一句话设计原则

本次避障重构只围绕这一句话执行：

> 不再解决“无人机现在从哪里绕过去”，而是解决“在未来几秒内，无人机在什么时间、以什么速度、经过什么空间，才能与动态车辆保持安全距离并持续向前”。

这才是本次车辆动态避障系统的核心。
