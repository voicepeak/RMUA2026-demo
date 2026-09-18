# RMUA 2026 最基础 Start-to-Goal 运动方案

## 1. 目标

本阶段只实现一个最小闭环：

> 无人机从当前起始位置出发，读取官方给出的 `goal location`，自动运动到目标点附近并停止。

暂时不处理：

- LiDAR 避障
- SLAM / LIO / VIO
- 工厂巡检
- 动态障碍
- Checkpoint
- 全局路径规划
- PWM 底层飞控
- 风场补偿

本阶段唯一目标是验证：

**“当前位置 → 目标位置 → 速度指令 → 无人机运动 → 更新当前位置”这一闭环能够正常工作。**

---

## 2. 最小系统结构

第一版只需要三个核心接口：

```text
/airsim_node/drone_1/debug/pose_gt
                ↓
           当前位姿

/airsim_node/end_goal
                ↓
             目标点

当前位姿 + 目标点
        ↓
   Goal Follower
        ↓
位置误差计算
        ↓
P 控制 + 速度限幅
        ↓
世界坐标系 → Body 坐标系
        ↓
/airsim_node/drone_1/vel_body_cmd
        ↓
      无人机
        ↓
新的 pose_gt
        ↓
重新计算
```

这构成最基本的闭环控制。

---

## 3. 三个核心量

### 3.1 当前位姿

调试阶段使用：

```text
/airsim_node/drone_1/debug/pose_gt
```

它提供无人机当前：

- 位置 `x, y, z`
- 姿态四元数 `qx, qy, qz, qw`

本阶段直接使用 `pose_gt`，目的是先验证导航与控制逻辑。

注意：

`pose_gt` 主要用于本地调试。正式比赛版本后续应逐步替换为 GPS、IMU、LiDAR 等允许使用的传感器所估计出的位姿。

---

### 3.2 目标位置

目标点来自：

```text
/airsim_node/end_goal
```

记目标位置为：

\[
G =
\begin{bmatrix}
x_g \\
y_g \\
z_g
\end{bmatrix}
\]

---

### 3.3 控制输出

速度控制接口：

```text
/airsim_node/drone_1/vel_body_cmd
```

本阶段使用速度控制，不直接使用 PWM。

原因是：

速度控制接口已经把“期望速度 → 姿态 → 电机控制”这一部分底层飞控工作封装掉。

我们只负责告诉无人机：

> 现在应该朝哪个方向，以多大的速度运动。

---

# 4. 为什么要计算位置误差

当前无人机位置记为：

\[
P =
\begin{bmatrix}
x \\
y \\
z
\end{bmatrix}
\]

目标位置为：

\[
G =
\begin{bmatrix}
x_g \\
y_g \\
z_g
\end{bmatrix}
\]

位置误差定义为：

\[
e = G-P
\]

即：

\[
e_x = x_g-x
\]

\[
e_y = y_g-y
\]

\[
e_z = z_g-z
\]

这里的“误差”不是测量错误，而是：

> 当前状态和目标状态之间还差多少。

例如：

```text
当前位置：
(2, 3, 2)

目标：
(10, 7, 2)
```

那么：

```text
ex = 8 m
ey = 4 m
ez = 0 m
```

也就是说：

> 在世界坐标系中，还需要向 x 方向移动 8 m，向 y 方向移动 4 m。

---

# 5. 用 P 控制把位置误差变成速度

最简单的控制方法：

\[
v = K_p e
\]

例如：

\[
v_x = K_p e_x
\]

\[
v_y = K_p e_y
\]

\[
v_z = K_{pz} e_z
\]

其中 `Kp` 是自己设定并通过实验调节的比例系数。

例如：

```text
Kp_xy = 0.3
Kp_z  = 0.3
```

如果距离目标还差 10 m：

```text
v = 0.3 × 10 = 3 m/s
```

如果只差 1 m：

```text
v = 0.3 × 1 = 0.3 m/s
```

因此 P 控制天然具备：

```text
距离目标远
→ 速度较快

距离目标近
→ 自动减速

接近目标
→ 速度趋近于 0
```

---

# 6. 为什么还需要速度限幅

如果目标距离很远：

```text
误差 = 100 m
Kp = 0.3
```

则理论上会得到：

```text
速度 = 30 m/s
```

第一阶段不需要这么激进。

因此设置最大速度：

```text
v_max_xy = 2 ~ 3 m/s
v_max_z  = 1 m/s
```

如果计算出的水平速度：

\[
\sqrt{v_x^2+v_y^2}>v_{max}
\]

则按比例缩小：

\[
scale=
\frac{v_{max}}
{\sqrt{v_x^2+v_y^2}}
\]

然后：

\[
v_x=v_x \cdot scale
\]

\[
v_y=v_y \cdot scale
\]

这样既保留运动方向，又限制最大速度。

---

# 7. 世界坐标系与 Body 坐标系

这是第一版最容易出错的地方。

## 7.1 世界坐标系

目标位置和当前位姿通常是在固定的地图坐标系中描述的。

世界坐标系不会随着无人机旋转。

例如：

```text
         +Y
          ↑
          |
          |
          +------→ +X
```

---

## 7.2 Body 坐标系

Body 坐标系绑定在无人机身上。

它会随着无人机 yaw 旋转。

可以简单理解：

```text
Body X
= 机头前方

Body Y
= 机身侧向

Body Z
= 垂直方向
```

因此：

> 世界坐标中的“向东飞”，不一定等于无人机自己的“向前飞”。

如果无人机机头朝北，而目标在东边，那么目标相对于无人机其实在侧面。

---

# 8. 世界速度转换为 Body 速度

设无人机当前 yaw 为：

\[
\psi
\]

世界坐标系期望速度为：

\[
v_x^W,\quad v_y^W
\]

转换到 Body 坐标系：

\[
v_x^B=
\cos\psi \, v_x^W
+
\sin\psi \, v_y^W
\]

\[
v_y^B=
-\sin\psi \, v_x^W
+
\cos\psi \, v_y^W
\]

然后将：

```text
vx_body
vy_body
vz
```

发送给：

```text
/airsim_node/drone_1/vel_body_cmd
```

---

# 9. yaw 从哪里来

`pose_gt` 中姿态通常以四元数形式存在：

```text
qx
qy
qz
qw
```

需要转换成欧拉角：

```text
roll
pitch
yaw
```

ROS Python 中可以使用：

```python
from tf.transformations import euler_from_quaternion

roll, pitch, yaw = euler_from_quaternion(
    [qx, qy, qz, qw]
)
```

本阶段真正需要的是：

```text
yaw
```

因为它决定世界坐标系和 Body 坐标系之间的水平旋转关系。

---

# 10. 第一版不主动控制 yaw

建议第一版：

```text
yaw_rate = 0
```

也就是说：

> 不要求机头始终朝向目标。

无人机允许侧飞。

例如：

```text
机头朝北
但目标在东边
```

程序经过坐标转换后，可以直接给侧向速度，使无人机向东运动。

这样可以减少第一版控制变量。

---

# 11. 到达目标的判断

不能判断：

```text
x == x_goal
y == y_goal
z == z_goal
```

因为连续控制中几乎不会精确相等。

使用距离：

\[
d=
\sqrt{
(x_g-x)^2+
(y_g-y)^2+
(z_g-z)^2
}
\]

例如设置：

```text
goal_tolerance = 1.0 m
```

如果：

```text
distance < 1.0 m
```

认为目标到达。

然后发送：

```text
vx = 0
vy = 0
vz = 0
yaw_rate = 0
```

无人机进入悬停状态。

---

# 12. 最小控制逻辑

完整流程：

```text
启动节点
   ↓
等待 pose_gt
   ↓
等待 end_goal
   ↓
读取当前位置
   ↓
读取目标位置
   ↓
计算位置误差
   ↓
计算距离
   ↓
距离 < 容差？
   │
   ├── 是
   │    ↓
   │  速度 = 0
   │    ↓
   │  GOAL_REACHED
   │
   └── 否
        ↓
      P 控制
        ↓
      速度限幅
        ↓
    读取当前 yaw
        ↓
世界速度 → Body 速度
        ↓
发送 vel_body_cmd
        ↓
等待下一帧 pose_gt
        ↓
重新计算
```

---

# 13. 伪代码

```python
while ros_is_running:

    if not pose_received:
        continue

    if not goal_received:
        continue

    # 1. 当前坐标
    x, y, z = current_pose.position

    # 2. 目标坐标
    xg, yg, zg = goal

    # 3. 世界坐标系位置误差
    ex = xg - x
    ey = yg - y
    ez = zg - z

    # 4. 目标距离
    distance = sqrt(
        ex * ex +
        ey * ey +
        ez * ez
    )

    # 5. 是否到达
    if distance < goal_tolerance:
        send_velocity(
            vx=0,
            vy=0,
            vz=0,
            yaw_rate=0
        )
        continue

    # 6. P 控制
    vx_world = Kp_xy * ex
    vy_world = Kp_xy * ey
    vz = Kp_z * ez

    # 7. XY 速度限幅
    speed_xy = sqrt(
        vx_world * vx_world +
        vy_world * vy_world
    )

    if speed_xy > vmax_xy:

        scale = vmax_xy / speed_xy

        vx_world *= scale
        vy_world *= scale

    # 8. Z 速度限幅
    vz = clamp(
        vz,
        -vmax_z,
        vmax_z
    )

    # 9. 四元数转 yaw
    yaw = quaternion_to_yaw(
        current_pose.orientation
    )

    # 10. 世界坐标系 → Body 坐标系
    vx_body = (
        cos(yaw) * vx_world
        +
        sin(yaw) * vy_world
    )

    vy_body = (
        -sin(yaw) * vx_world
        +
        cos(yaw) * vy_world
    )

    # 11. 发布速度
    send_velocity(
        vx=vx_body,
        vy=vy_body,
        vz=vz,
        yaw_rate=0
    )
```

---

# 14. 推荐初始参数

第一阶段以稳定调试为目标，不追求速度。

建议先从：

```text
Kp_xy = 0.3
Kp_z  = 0.3

vmax_xy = 2.0 ~ 3.0 m/s
vmax_z  = 1.0 m/s

goal_tolerance = 1.0 ~ 2.0 m

yaw_rate = 0
```

开始。

如果表现为：

```text
飞得太慢
→ 适当增大 Kp 或 vmax

接近目标时冲过头
→ 减小 Kp

远距离飞得太慢
但近距离控制正常
→ 优先增大 vmax

上下波动明显
→ 单独减小 Kp_z
```

---

# 15. 推荐开发顺序

## Step 0：确认接口

运行：

```bash
rostopic list
```

确认至少存在：

```text
/airsim_node/drone_1/debug/pose_gt
/airsim_node/end_goal
/airsim_node/drone_1/vel_body_cmd
```

然后查看消息类型：

```bash
rostopic type /airsim_node/drone_1/debug/pose_gt

rostopic type /airsim_node/end_goal

rostopic type /airsim_node/drone_1/vel_body_cmd
```

再进一步：

```bash
rosmsg show <消息类型>
```

不要提前猜字段名。

---

## Step 1：手动速度测试

先不写导航。

手动发送：

```text
vx = 1
vy = 0
vz = 0
```

观察无人机实际往哪里运动。

然后分别测试：

```text
vx
vy
vz
```

确认：

- Body X 正方向
- Body Y 正方向
- Body Z 正方向

尤其确认 Z 轴方向。

---

## Step 2：固定目标点

先不读取 `end_goal`。

代码中手动写：

```text
goal = (10, 0, z_target)
```

验证：

```text
pose_gt
→ P控制
→ vel_body_cmd
```

能够稳定到达固定目标。

---

## Step 3：接入官方 end_goal

把固定目标：

```text
goal = (10, 0, ...)
```

替换成：

```text
/airsim_node/end_goal
```

形成真正的：

```text
pose_gt
+
end_goal
↓
goal follower
↓
vel_body_cmd
```

闭环。

---

# 16. 第一阶段验收标准

建议不要用“看起来能飞过去”作为验收标准。

至少满足：

1. 节点能够正常读取 `pose_gt`
2. 节点能够正常读取 `end_goal`
3. 飞机启动后能够自动朝目标方向移动
4. 坐标系方向无反转
5. 接近目标时能够自动减速
6. 距离目标 1～2 m 时自动悬停
7. 不发生持续振荡
8. 重启多次后仍能重复完成
9. 连续测试 10 次，能够稳定达到目标附近
10. 所有控制量都有速度限幅

达到以上条件后，本阶段完成。

---

# 17. 本阶段暂时不要做的事情

在第一个 Start-to-Goal 跑通之前，不建议加入：

```text
LiDAR
SLAM
FAST-LIO
EGO-Planner
YOLO
Checkpoint Manager
Dynamic Obstacle Tracking
PWM Controller
MPC
Wind Compensation
```

因为这些模块会让问题复杂化。

现在只验证最基本的：

```text
我在哪
+
我要去哪
↓
怎么算方向和速度
↓
飞机能不能真的过去
```

---

# 18. 下一阶段

完成本阶段后，推荐按照以下顺序继续：

```text
Stage 0
pose_gt → end_goal
基础运动闭环

↓
Stage 1
GPS + IMU
替代 pose_gt

↓
Stage 2
加入 waypoint
不再只追一个最终目标

↓
Stage 3
加入 LiDAR
实现静态避障

↓
Stage 4
局部规划
处理动态障碍

↓
Stage 5
Checkpoint / 正式路线

↓
Stage 6
工厂定位与视觉任务
```

---

# 19. 本阶段最核心的一句话

整个第一版程序，本质上只做：

```text
目标位置 - 当前位置
        ↓
      位置误差
        ↓
       Kp
        ↓
     期望速度
        ↓
世界坐标系 → Body 坐标系
        ↓
    vel_body_cmd
        ↓
      无人机
```

只要这条链路稳定跑通，就完成了 RMUA 项目的第一个最小可运行版本。
