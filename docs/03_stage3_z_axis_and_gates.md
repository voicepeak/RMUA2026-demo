# RMUA 2026 第三阶段：Z 轴规划与检测门穿越方案

## 0. 当前进展

目前已经完成两件重要的事情：

### 第一阶段：基本 Start-to-Goal 闭环

已经能够完成：

```text
pose_gt
  ↓
当前位置

end_goal
  ↓
目标位置

当前位置 - 目标位置
  ↓
P 控制
  ↓
World → Body
  ↓
vel_body_cmd
  ↓
无人机运动
```

这解决了：

> 飞机能不能根据目标位置自动运动。

---

### 第二阶段：赛道内路径跟踪

由于直接追 `end_goal` 会切出赛道，目前已经增加：

```text
Reference Path
      ↓
Path Follower
      ↓
Look-ahead Point
      ↓
Cross-track Correction
      ↓
Boundary Guard
      ↓
Velocity Controller
```

现在无人机已经可以比较稳定地：

> 沿着赛道 XY 路径前进，而不是直接穿越边界。

---

# 1. 现在的新问题：Z 轴实际上还没有被“规划”

目前 XY 路径已经比较合理，但 Z 轴大概率仍然类似：

```text
z_ref = 一个固定值
```

或者：

```text
沿整条路线一直保持起始高度
```

这可以让无人机飞起来，但不是真正的三维路径规划。

问题在于 RMUA 不是纯二维赛道。

官方规则明确：

- 世界坐标系采用 NED；
- 道路空间宽约 10 m；
- 道路飞行空间高度约 5 m；
- 沿飞行路径设置一系列检测门；
- 检测门需要按顺序穿过才能得分；
- 成绩首先比较成功触发的检测门数量。

因此 Z 轴不仅是“飞多高”的问题，而是：

> **能不能从正确高度穿过下一道检测门。**

---

# 2. 一个非常重要的坐标系问题：NED

官方规则规定：

```text
World Frame = NED
```

即：

```text
X：North
Y：East
Z：Down
```

因此：

```text
Z 增大
= 往下

Z 减小
= 往上
```

这一点必须统一到整个程序中。

不要用平常直觉中的：

```text
z 越大 = 越高
```

来写控制逻辑。

推荐整个项目中明确使用：

```text
z_ned
```

而不是模糊的：

```text
height
```

如果要使用人类更直观的高度：

```text
altitude = -z_ned
```

再进行转换。

---

# 3. 我对 Z 轴方案的总体判断

不建议继续：

```text
固定 Z
```

也不建议现在马上变成：

```text
完全靠视觉实时找检测门
```

最合理的是一个三层方案：

```text
第一层：
3D Reference Path
提供正常情况下的 Z 参考

第二层：
Gate Alignment
看到检测门以后主动对准检测门中心

第三层：
LiDAR Safety
保证上下和周围有足够安全距离
```

即：

```text
3D 路线
   ↓
基础高度

检测门识别
   ↓
局部高度修正

LiDAR
   ↓
安全限制
```

---

# 4. 推荐方案：把现有 2D 路径升级成 3D 路径

现在可能保存的是：

```text
Waypoint = [x, y]
```

或者：

```text
Waypoint = [x, y, 固定z]
```

下一版改为：

```text
Waypoint = [x, y, z_ref]
```

例如：

```yaml
route_1_3:

  - [x0, y0, z0]
  - [x1, y1, z1]
  - [x2, y2, z2]
  - [x3, y3, z3]
  - [x4, y4, z4]
```

这样一条路线就不再是：

```text
二维折线
```

而是：

```text
真正的三维参考轨迹
```

---

# 5. Z 值从哪里来

第一版不需要自动计算。

最稳妥的办法是：

> **利用本地模拟器 + pose_gt，对关键位置进行人工测量。**

需要重点记录：

```text
起点中心高度
道路正常巡航高度
每一个检测门中心高度
中央枢纽入口高度
中央枢纽内部高度
中央枢纽出口高度
终点触发区中心高度
```

例如：

```yaml
waypoints:

  - name: road_start
    x: ...
    y: ...
    z: ...

  - name: gate_01
    x: ...
    y: ...
    z: ...

  - name: gate_02
    x: ...
    y: ...
    z: ...

  - name: hub_entry
    x: ...
    y: ...
    z: ...

  - name: hub_center
    x: ...
    y: ...
    z: ...
```

其中：

```text
gate_xx
```

就是检测门附近的关键约束点。

---

# 6. 为什么检测门应该成为 Z 轴关键节点

RMUA 的成绩首先看：

```text
成功触发多少个检测门
```

并且必须：

```text
Gate 1
 ↓
Gate 2
 ↓
Gate 3
 ↓
...
```

按顺序触发。

因此路径规划不能只考虑：

```text
我有没有沿着道路走
```

还必须考虑：

```text
我有没有真正从检测门内部穿过去
```

如果：

```text
XY 对了
```

但：

```text
Z 太高
```

或者：

```text
Z 太低
```

就有可能：

```text
从门的上方 / 下方错过
```

结果是：

```text
没有得分
```

而下一道门可能也无法正常触发。

所以检测门中心应该是：

> **比普通 Waypoint 更高级的强制路径点。**

---

# 7. 两种 Waypoint

建议以后把路径点分成两类。

## 普通路径点

作用：

```text
保持飞机沿道路走
```

例如：

```text
ROAD_GUIDE
```

它不需要精确经过。

允许：

```text
误差 1~2 m
```

---

## Gate Path Point

作用：

```text
确保穿过检测门
```

例如：

```text
GATE
```

要求更严格。

例如：

```text
XY error < 0.5~1.0 m
Z error  < 0.3~0.5 m
```

实际阈值后续根据模拟器测试调整。

---

# 8. 整条路径就变成

```text
Start
  ↓
Road Guide
  ↓
Road Guide
  ↓
Gate 1
  ↓
Road Guide
  ↓
Gate 2
  ↓
Road Guide
  ↓
Hub Entry
  ↓
Gate ...
  ↓
Hub
  ↓
Goal Road
  ↓
Gate ...
  ↓
Goal
```

这比：

```text
一条普通平滑曲线
```

更加符合比赛规则。

---

# 9. Z 轴不能在 Waypoint 之间突然跳变

假设：

```text
P1.z = -1.5
P2.z = -3.5
```

如果到 P1 以后突然：

```text
z_target = -3.5
```

无人机会突然产生较大垂直速度。

因此不能采用：

```text
到点
→ Z 瞬间切换
```

应该插值。

---

# 10. 最简单：线性 Z 插值

假设当前路径段：

```text
A → B
```

之前已经计算出：

```text
t ∈ [0,1]
```

表示当前沿该段前进的比例。

那么：

\[
z_{ref}
=
(1-t)z_A+t z_B
\]

即：

```text
t = 0
→ z = z_A

t = 0.5
→ 位于两者中间高度

t = 1
→ z = z_B
```

这样 Z 会随着路径自然变化。

---

# 11. 第一版 Z 控制

得到：

```text
z_ref
```

以后：

\[
e_z=z_{ref}-z
\]

然后：

\[
v_z=K_z e_z
\]

再限制：

```text
|vz| < vz_max
```

例如第一版：

```text
Kz = 0.3 ~ 0.5
vz_max = 0.5 ~ 1.0 m/s
```

注意 NED 符号。

实际代码必须通过模拟器确认：

```text
vel_body_cmd 的 z 正方向
```

与：

```text
pose_gt 的 NED Z
```

是否一致。

不要凭公式直接假设。

---

# 12. 更推荐：平滑高度曲线

等线性插值稳定以后，可以升级成：

```text
Cubic
```

或者：

```text
Smoothstep
```

例如使用：

\[
s(t)=3t^2-2t^3
\]

然后：

\[
z_{ref}
=
z_A+s(t)(z_B-z_A)
\]

相比线性：

```text
z
|
|      /
|     /
|    /
|___/________
```

Smoothstep 更像：

```text
z
|
|       __
|     /
|   /
|__/
```

开始和结束时变化更柔和。

对于无人机更加自然。

---

# 13. 是否要识别“得分方框 / 检测门”？

我的建议是：

> **要做，但不是现在就把它作为唯一的 Z 轴来源。**

正确定位应该是：

```text
3D 路线
= 主导航

检测门视觉识别
= 局部精确对准

LiDAR
= 安全保证
```

而不是：

```text
摄像头看见框
→ 所有导航都依赖这个框
```

---

# 14. 为什么不能完全依赖视觉识别检测门

因为存在：

```text
距离太远
```

```text
视角很斜
```

```text
被障碍物遮挡
```

```text
运动模糊
```

```text
门可能暂时不在 FOV 内
```

如果整个导航系统完全依赖：

```text
Gate Detector
```

一旦检测失败：

```text
没有目标
→ 飞机不知道去哪里
```

这是很危险的。

---

# 15. 检测门识别最适合解决的问题

它非常适合解决：

> **“我已经知道下一道门大概在哪里，现在帮我精确对准它。”**

也就是：

```text
Reference Path
       ↓
把飞机送到门附近
       ↓
Camera Detect Gate
       ↓
估计 Gate Center
       ↓
Local Target = Gate Center
       ↓
精确穿门
       ↓
恢复 Reference Path
```

---

# 16. 推荐新增 Gate Manager

建立：

```text
gate_index
```

例如：

```text
gate_index = 0
```

表示：

```text
现在应该通过 Gate 0
```

Gate Manager 维护：

```text
下一道门是谁
```

而不是：

```text
看见哪个门就冲哪个门
```

因为规则要求按顺序触发。

结构：

```text
Route Manager
     ↓
Expected Gate
     ↓
Gate Manager
     ↓
Gate Detector
     ↓
Gate Center
     ↓
Gate Alignment
```

---

# 17. 检测门识别可以怎么做

有三种方案。

---

## 方案 A：预先记录 Gate 3D 坐标

这是最简单、现在最推荐的。

如果模拟器中的检测门位置固定，那么本地测试时：

```text
人工找到 Gate
 ↓
记录 Gate Center
 ↓
保存 World XYZ
```

运行时直接：

```text
Gate Center
=
[xg, yg, zg]
```

优点：

```text
简单
稳定
几乎没有视觉失败问题
开发最快
```

缺点：

```text
依赖场景固定
```

---

# 18. 方案 B：相机识别 Gate

使用：

```text
Front Camera
```

检测：

```text
矩形框 / 门框
```

如果检测门视觉特征非常明显，可以：

```text
OpenCV
```

先做，不一定需要 YOLO。

大致：

```text
Image
 ↓
颜色 / 边缘
 ↓
Contour
 ↓
Quadrilateral
 ↓
Gate Candidate
```

然后得到图像上的：

```text
Gate Center Pixel
(u, v)
```

---

# 19. 仅有 2D 像素中心还不够

如果只知道：

```text
Gate 在画面中央偏上
```

只能用于：

```text
视觉伺服
```

例如：

```text
Gate center 在画面上方
→ 飞机向上调整

Gate center 在画面左边
→ 飞机向左调整
```

这已经可以用于穿门。

但如果想得到：

```text
Gate 的真实 XYZ
```

需要深度。

---

# 20. 推荐使用双目得到 Gate 深度

官方提供前视双目：

```text
front_left
front_right
```

因此可以：

```text
左右图像
 ↓
Stereo Matching
 ↓
Depth
 ↓
Gate Center Depth
```

然后：

```text
Gate pixel + Depth
 ↓
Camera 3D Coordinate
 ↓
TF
 ↓
World Coordinate
```

最终得到：

```text
Gate Center World XYZ
```

这时候 Gate 就可以真正成为局部 3D Goal。

---

# 21. 也可以使用 LiDAR 辅助 Gate 定位

另一个办法：

```text
Camera
 ↓
识别 Gate ROI

LiDAR
 ↓
提供附近点云

Camera + LiDAR
 ↓
估算门框平面 / 中心
```

这种方法后期会比较稳。

但开发复杂度明显高于：

```text
预存 Gate 坐标
```

所以不是当前最优先项。

---

# 22. 当前我最推荐的 Gate 方案

按阶段做。

## Stage A

```text
手工记录 Gate World XYZ
```

先让算法：

```text
100% 能按顺序通过检测门
```

---

## Stage B

加入：

```text
Camera Gate Detector
```

但只用于：

```text
验证 Gate 是否出现在预期位置
+
局部修正
```

---

## Stage C

再变成：

```text
Camera + Stereo
```

自动估算 Gate Center。

这样不会因为视觉模块不成熟拖死整个系统。

---

# 23. Gate Approach Mode

靠近检测门以后，建议自动切换控制模式。

正常道路：

```text
PATH_FOLLOWING
```

例如：

```text
速度 3~5 m/s
```

检测到距离 Gate：

```text
< 10~20 m
```

切换：

```text
GATE_APPROACH
```

降低速度：

```text
1~2 m/s
```

同时提高：

```text
XY 对准权重
Z 对准权重
```

---

# 24. Gate Approach 控制

设检测门中心：

\[
G=
(x_g,y_g,z_g)
\]

无人机：

\[
P=
(x,y,z)
\]

则：

\[
e_g=G-P
\]

可以分别控制：

\[
v_{xy}
=
K_{gate,xy} e_{xy}
\]

\[
v_z
=
K_{gate,z} e_z
\]

限制：

```text
速度较低
```

确保真正从门中央通过。

---

# 25. 不要在“门平面”前停住

目标不应该只是：

```text
Gate Center
```

否则 P 控制会在：

```text
门中心附近减速到 0
```

可能停在门里。

更好的方法是定义三个点：

```text
Gate Approach Point
        ↓
Gate Center
        ↓
Gate Exit Point
```

例如：

```text
          Gate

Approach    |    Exit
    ● ------□------ ●
```

形成：

```text
P_approach
    ↓
P_gate
    ↓
P_exit
```

这样飞机会：

```text
对准
→ 穿过去
→ 离开门区域
```

而不是：

```text
飞到门中心停住
```

---

# 26. Gate 三点模型

假设 Gate 法向单位向量为：

```text
n
```

Gate Center：

```text
G
```

定义：

\[
P_{approach}=G-dn
\]

\[
P_{exit}=G+dn
\]

例如：

```text
d = 2 ~ 5 m
```

于是：

```text
Approach
   ↓
Center
   ↓
Exit
```

全部采用 Gate Center 的 Z。

这会显著提高触发成功率。

---

# 27. 检测门通过判断

不要仅仅判断：

```text
distance_to_gate < threshold
```

因为飞机可能只是在门旁边。

更好的逻辑是：

```text
之前：
位于 Gate Plane 前方

之后：
位于 Gate Plane 后方
```

并且同时满足：

```text
横向偏差足够小
高度偏差足够小
```

即：

```text
Crossed Gate Plane
+
Inside Gate Region
```

则认为：

```text
Gate Passed
```

然后：

```text
gate_index += 1
```

---

# 28. LiDAR 在 Z 轴中应该承担什么角色

LiDAR 不建议现在负责：

```text
决定比赛高度
```

更适合负责：

```text
安全限制
```

例如实时计算：

```text
上方最近障碍距离
下方最近障碍距离
前方最近障碍距离
```

如果：

```text
上方太近
```

禁止继续向上。

如果：

```text
下方太近
```

禁止继续向下。

也就是说：

```text
Planner:
我想往上

Safety:
上面只有 0.3 m 空间

最终：
禁止往上
```

---

# 29. 三层 Z 控制结构

最终推荐：

```text
                  Gate Center
                      │
                      ▼
3D Reference Path → z_ref
                      │
                      ▼
                Z Controller
                      │
                      ▼
                desired vz
                      │
                      ▼
                  LiDAR Safety
                      │
                      ▼
                  final vz
```

其中：

```text
正常情况：
z_ref 来自 3D Path

靠近 Gate：
z_ref 渐渐变成 Gate Center Z

安全冲突：
LiDAR Safety 拥有最高优先级
```

---

# 30. Z Reference Blending

不要突然从：

```text
Path Z
```

切换到：

```text
Gate Z
```

可以使用权重融合：

\[
z_{ref}
=
(1-\alpha)z_{path}
+
\alpha z_{gate}
\]

其中：

```text
距离 Gate 很远：
α = 0

逐渐接近：
α 增大

靠近 Gate：
α = 1
```

例如：

```text
>20m：
α = 0

20~10m：
α 从 0 → 1

<10m：
α = 1
```

这样高度变化会非常平滑。

---

# 31. 推荐完整状态机

```text
PATH_FOLLOWING
      ↓
发现接近下一 Gate
      ↓
GATE_APPROACH
      ↓
高度 / XY 对准
      ↓
GATE_CROSSING
      ↓
保持 Gate Center 方向
      ↓
GATE_EXIT
      ↓
确认穿过
      ↓
gate_index++
      ↓
PATH_FOLLOWING
```

---

# 32. 路径数据结构推荐

建议以后 Waypoint 不再只是：

```text
[x, y, z]
```

而是：

```yaml
- type: ROAD
  x: ...
  y: ...
  z: ...

- type: GATE
  id: 3
  x: ...
  y: ...
  z: ...
  approach_distance: 4.0
  speed: 1.5

- type: ROAD
  x: ...
  y: ...
  z: ...
```

这样 Route Manager 可以知道：

```text
普通点
```

和：

```text
得分门
```

控制策略应该不同。

---

# 33. 推荐代码模块

在现有工程基础上新增：

```text
rmua_route/
    route_manager

rmua_gate/
    gate_manager
    gate_detector
    gate_tracker

rmua_planner/
    path_follower
    z_profile

rmua_safety/
    vertical_clearance
```

第一版真正需要实现的只有：

```text
z_profile
gate_manager
```

Gate Detector 可以晚一点。

---

# 34. 第一版实际开发顺序

推荐严格按照以下顺序：

## Step 1

检查目前代码：

```text
Z 是怎么生成的？
```

确认是不是：

```text
固定 z
```

---

## Step 2

人工记录：

```text
路线关键点的 XYZ
```

尤其：

```text
所有可见 Gate Center XYZ
```

---

## Step 3

将：

```text
2D Path
```

升级为：

```text
3D Path
```

---

## Step 4

实现：

```text
z_ref 插值
+
Z P Controller
+
vz limit
```

---

## Step 5

让无人机：

```text
不做视觉
```

先通过至少数个检测门。

验证：

```text
Gate 坐标 + 3D Path
```

是否足够可靠。

---

## Step 6

新增：

```text
Gate Manager
```

管理：

```text
当前应该穿第几个 Gate
```

---

## Step 7

把 Gate 加入：

```text
Approach
Center
Exit
```

三点模型。

---

## Step 8

再加入：

```text
Camera Gate Detection
```

用于：

```text
局部修正
```

而不是替代整个导航。

---

## Step 9

最后加入：

```text
LiDAR Vertical Safety
```

限制危险的上下运动。

---

# 35. 当前最推荐的 MVP

当前不要一下做复杂视觉。

最值得马上做的是：

```text
现有稳定 XY Path
        +
3D Waypoint
        +
Gate Center XYZ
        +
Z Interpolation
        +
Gate Approach / Center / Exit
```

即：

```text
Route
  ↓
3D Reference Path
  ↓
Look-ahead XY
  +
Interpolated Z
  ↓
Gate Near?
  │
  ├── No
  │    ↓
  │  正常 Path Following
  │
  └── Yes
       ↓
    Gate Approach
       ↓
    Align XYZ
       ↓
    Cross Gate
       ↓
    Gate Exit
```

---

# 36. 为什么我不建议现在把“识别方框”作为主方案

因为当前你们已经拥有：

```text
稳定的路径跟踪能力
```

如果现在把导航全部改成：

```text
Camera → 找框 → 飞框
```

相当于把一个已经稳定的系统重新变成：

```text
依赖视觉是否识别成功
```

风险反而变大。

当前比赛最重要的是：

```text
多触发检测门
```

所以应该优先：

```text
可靠穿门
```

而不是：

```text
一定要靠 AI 识别门
```

视觉应该成为：

> **提升 Gate Center 精度的传感器。**

不是：

> **整个导航系统唯一的基础。**

---

# 37. 最终推荐架构

```text
                 Route Manager
                      ↓
               3D Reference Path
                      ↓
             XY Path Follower
                      │
                      ├──────────┐
                      │          │
                      ↓          ↓
               XY Reference   Z Profile
                      │          │
                      └────┬─────┘
                           ↓
                     Local Target
                           │
                  接近下一 Gate？
                           │
             ┌─────────────┴─────────────┐
             │                           │
            No                          Yes
             │                           │
             ↓                           ↓
        正常路径跟踪                Gate Manager
                                         │
                             Camera Gate Detector
                                  （后续加入）
                                         │
                                         ↓
                                  Gate Center XYZ
                                         │
                                         ↓
                              Approach → Center → Exit
                                         │
                                         ↓
                                   Velocity Control
                                         │
                                         ↓
                                   LiDAR Safety
                                         │
                                         ↓
                                  vel_body_cmd
```

---

# 38. 本阶段的完成标准

认为 Z 轴与检测门阶段完成，需要满足：

- 路径已经由 2D 升级成 3D；
- 不再全程使用一个固定 Z；
- Z 能沿路径平滑变化；
- Z 方向和 NED 坐标定义完全确认；
- 每一道关键 Gate 有对应的 3D 中心；
- Gate 被当作强制通过点，而不是普通 waypoint；
- 可以按照 `Approach → Center → Exit` 穿门；
- 不会在 Gate Center 停住；
- 可以连续通过多道 Gate；
- Gate 顺序由 `gate_index` 管理；
- 发生较大 Z 误差时能够减速；
- 后续能够无缝加入视觉 Gate Detector；
- 后续能够无缝加入 LiDAR 垂直安全约束。

---

# 39. 最终结论

当前最优路线不是：

```text
固定 Z
```

也不是直接跳到：

```text
纯视觉识别得分方框
```

而是：

```text
第一步：
把现有 XY Path 升级成 XYZ Path

第二步：
把检测门中心加入 3D 路线，
作为必须经过的强约束点

第三步：
通过 Approach → Center → Exit
可靠穿过检测门

第四步：
再加入 Camera，
识别检测门并精确修正 Gate Center

第五步：
加入 LiDAR，
负责垂直和周围安全约束
```

对于当前已经能稳定沿路径飞行的系统，这条升级路线风险最低、改动最小，也最直接服务于比赛的核心评分目标：

> **尽可能多地按顺序通过检测门。**

---

# 40. 规则依据

RMUA 2026 规则中与本方案直接相关的要点：

- 世界系采用 NED；
- 道路宽约 10 m、高约 5 m；
- 沿飞行路径设置一系列检测门；
- 检测门必须按照“起点 → 中央枢纽 → 终点”的顺序逐个触发；
- 前一道检测门成功后，下一道检测门才可以有效触发；
- 每成功穿过一个未通过的检测门可完成有效触发；
- 成绩首先比较检测门触发数量；
- 检测门数量相同时才比较完成时间；
- 官方提供前后双目相机、MID360 LiDAR、IMU、GPS 等传感器；
- 随机生成的是道路中的线状障碍与悬浮汽车，规则没有说明检测门随机生成。

参考：

RoboMaster RMUA 官方赛事页面  
https://www.robomaster.com/zh-CN/robo/drone

RMUA 2026 官方模拟器  
https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/tree/RMUA2026-01

RMUA 2026 规则文本镜像  
https://github.com/guo1hao/RMUA2026/blob/main/doc/RoboMaster2026_rules_v1.txt

实际参赛时应以官方当赛季最新规则和模拟器行为为准。
