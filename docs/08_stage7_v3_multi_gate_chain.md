# RMUA 2026 Stage 7 v3：多 Gate 前视、矩形链拟合、动态 Z 轨迹与提速方案

## 0. 当前问题

目前已经完成：

- XY 路径可以稳定跟随；
- OpenCV 可以识别 Gate；
- 双目可以估计 Gate XYZ；
- 已经采集到一批真实 Gate；
- 基础 Gate 顺序和穿门逻辑已经建立。

但是当前控制策略仍存在三个明显问题：

```text
1. Gate 吸附策略太局部
   → 靠近门以后突然拉向门中心
   → 轨迹不够顺滑

2. 双目识别没有稳定保证“下一道门一定在前一扇门之后”
   → 可能出现 Gate 顺序 / 深度关联错误
   → Z Profile 可能被错误 Gate 带偏

3. 飞行和调试速度偏慢
   → 一次完整测试耗时长
   → 参数迭代效率低
```

另外目前仍存在：

> **后半程 Z 越来越低，最后进入异常区域后无法继续。**

这一问题不能再只通过单门 Z 控制解决。

下一阶段应该把问题从：

```text
“追下一扇门”
```

升级成：

> **“提前看到未来多扇门，把这些矩形门组成一条有顺序的三维 Gate Chain，再根据 Gate Chain 生成连续、平滑、可高速跟踪的 3D Reference Path。”**

---

# 1. 核心策略变化

旧策略：

```text
XY Route
 ↓
接近 Gate
 ↓
Gate 吸附
 ↓
对准中心
 ↓
穿门
```

问题：

```text
局部反应式
门与门之间轨迹不连续
容易突然横移 / 升降
```

新策略：

```text
提前识别多个 Gate
 ↓
确定 Gate 顺序
 ↓
构造 Gate Chain
 ↓
拟合未来 3D Path
 ↓
无人机沿平滑轨迹穿过多个 Gate
```

核心思想：

> **Gate 不再是一个个独立目标，而是一组控制点。**

---

# 2. Gate Chain

假设相机当前能看到：

```text
Gate A
Gate B
Gate C
Gate D
```

不再：

```text
只追 Gate A
```

而是建立：

```text
Gate A Center
    ↓
Gate B Center
    ↓
Gate C Center
    ↓
Gate D Center
```

形成：

```text
Gate Chain
```

然后通过这些 Gate Center 拟合：

```text
3D Reference Curve
```

飞机直接跟随这条曲线。

---

# 3. 为什么这比 Gate 吸附更适合当前赛道

赛道中的 Gate：

```text
数量多
大致沿道路排列
形状统一
方向连续
```

因此天然具有：

```text
空间序列结构
```

也就是说：

```text
单个 Gate
```

识别可能有误差，

但：

```text
连续多个矩形 Gate
```

整体排列关系很稳定。

可以利用：

```text
前后关系
大小关系
透视关系
中心连线
路径方向
深度关系
```

共同判断 Gate 顺序。

---

# 4. 新的总体架构

```text
Front Stereo Camera
        ↓
Gate Rectangle Detection
        ↓
Left/Right Stereo Matching
        ↓
Gate 3D Candidates
        ↓
Gate Chain Builder
        ↓
Ordered Gates
G0 → G1 → G2 → G3 ...
        ↓
3D Curve Fitting
        ↓
Reference Path XYZ
        ↓
Look-ahead Controller
        ↓
Velocity Planner
        ↓
vel_body_cmd
```

原来的：

```text
XY Route
```

仍然保留。

作用变成：

```text
Gate Chain 的全局方向先验
+
Gate 顺序验证
+
识别失败时的 fallback
```

---

# 5. 第一核心问题：必须保证 Gate 是“前后有序”的

当前双目只解决：

```text
这个矩形离我多远？
```

但没有完全解决：

```text
这个矩形到底是第几道门？
```

因此必须增加：

> **Gate Ordering / Gate Chain Validation**

---

# 6. Gate 顺序不能只靠双目深度

单纯按照：

```text
depth 从小到大
```

排序不够可靠。

因为：

- 双目远距离误差会增大；
- Gate 可能不完全正对相机；
- 不同道路可能同时进入视野；
- Gate 后面可能出现其他矩形结构；
- 某一帧匹配可能错误。

因此 Gate 顺序应该综合：

```text
Route Progress
+
Camera Depth
+
Image Geometry
+
Temporal Tracking
```

---

# 7. Gate 顺序的第一约束：Route Progress

已有 XY Route。

每个 Gate World XYZ 得到以后：

```text
Gate XY
 ↓
投影到 XY Route
 ↓
得到 Path Progress s_gate
```

然后要求：

```text
下一 Gate 的 s
必须 > 当前 Gate 的 s
```

例如：

```text
G0.s < G1.s < G2.s < G3.s
```

这是最重要的顺序约束。

如果视觉识别出：

```text
Gate Candidate C
```

但：

```text
C.s < current_gate.s
```

说明：

```text
它在飞机后面
```

或者：

```text
发生误匹配
```

直接拒绝。

---

# 8. 第二约束：相机前方约束

所有有效未来 Gate 必须：

```text
位于相机前方
```

即 Camera Frame：

```text
Z_camera > 0
```

并设置最小值：

```text
Z_camera > 2~3m
```

避免：

```text
已经穿过的 Gate
```

仍然因为边缘可见而被识别成下一 Gate。

---

# 9. 第三约束：深度单调关系

正常情况下：

```text
Gate 0
Gate 1
Gate 2
```

在当前相机视角中：

```text
depth_0 < depth_1 < depth_2
```

因此候选链必须大体满足：

```text
D0 < D1 < D2 ...
```

允许一定误差：

```text
D_(i+1) > D_i + depth_margin
```

例如：

```text
depth_margin = 1~3m
```

---

# 10. 第四约束：图像尺寸规律

透视情况下：

```text
越近的 Gate
→ 图像中越大

越远的 Gate
→ 图像中越小
```

因此：

```text
bbox_area_0
>
bbox_area_1
>
bbox_area_2
```

不需要严格单调，

但可以作为：

```text
排序置信度
```

的一部分。

---

# 11. 第五约束：中心点应形成连续曲线

图像中多个 Gate Center：

```text
C0
C1
C2
C3
```

理论上不会：

```text
左
右
左
突然跳
```

而应该形成相对连续的：

```text
透视中心线
```

因此可以对：

```text
Gate Center Pixels
```

做：

```text
Polynomial / Spline Fit
```

如果某个 Candidate：

```text
偏离拟合曲线很远
```

则认为可能是：

```text
误检
或者
其他道路 Gate
```

---

# 12. Gate Chain Score

每个 Gate Candidate 不再单独判断。

给整个 Gate Chain 一个 Score：

```text
score =
w_route     * route_order_score
+
w_depth     * depth_order_score
+
w_size      * perspective_size_score
+
w_curve     * center_curve_score
+
w_temporal  * tracking_score
+
w_conf      * detector_confidence
```

选择：

```text
总 Score 最高的 Gate Chain
```

而不是：

```text
单帧 confidence 最大的 Gate。
```

---

# 13. 远距离识别要更积极

当前 Gate Detection 应该把目标从：

```text
“准确识别最近一扇门”
```

改成：

> **“尽可能提前看到未来 3~5 扇门。”**

因此 OpenCV Detector 调参方向：

```text
允许更小的 bbox
降低最小 contour area
允许更细的矩形边缘
增强远距离橙色 / 白色检测
```

但是：

```text
远距离检测置信度可以低
```

因为最终会经过：

```text
Gate Chain Validation
```

过滤。

---

# 14. 推荐识别范围

如果画面允许：

```text
至少稳定看到：
2~3 个 Gate
```

最好：

```text
3~5 个 Gate
```

当前控制器实际只需要：

```text
最近 2~3 个可靠 Gate
```

用于轨迹拟合。

远处 Gate 只是：

```text
趋势参考。
```

---

# 15. 多 Gate 追踪

每一帧都重新识别 Gate 会有跳变。

必须保留：

```text
Gate Track
```

每个 Track 保存：

```text
track_id
world_xyz
camera_depth
image_center
bbox_size
confidence
support
last_seen
path_s
```

连续帧通过：

```text
Nearest Neighbor
+
Route s
+
3D Distance
+
IoU
```

保持 Gate ID。

---

# 16. Gate Track 预测

下一帧可以根据：

```text
无人机速度
+
上一帧 Gate Camera XYZ
```

预测 Gate 在新图像中的大致位置。

因此检测器搜索区域可以更小。

这会提高：

```text
速度
稳定性
远距离小 Gate 保持能力
```

---

# 17. Gate Chain 使用世界坐标，不使用当前单帧坐标

一旦一个 Gate Track 稳定：

```text
连续 N 帧
```

就形成：

```text
World Gate Anchor
```

后面不需要每一帧完全相信当前双目。

使用：

```text
历史融合 XYZ
```

例如：

```text
median
EMA
Kalman Filter
```

这样：

```text
远处 Gate 深度抖动
```

不会直接导致：

```text
Z Path 抖动。
```

---

# 18. 重点修复 Z 过低问题

当前最后 Z 太低导致停止，有两种可能来源：

```text
A. 真实赛道确实继续下降
B. 双目 Gate 深度 / Gate 关联错误，
   把错误矩形的 Z 当成下一 Gate
```

当前必须优先验证：

> **后半段 Gate 是否构成合理、连续的 3D 空间曲线。**

不能再只看：

```text
单个 Gate Z。
```

---

# 19. Gate Z 连续性检查

对连续 Gate：

```text
G_i
G_(i+1)
```

计算：

```text
Δs = s_(i+1) - s_i
Δz = z_(i+1) - z_i
```

定义坡度：

\[
k_z=
\frac{\Delta z}{\Delta s}
\]

如果出现：

```text
|k_z| 过大
```

例如明显超出周围 Gate 趋势：

```text
Gate i:
z=-7

Gate i+1:
z=-15

Gate i+2:
z=-8
```

则：

```text
Gate i+1
```

极可能：

```text
深度 / 匹配 / 识别异常。
```

不能立即用于控制。

---

# 20. Z Gate Outlier Reject

推荐初始：

```text
如果某 Gate 的：
Δz / Δs
```

明显超出：

```text
邻近 Gate 中位坡度
```

则标记：

```text
z_suspect = true
```

暂时：

```text
不参与 Z Curve Fit
```

继续观察后续多帧。

---

# 21. 多 Gate Z 拟合

不要：

```text
Gate A Z
→ 线性插值
→ Gate B Z
```

一扇一扇直接接。

改为：

```text
未来 3~5 个 Gate
```

一起拟合。

推荐：

```text
Cubic Spline
```

或者第一版：

```text
PCHIP
```

PCHIP 的优点：

```text
经过所有有效 Gate Anchor
不容易产生过冲
保持局部单调性
```

非常适合 Z。

---

# 22. 3D Curve

XY 不再只用原本 route。

可以逐步改为：

```text
未来 Gate Centers
+
已有 XY Route
```

拟合：

```text
x(s)
y(s)
z(s)
```

当前第一版建议：

```text
XY：
仍以原 Route 为骨架

Z：
由 Gate Chain 拟合

Gate 附近：
XY 向 Gate Center 轻度收敛
```

不要再使用强吸附。

---

# 23. Gate Center 作为软约束

旧：

```text
10m 内：
Gate 100% 接管
```

新：

```text
Gate Center
只是 Reference Curve 的控制点
```

即：

```text
Path
↓
自然经过 Gate Center
```

飞机实际跟随：

```text
Look-ahead Point
```

而不是：

```text
直接追 Gate Center。
```

---

# 24. 新的 Look-ahead 控制

获得：

```text
3D Reference Curve
```

以后：

```text
当前路径最近点 s_now
```

然后：

```text
s_target =
s_now + L
```

得到：

```text
Target = Curve(s_target)
```

控制：

```text
P → Target
```

这样即使 Gate 很密集：

```text
目标点始终在前方
```

不会：

```text
冲向一扇 Gate
↓
穿完
↓
突然拐向下一扇 Gate。
```

---

# 25. Look-ahead 距离动态调整

高速：

```text
Look-ahead 应该更远。
```

低速 / 急弯：

```text
Look-ahead 应该更近。
```

推荐：

\[
L =
L_0 + k_v v
\]

例如：

```text
L0 = 4m
kv = 1.5~2.0s
```

如果：

```text
v = 4m/s
```

则：

```text
L ≈ 10~12m
```

这样高速时仍然顺滑。

---

# 26. Gate 不再有“吸附状态”

建议删除或弱化：

```text
GATE_CAPTURE
GATE_ALIGN
```

新的状态更简单：

```text
PATH_TRACKING
 ↓
GATE_CORRIDOR
 ↓
GATE_PASS
 ↓
PATH_TRACKING
```

实际上 Gate Corridor 可以：

```text
全程自然存在。
```

---

# 27. Gate Pass 仍然保留

虽然不吸附，

Gate 仍然是：

```text
计分检查点。
```

所以当飞机穿过 Gate Plane：

```text
检查：

lateral error
vertical error
```

满足：

```text
门洞范围
```

则：

```text
PASSED
```

然后继续。

---

# 28. 如果预测下一 Gate 会穿偏

因为已经有：

```text
未来 Reference Curve
```

所以可以提前判断。

例如预测：

```text
在 Gate Plane 处：

predicted_x
predicted_z
```

与 Gate Center：

```text
gate_x
gate_z
```

比较。

如果预计：

```text
会偏出门洞
```

则：

```text
提前 20~30m
```

就开始调整曲线。

不是：

```text
10m 内强行吸附。
```

---

# 29. 预判距离建议扩大

当前：

```text
20m 左右开始 Gate 控制
```

可以改成：

```text
30~50m
```

甚至：

> **只要远处 Gate 能可靠识别，就立刻进入 Gate Chain。**

例如：

```text
Gate 1:
20m

Gate 2:
40m

Gate 3:
60m
```

当前轨迹就已经知道：

```text
未来 60m 的大致三维走向。
```

这会大幅提高平滑度。

---

# 30. 避障接口继续保留

即使改成 Gate Chain，

以后仍然需要：

```text
Obstacle Avoidance
```

所以整体结构保持：

```text
Reference Curve
 ↓
Nominal Velocity
 ↓
Obstacle Avoidance
 ↓
Final Velocity
```

其中：

```text
Gate Chain
```

生成：

```text
Nominal Path
```

避障则：

```text
在局部修改 Nominal Path / Velocity
```

而不是：

```text
Gate 吸附 与 避障抢控制权。
```

这比上一版 Arbiter 更自然。

---

# 31. 推荐未来避障接口

当前留：

```text
Local Planner Input:
- Reference Curve
- Current Pose
- Current Velocity
- Obstacle Map

Output:
- Local Collision-Free Trajectory
```

现在：

```text
没有障碍
→ 直接输出 Reference Curve
```

以后：

```text
有障碍
→ 局部绕开
→ 再回到 Gate Chain
```

---

# 32. 速度必须明显提高

目前调试慢的主要原因：

```text
Gate 模式过早减速
+
每一道 Gate 都进入低速 ALIGN
```

取消强吸附以后，

大多数 Gate 可以：

```text
连续通过
```

不需要降到：

```text
0.8~1.0m/s
```

---

# 33. 新速度策略

建议开发阶段：

```text
Straight / Stable:
4 ~ 6 m/s

轻微转向:
3 ~ 4 m/s

高度变化明显:
2.5 ~ 4 m/s

Gate 前:
如果预测通过误差小
不减速

只有预测会偏出 Gate:
降到 2 ~ 3 m/s
```

也就是：

> **不再“看到 Gate 就减速”。**

---

# 34. Curvature-based Speed

根据 Reference Curve 曲率：

```text
κ
```

调节速度：

\[
v_{curve}
=
\frac{v_{max}}
{1+k_\kappa |\kappa|}
\]

直线：

```text
速度高。
```

弯道：

```text
自动降速。
```

---

# 35. Z Slope-based Speed

同时根据：

\[
k_z=
\left|
\frac{dz}{ds}
\right|
\]

限制水平速度：

\[
v_zlimit
=
\eta
\frac{v_{z,max}}
{k_z+\epsilon}
\]

最终：

```text
v_desired =
min(
    v_cruise,
    v_curve,
    v_zlimit
)
```

这样：

```text
道路直
高度平缓
→ 6m/s

高度变化大
→ 自动降速
```

---

# 36. 调试快速模式

建议专门增加：

```text
debug_fast_mode
```

参数：

```yaml
debug_fast_mode: true

cruise_speed: 5.0
max_speed: 6.0

lookahead_base: 5.0
lookahead_kv: 1.5

slow_only_if_gate_miss_predicted: true
```

调试时：

```text
快速跑。
```

最终比赛再调：

```text
race_mode
```

---

# 37. 可以中途启动

继续保留：

```text
start_from_s
```

或者：

```text
自动检测当前 path progress
```

测试后半段 Gate 时：

```text
直接从 Gate 5 附近开始
```

不需要每次：

```text
从 Gate 0 跑 2 分钟。
```

---

# 38. 建议增加 Segment Test

启动参数：

```text
--start_gate 5
--end_gate 9
```

只跑：

```text
Gate 5 → Gate 9
```

或者：

```text
Gate 6 → Gate 7
```

专项测试。

这会显著降低调参时间。

---

# 39. OpenCV 远距离检测参数分层

建议 Detector 分成：

```text
NEAR
MID
FAR
```

三个层级。

---

## FAR

目标：

```text
尽可能发现远处 Gate
```

允许：

```text
低 confidence
较小 contour
较弱白边
```

但只用于：

```text
Gate Chain 趋势。
```

不能直接控制穿门。

---

## MID

用于：

```text
稳定 Gate Tracking
```

---

## NEAR

用于：

```text
精确 Gate Center
Gate Pass 判断
```

---

# 40. Gate Confidence 随距离不同用途

```text
Far Gate:
只影响曲线趋势

Mid Gate:
参与 Curve Fit

Near Gate:
作为强约束
```

即：

```text
距离越近
Gate 权重越高
```

但不是：

```text
速度吸附。
```

而是：

```text
曲线拟合权重。
```

---

# 41. Weighted Curve Fit

例如：

```text
Gate 0:
距离 10m
weight = 1.0

Gate 1:
距离 30m
weight = 0.6

Gate 2:
距离 50m
weight = 0.3
```

近 Gate：

```text
决定精确穿门位置。
```

远 Gate：

```text
决定未来趋势。
```

---

# 42. Z 最低异常检测

为了防止：

```text
双目错误
→ 某 Gate Z 极低
→ 整条曲线被拉下去
```

必须新增：

```text
Z sanity check
```

一个 Gate Z 进入 Curve 前必须同时满足：

```text
1. 多帧稳定
2. Route s 合理
3. Depth 顺序合理
4. 邻近 Gate slope 合理
5. 世界坐标重复观测一致
```

任何一个失败：

```text
Gate Z 不作为硬控制点。
```

---

# 43. Gate Quality

建议每个 Gate 保存：

```yaml
id:
x:
y:
z:
s:

confidence:
support:

sigma_x:
sigma_y:
sigma_z:

depth:
bbox_area:

quality:
  route_order: true
  depth_order: true
  slope_valid: true
  stable: true
```

只有：

```text
quality 全部通过
```

才：

```text
hard_anchor = true
```

---

# 44. Soft Anchor / Hard Anchor

未来 Gate 分成：

```text
Soft Anchor
```

和：

```text
Hard Anchor
```

Far Gate：

```text
Soft
```

只引导趋势。

Near + 稳定 Gate：

```text
Hard
```

必须让曲线经过 Gate Center。

这样非常适合：

```text
远距离预判
+
近距离精确穿门。
```

---

# 45. 推荐 Curve Builder

输入：

```text
current_pose
XY Route
Gate Tracks
```

输出：

```text
future_reference_points:
[
P0,
P1,
P2,
...
]
```

再使用：

```text
Cubic B-Spline
```

或者：

```text
PCHIP for Z
+
Spline for XY
```

第一版建议：

```text
XY Route 保持现有
Z 用 PCHIP
Gate XY 作为小幅修正
```

这样改动最小。

---

# 46. 第一版实施顺序

## Step 1

取消：

```text
Gate 10m 内完全吸附。
```

恢复：

```text
连续 Path Following。
```

---

## Step 2

Gate Detector 改成：

```text
尽可能同时识别未来 3~5 个 Gate。
```

---

## Step 3

增加：

```text
Gate Chain Builder
```

约束：

```text
s 单调
depth 单调
bbox 大小大致单调
center 曲线连续
```

---

## Step 4

增加：

```text
Gate Track
```

避免单帧跳变。

---

## Step 5

对 Gate Z 增加：

```text
slope outlier rejection。
```

---

## Step 6

使用：

```text
未来 3~5 Gate
```

拟合：

```text
z_profile(s)
```

---

## Step 7

Look-ahead 改为：

```text
随速度变化。
```

---

## Step 8

提高：

```text
cruise_speed
```

到：

```text
4~6m/s
```

---

## Step 9

加入：

```text
curvature speed limit
+
Z slope speed limit
```

---

## Step 10

增加：

```text
start_gate / end_gate
```

用于快速局部测试。

---

# 47. 推荐当前参数

第一轮：

```text
Gate Detection Lookahead:
尽可能 40~60m
```

```text
Target Visible Gates:
3~5
```

```text
Control Gates:
nearest 2~3 valid gates
```

```text
Cruise Speed:
4.0 m/s
```

稳定后：

```text
5~6 m/s
```

```text
Min Speed:
2.0~2.5 m/s
```

只有：

```text
预测穿门失败
```

才降到更低。

```text
Lookahead:
L = 5 + 1.5*v
```

例如：

```text
v=4m/s
L≈11m
```

---

# 48. 新的最终控制结构

```text
Stereo Camera
     ↓
Rectangle Gate Detector
     ↓
Multi-Gate Tracker
     ↓
Gate Chain Builder
     ↓
Ordered Gate Sequence
     ↓
Gate Quality Filter
     ↓
Future 3D Reference Builder
     │
     ├── XY Route
     │
     └── Gate Z Profile
     ↓
Dynamic Look-ahead
     ↓
Nominal Velocity
     ↓
Speed Scheduler
(curvature + Z slope)
     ↓
Obstacle Avoidance
（当前 bypass）
     ↓
World → Body
     ↓
vel_body_cmd
```

---

# 49. 和上一版最大的不同

上一版：

```text
Gate 是局部吸引目标。
```

这一版：

> **Gate 是未来轨迹的控制点。**

上一版：

```text
越接近 Gate
→ 吸附越强。
```

这一版：

```text
越远的 Gate
→ 提供未来趋势

越近的 Gate
→ 提供精确控制点

所有 Gate
→ 一起决定一条连续轨迹。
```

---

# 50. 对 Z 问题的最终处理逻辑

不要再：

```text
某一 Gate 测到 z=-14
↓
立刻相信
↓
飞机一路向下。
```

而是：

```text
Gate Z
↓
多帧融合
↓
Gate Chain 顺序验证
↓
邻门 Z slope 检查
↓
异常值剔除
↓
未来多个 Gate 联合拟合
↓
连续 z_profile(s)
↓
无人机跟踪
```

这样可以同时解决：

```text
双目误匹配
Gate 顺序错误
Z 突变
最后飞太低
```

---

# 51. 对“矩形始终保持在前面”的实现

你的思路可以明确转化成：

> **维护一条始终位于当前无人机前方的 Ordered Gate Queue。**

例如：

```text
Current Pose
 ↓
Gate Queue:

G0  15m
G1  34m
G2  52m
G3  70m
```

每个 Gate 必须满足：

```text
camera_z > 0
path_s > current_s
```

而且：

```text
G0.s < G1.s < G2.s < G3.s
```

穿过 G0：

```text
pop G0
```

队列：

```text
G1
G2
G3
```

同时继续从远处：

```text
append G4
```

这样永远保持：

> **未来的一串 Gate 在无人机前方。**

这应该成为下一版 Gate Manager 的核心数据结构。

---

# 52. 最终验收标准

本阶段完成需要：

- 相机能同时稳定识别多个 Gate；
- 能维护未来 Gate Queue；
- Gate Queue 的 `s` 始终单调增加；
- 已经过的 Gate 不会重新进入 Queue；
- 错误 Gate Z 不会直接进入控制；
- 后半段不会因为单个错误 Z 持续下降；
- Z Profile 基于未来多个 Gate 生成；
- 飞机轨迹明显比吸附模式更平滑；
- 大多数 Gate 不需要明显减速；
- Cruise Speed 达到至少 4m/s；
- 可以局部启动测试任意 Gate 区间；
- Obstacle Avoidance 接口继续保留但当前 bypass；
- 后续深度学习只替换 Detector，不影响 Gate Chain 和 Planner。

---

# 53. 当前最终结论

下一版不要继续强化：

```text
Gate Attraction。
```

应该正式转向：

> **Multi-Gate Lookahead + Ordered Rectangle Chain + 3D Reference Curve。**

即：

```text
尽可能远地看到未来多扇矩形门
↓
利用 XY Route + 双目深度 + 透视几何
保证这些门按前后顺序排列
↓
把多个 Gate Center 当成未来轨迹控制点
↓
拟合平滑 3D Path
↓
无人机用 Look-ahead 连续穿过去
```

其中 Z 轴不再：

```text
跟着一个 Gate 突然变化
```

而是：

```text
未来多个 Gate 联合决定连续高度曲线。
```

速度策略也改成：

```text
默认高速
只有曲率大 / Z 坡度大 / 预测穿门失败时才减速。
```

这比上一版“门前吸附 + 低速对准”更符合当前赛道大量连续矩形 Gate 的结构，也更利于后续直接接入局部避障。
