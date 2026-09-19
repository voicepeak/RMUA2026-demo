# RMUA 2026：启动高度跳变与 Gate 6 高度认知错误修复方案

## 0. 本轮问题结论

根据最新日志，当前出现的是两个不同层级的问题：

```text
问题 A：
启动瞬间 z_ref 与当前实际高度相差过大
→ 一启动就给满 vz
→ 飞机原地猛升
→ 撞到起点上方结构
```

以及：

```text
问题 B：
Gate 6 附近实际高度误差已经很大
但 vz_cmd 始终不高，且 Z_LIMIT=NONE
→ 说明不是“垂直能力不够”
→ 而是上游 Planner 没认为应该更快爬升
```

因此本轮不继续盲目放大：

```text
vz_up_limit
vz_accel_limit
Kz
```

而是集中修：

```text
① 启动高度初始化
② Gate 6 的真实 Z 来源
③ Gate Map / YAML / Vision 的优先级
④ Altitude Profile 是否生成了正确的 z_ref / dz_ds
⑤ Camera → World 的时间同步与姿态补偿
```

---

# 1. 问题 A：启动瞬间高度跳变

最新日志中：

```text
当前飞机：
z ≈ 0.31

start_anchor:
z = -4.0
```

于是：

\[
e_z
=
0.31 - (-4.0)
=
4.31
\]

在当前较大的：

```text
Kz
+
vz_up_limit
```

下，控制器立刻：

```text
vz_cmd ≈ 4m/s
```

飞机还没向前走：

```text
就先原地向上猛冲。
```

这不是垂直能力问题。

这是：

> **初始参考高度设置错误。**

---

# 2. 启动时 start_anchor.z 必须来自当前实际高度

旧逻辑：

```text
start_anchor.z
=
固定值 / 预设值
```

必须改为：

```text
start_anchor.z
=
current_pose.z
```

也就是：

```python
start_anchor_z = current_pose.position.z
```

启动阶段：

```text
飞机现在在哪个高度
↓
Z Profile 就从这个高度开始
```

不能要求飞机：

```text
一启动就先跳到某个预设高度。
```

---

# 3. start_anchor 的正确初始化流程

推荐：

```text
节点启动
↓
等待 pose 有效
↓
读取 current_pose.z
↓
建立 start_anchor
↓
再建立后续 Gate Anchor
↓
生成 z_profile
↓
开始运动
```

伪代码：

```python
if not start_anchor_initialized:

    if pose_valid:

        start_anchor.s = current_s
        start_anchor.z = current_pose.z

        start_anchor_initialized = True
```

---

# 4. 启动阶段不要立刻使用第一道 Gate 强拉高度

即使 Gate 0 比当前高度高 / 低很多，

也不要：

```text
start
↓
马上追 Gate0.z
```

应该形成：

```text
current_z
↓
平滑过渡
↓
Gate0.z
```

---

# 5. Start → Gate0 生成专门的平滑段

定义：

```text
Start Anchor:
(s0, z_current)

Gate0:
(s1, z_gate0)
```

使用：

```text
PCHIP / Smoothstep
```

生成：

```text
z(s)
```

避免：

```text
z_ref 突然跳变。
```

---

# 6. 启动保护

启动前：

```text
如果 |z_ref - z_actual| > startup_z_jump_limit
```

不要：

```text
直接给满 vz。
```

推荐：

```text
startup_z_jump_limit = 0.5 ~ 1.0m
```

如果超过：

```text
重新初始化 start_anchor
```

而不是：

```text
强制飞向错误参考高度。
```

---

# 7. 问题 B：Gate 6 MISS 不是拉升能力不足

最新日志关键点：

```text
Gate 6:
vert ≈ -2.0m

vz_cmd 全程：
基本 <= 1.02m/s

Z_LIMIT:
NONE
```

这说明：

```text
飞行器没有请求很大的向上速度。
```

因此不是：

```text
vz_cmd 被 3m/s / 4m/s clamp 卡住。
```

而是：

> **Planner 本身没有认为这里需要更大的垂直速度。**

---

# 8. 当前真正需要排查的链路

Gate 6 的高度应该经过：

```text
YOLO / Stereo
↓
Gate Observation
↓
Camera XYZ
↓
Camera → Body → World
↓
Persistent Gate Map
↓
Gate Chain
↓
Altitude Profile
↓
z_ref
↓
dz/ds
↓
vz_ff
↓
vz_fb
↓
vz_target
↓
vz_cmd
```

现在很可能是其中某一步：

```text
给了错误的 Gate 6 高度
```

或者：

```text
Altitude Profile 没有真正使用正确 Gate 6 Z。
```

---

# 9. Gate 6 调试必须一次性打印完整链路

到 Gate 6 前：

```text
必须同时输出：
```

```text
Gate6.visual_raw_z
Gate6.camera_z
Gate6.world_z

Gate6.map_z
Gate6.yaml_z

Gate6.source
Gate6.hard_anchor
Gate6.sigma_z
Gate6.support

z_ref_raw
z_ref_final

dz_ds_current
dz_ds_preview

vz_ff
vz_fb
vz_target
vz_cmd

z_actual
z_error
```

不能只看：

```text
最终 vz_cmd。
```

---

# 10. 第一种可能：Gate Map / YAML 高度就是错的

例如：

```text
视觉真实：
Gate6.world_z = -8.0
```

但 Planner 实际使用：

```text
Gate6.map_z = -6.0
```

或者：

```text
Gate6.yaml_z = -6.0
```

那么：

```text
z_ref
```

自然就不会要求飞机爬到：

```text
-8.0。
```

---

# 11. Gate 数据源优先级必须明确

建议统一：

```text
1. Hard Persistent Vision Gate
2. Verified Static Gate
3. Soft Vision Gate
4. Fallback Route
```

不能出现：

```text
视觉已经有真实 Gate
但 Planner 仍悄悄用旧 YAML。
```

---

# 12. 每个 Gate 必须记录 source

Gate 数据结构增加：

```yaml
source:
  persistent_vision
  static_yaml
  soft_vision
  fallback
```

运行日志必须打印：

```text
Gate6 source = persistent_vision
```

否则很难判断控制器到底用了谁。

---

# 13. use_gate_map 必须显式开启

启动时：

```text
use_gate_map = true
```

不要依赖默认值。

建议启动打印：

```text
[GATE_MAP] ENABLED
```

如果：

```text
false
```

直接 Warning：

```text
[WARN] Persistent Gate Map disabled
```

---

# 14. 第二种可能：Gate 6 World Z 在转弯 / 姿态变化时漂移

如果 Gate 6 的视觉 Z 本身不稳定，

那么可能出现：

```text
远处：
z = -8.0

转弯后：
z = -6.5

再靠近：
z = -7.0
```

这样 Persistent Gate Map 会：

```text
把错误 Z 融进去。
```

---

# 15. Camera → World 必须确认完整 Quaternion

转换必须：

```text
Camera
↓
Body
↓
World
```

使用：

```text
Roll
Pitch
Yaw
```

完整姿态。

不能只使用：

```text
Yaw。
```

---

# 16. 高速时必须时间同步

尤其：

```text
10m/s
```

如果图像和 IMU / pose 相差：

```text
50ms
```

飞机已经移动：

```text
0.5m
```

而且转弯 / pitch 中：

```text
姿态也发生变化。
```

所以必须：

```text
Image Timestamp
↓
找到同一时刻 Pose / IMU
↓
Position 插值
Quaternion SLERP
↓
Camera → World
```

---

# 17. Gate 6 必做 World Z 稳定性测试

固定看同一 Gate 6。

让飞机：

```text
直线
转弯
加速
减速
```

记录：

```text
Gate6.world_z
roll
pitch
yaw
timestamp
```

理想：

```text
Gate6.world_z
基本稳定。
```

例如：

```text
-8.03
-8.08
-7.99
-8.04
```

如果变成：

```text
-8.0
-7.2
-6.3
-5.8
```

则：

```text
先修坐标变换 / 时间同步
```

不要再调 Z Controller。

---

# 18. 第三种可能：Altitude Profile 没有提前形成正确坡度

即使：

```text
Gate6.z 正确
```

也可能：

```text
z_ref 最终能到 Gate6.z
```

但：

```text
前面几十米 dz/ds 太小。
```

结果：

```text
vz_ff 一直很小
```

到了 Gate 附近才发现高度不够。

---

# 19. Gate6 前必须看 z_ref / dz_ds 曲线

重点检查：

```text
Gate5
↓
Gate6
```

这一段：

```text
z_ref(s)
dz/ds(s)
```

是不是合理。

如果：

```text
前面 20m
dz/ds ≈ 0
```

最后 5m 才突然变大，

那就是：

```text
Altitude Profile 太晚。
```

---

# 20. Altitude Profile 应该提前从上一 Gate 开始过渡

正确：

```text
Gate5 穿过
↓
马上开始向 Gate6 高度变化
↓
到 Gate6 前已经接近目标高度
```

错误：

```text
大部分区间维持 Gate5 高度
↓
靠近 Gate6
↓
才快速修改 z。
```

---

# 21. 推荐 Gate 间使用 PCHIP / Smooth Spline

输入：

```text
(s_i, z_i)
```

生成：

```text
z(s)
```

要求：

```text
连续
平滑
不过冲
提前变化
```

推荐：

```text
PCHIP
```

优先于普通 cubic spline。

---

# 22. dz/ds 应来自真正的 z_profile

Feedforward：

\[
v_{z,ff}
=
-K_{ff}
\frac{dz}{ds}
v_s
\]

其中：

```text
dz/ds
```

必须是：

```text
真实 z_profile 的导数。
```

不能来自：

```text
单独 Gate 差分
```

或：

```text
旧缓存值。
```

---

# 23. 建议打印当前 anchor pair

例如：

```text
ALT_PROFILE:
prev_gate=5
next_gate=6

prev_z=-7.2
next_z=-8.0
```

如果 Gate 6 已经当前目标，

但仍然打印：

```text
next_gate=5
```

说明：

```text
Gate Chain / Progress
```

切换有问题。

---

# 24. Gate Index / Route Progress 要确认同步

Gate 6 MISS 也可能不是高度数据本身错，

而是：

```text
Planner 还在跟前一个 Gate。
```

必须打印：

```text
current_s
next_gate_id
next_gate_s
```

检查：

```text
Gate 6 前
next_gate_id 是否真的已经是 6。
```

---

# 25. 不要因为 Gate Pass / Miss 太早切换下一 Gate

如果：

```text
Gate5
```

刚过，

不要：

```text
马上跳过 Gate6
```

或：

```text
Gate6 还没真正进入有效区
就切 Gate7。
```

Gate 状态必须和：

```text
Path Progress
```

一致。

---

# 26. Gate6 MISS 必须拆成“规划错”还是“控制错”

建立明确诊断：

## 情况 A

```text
Gate6 z 错
```

→ 感知 / Map 问题。

## 情况 B

```text
Gate6 z 对
z_ref 错
```

→ Altitude Profile 问题。

## 情况 C

```text
z_ref 对
dz/ds 太小
```

→ 曲线生成 / Preview 问题。

## 情况 D

```text
z_ref 对
dz/ds 对
vz_target 大
vz_cmd 小
```

→ Clamp / Rate Limit 问题。

## 情况 E

```text
vz_cmd 大
actual climb 小
```

→ VEL 飞控 / 动力学问题。

目前这次日志更像：

```text
A / B / C
```

而不是：

```text
D / E。
```

---

# 27. 当前不要继续提高 vz_up_limit

这次：

```text
vz_cmd <= 1.02
```

且：

```text
Z_LIMIT=NONE
```

说明：

```text
当前不是上限卡住。
```

所以先不要继续：

```text
4 → 5 → 6
```

这样没有意义。

---

# 28. 当前 Kz 也不是第一优先级

如果：

```text
z_ref 本身就不对
```

那么：

```text
Kz 越大
```

只会：

```text
更坚定地追错误高度。
```

所以先修：

```text
z_ref / Gate Z。
```

---

# 29. 建议新增 Debug 模式：Gate6 Trace

新增：

```text
--trace_gate 6
```

当进入 Gate 6 前：

```text
开始高频日志。
```

例如：

```text
10 Hz
```

记录完整链路。

通过 Gate 6 后：

```text
停止。
```

这样不会让全局日志太乱。

---

# 30. Gate6 Trace 推荐日志格式

```text
[G6]

s=...

visual_z=...
world_z=...
map_z=...
yaml_z=...

source=...
hard=...
sigma_z=...

z_ref_raw=...
z_ref=...
z_actual=...

dzds=...
dzds_preview=...

vz_ff=...
vz_fb=...
vz_target=...
vz_cmd=...

next_gate=...
prev_gate=...
```

---

# 31. 建议同时输出 CSV

CSV：

```text
gate6_trace.csv
```

列：

```text
time
s

visual_z
world_z
map_z
yaml_z

z_ref_raw
z_ref
z_actual

dz_ds
dz_ds_preview

vz_ff
vz_fb
vz_target
vz_cmd

roll
pitch
yaw

source
hard_anchor
sigma_z
```

---

# 32. 一张图就能快速定位问题

横轴：

```text
Path Progress s
```

纵轴：

```text
Z
```

画：

```text
Gate5 Z
Gate6 Z
Gate7 Z

Map Gate Z

z_ref
z_actual
```

如果：

```text
Gate6 Map Z 就不对
```

一眼可见。

如果：

```text
Map Z 对
z_ref 曲线没过去
```

也一眼可见。

---

# 33. 第二张图：Z 控制链

横轴：

```text
time
```

画：

```text
vz_ff
vz_fb
vz_target
vz_cmd
```

如果：

```text
ff≈0
```

说明坡度没生成。

如果：

```text
target大
cmd小
```

才是限幅问题。

---

# 34. 第三张图：姿态与 Gate Z

画：

```text
Gate6.world_z
pitch
roll
yaw
```

如果：

```text
Gate6.world_z
```

随着：

```text
pitch / yaw
```

同步变化，

则：

```text
Camera → World
```

有问题。

---

# 35. 修复顺序

## Step 1

修：

```text
start_anchor.z = current_pose.z
```

解决：

```text
启动原地猛升。
```

---

## Step 2

确认：

```text
use_gate_map = true
```

并打印：

```text
Gate 数据 source。
```

---

## Step 3

给 Gate6 打完整 Trace。

先不改控制参数。

---

## Step 4

检查：

```text
visual_z
world_z
map_z
yaml_z
```

是否一致。

---

## Step 5

如果：

```text
world_z 漂
```

修：

```text
完整 Quaternion
+
时间同步。
```

---

## Step 6

如果：

```text
world_z 对
map_z 错
```

修：

```text
Tracker / EMA / Data Source Priority。
```

---

## Step 7

如果：

```text
map_z 对
z_ref 错
```

修：

```text
Altitude Profile。
```

---

## Step 8

如果：

```text
z_ref 对
dz/ds 太小
```

修：

```text
PCHIP / Preview / Gate Anchor 顺序。
```

---

## Step 9

如果：

```text
vz_target 很大
vz_cmd 被截
```

再回来调：

```text
vz limit。
```

---

## Step 10

如果：

```text
vz_cmd 已很大
但实际爬升不足
```

才做：

```text
VEL Capability Calibration / PWM。
```

---

# 36. 启动高度保护建议

增加：

```text
startup_mode
```

节点启动后：

```text
WAIT_POSE
↓
INIT_START_ANCHOR
↓
BUILD_PROFILE
↓
RUN
```

不要：

```text
程序启动第一帧就 RUN。
```

---

# 37. WAIT_POSE 条件

必须满足：

```text
pose received
pose timestamp fresh
z finite
quaternion valid
```

才进入：

```text
INIT_START_ANCHOR。
```

---

# 38. INIT_START_ANCHOR

记录：

```text
start_s
start_x
start_y
start_z = current_z
```

然后：

```text
build altitude profile。
```

---

# 39. 启动 Profile sanity check

生成 Profile 后：

```text
检查：

|z_profile(start_s) - current_z|
```

要求：

```text
< 0.2~0.5m
```

如果超过：

```text
拒绝 RUN。
```

重新初始化。

---

# 40. Gate6 Hard Anchor 条件建议

Gate 6 进入真正 Z 控制前：

```text
support > threshold
sigma_z < threshold
world_z stable
source == persistent_vision
route order valid
timestamp valid
```

否则：

```text
Soft Anchor。
```

---

# 41. Soft Anchor 不能强制 Z

Soft Gate：

```text
只提供趋势。
```

不能：

```text
单独决定 z_ref。
```

如果 Gate 6 当前仍是 Soft：

```text
z_profile 应主要由前后 Hard Gate 平滑估计。
```

---

# 42. 对 Gate6 这种关键坡段增加二次确认

如果：

```text
|Δz/Δs|
```

明显比前后坡度大，

则：

```text
Gate6/7
```

必须：

```text
重新观测
+
多帧确认。
```

避免：

```text
单个错误 Z
```

造成整个坡度错误。

---

# 43. 暂时不要动的东西

当前不要再同时改：

```text
YOLO 网络
LiDAR
PWM
复杂避障
速度 10m/s
```

先用：

```text
3~5m/s
```

把：

```text
Gate6 Z 链路
```

查清楚。

---

# 44. Gate6 修好以后再恢复高速

顺序：

```text
3m/s
↓
5m/s
↓
7m/s
↓
10m/s
```

每一级确认：

```text
Gate6 world_z
z_ref
vz_ff
实际通过高度
```

都正常。

---

# 45. 本阶段验收标准

### 启动

- 启动后不再原地猛升；
- `start_anchor.z` 等于当前实际高度；
- 第一帧 `z_error` 不再是数米级；
- 起点附近不再撞上方结构。

### Gate6

- Gate6 的 `visual/world/map/yaml Z` 来源可追踪；
- Persistent Gate Map 明确参与控制；
- Gate6 World Z 在转弯时保持稳定；
- `z_ref` 能正确接近 Gate6 Z；
- `dz/ds` 在 Gate6 前提前形成；
- `vz_ff` 在进入坡前出现；
- 不再出现“高度差 2m，但 vz_cmd 只有 1m/s 左右且 Planner 无感知”的情况。

---

# 46. 最终判断逻辑

以后任何“拉不上去”的问题都按下面顺序判断：

```text
Gate Z 对吗？
↓
Map Z 对吗？
↓
z_ref 对吗？
↓
dz/ds 对吗？
↓
vz_target 对吗？
↓
vz_cmd 被限制了吗？
↓
实际 dz/dt 跟得上吗？
```

不要再直接：

```text
“拉不上去”
↓
调大 vz limit。
```

---

# 47. 一句话总结

这次最新日志说明：

> **当前有两个独立 Bug：启动参考高度错误，以及 Gate 6 附近 Planner 对真实高度需求认知不足。**

启动问题：

```text
start_anchor.z
必须等于当前实际 z。
```

Gate6 问题：

```text
不要再先调垂直能力，
而要追踪：

Vision Z
→ World Z
→ Map Z
→ z_ref
→ dz/ds
→ vz_ff
```

只有把这条链路查清楚，才能判断真正的下一步到底是：

```text
感知
地图
轨迹
控制
还是飞控能力。
```
