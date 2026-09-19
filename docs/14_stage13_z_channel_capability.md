# RMUA 2026：最新版 Z 通道解限、垂直能力标定与陡坡高速通过方案

## 0. 当前状态

最新版已经完成：

- Yaw 前视控制已经真正接入 `VelCmd.yawRate`；
- 转弯时不再固定 `yaw_rate = 0`；
- Speed Scheduler 已从“门前误差大就直接降到极低速”改成更合理的多约束调度；
- Persistent Gate Map、Gate Chain、连续 3D Reference、Z Feedforward 等框架已经建立；
- 当前主要剩余问题集中在：

```text
陡坡 / 快速拉高时：
Z 跟不上
↓
实际飞行高度落后
↓
飞机从坡下方掉出合法区域
↓
任务结束
```

因此本阶段不再大改规划框架，而是集中解决：

> **Z 通道软件限幅过于保守、实际垂直能力未知、Slope Speed Limit 使用假定能力值这三个问题。**

---

# 1. 当前最可疑的 Z 通道瓶颈

当前 Z 控制链大致为：

```text
Gate / 3D Path
↓
Altitude Profile
↓
z_ref
↓
Z Feedforward + Feedback
↓
vz_target
↓
vz clamp
↓
vz rate limit
↓
vel_body_cmd
```

目前至少存在以下几层限制：

```text
z_rate_max ≈ 1.5 m/s

vz_up_limit ≈ 3.0 m/s

vz_accel_limit ≈ 3.0 m/s²

Kz ≈ 0.6

vz_up_safe ≈ 3.0 m/s
```

其中：

```text
vz_up_safe
```

还是人为假定值，而不是实测能力。

---

# 2. 第一问题：z_ref 自己变化得太慢

当前 `AltitudeProfile` 对：

```text
z_ref
```

做了 Rate Limit。

类似：

```python
md = z_rate_max * dt

z_ref =
clamp(
    z_ref_target,
    prev_z_ref - md,
    prev_z_ref + md
)
```

如果：

```text
z_rate_max = 1.5 m/s
```

意味着：

> 即使未来赛道高度要求快速变化，参考高度本身每秒也只能变化 1.5m。

对于缓坡没问题。

但对于：

```text
短距离内高度变化数米
```

的陡坡：

```text
z_ref 本身就会落后。
```

---

# 3. z_rate_max 与 Z Feedforward 存在潜在矛盾

当前控制又有：

```text
vz_ff
=
-dz/ds * v
```

例如未来坡度理论要求：

```text
vz_ff = 3 m/s
```

但：

```text
z_ref
```

仍然以：

```text
1.5 m/s
```

速度缓慢移动。

结果就是：

```text
Feedforward 想快速爬升
↓
Feedback 看到的 z_ref 却还在后面
↓
两个模块追的不是完全一致的时间轨迹
```

这会使 Z 控制变得不够直观。

---

# 4. 第一阶段建议：放开 z_rate_max

建议首轮改：

```text
z_rate_max:
1.5 → 4.0 m/s
```

如果测试稳定，可以继续：

```text
4.0 → 5.0 m/s
```

甚至后续考虑：

```text
取消这个二次 Rate Limit
```

前提是：

```text
z_profile(s)
```

本身已经经过：

```text
PCHIP / Smooth Curve
```

处理。

---

# 5. 第二问题：vz_up_limit 太保守

当前：

```text
vz_up_limit ≈ 3.0 m/s
```

这意味着：

```text
不管路径要求多大垂直速度
```

最终：

```text
vz_cmd <= 3m/s
```

如果某一段：

```text
10m/s 前进
+
坡度 |dz/ds| ≈ 0.3
```

理论要求：

\[
|v_z|
\approx
10 \times 0.3
=
3m/s
\]

这已经刚好碰到上限。

再考虑：

```text
控制滞后
姿态耦合
速度跟踪误差
```

就必然留不出余量。

---

# 6. 建议放开 vz_up_limit

第一轮：

```text
vz_up_limit:
3.0 → 4.0 m/s
```

如果实际 VEL 模式能稳定做到：

```text
4m/s
```

再尝试：

```text
4.5 ~ 5.0 m/s
```

注意：

> 这里先是“放开软件上限”，不是默认认为飞机一定能稳定达到 5m/s。

真正能力要通过实测得到。

---

# 7. 第三问题：vz_accel_limit 太低

当前：

```text
vz_accel_limit ≈ 3.0 m/s²
```

意味着：

```text
0 → 3m/s
```

大约需要：

```text
1s
```

如果陡坡出现得很快，

可能发生：

```text
规划器已经知道要爬升
↓
vz_target 已经很大
↓
但 rate limit 仍然让实际 vz_cmd 慢慢增加
↓
进入坡后才真正拉起来
```

高速下这会非常明显。

---

# 8. 建议提高垂直加速度限幅

第一轮：

```text
vz_accel_limit:
3.0 → 6.0 m/s²
```

若稳定：

```text
6.0 → 8.0 m/s²
```

目的：

> **让垂直前馈可以更快建立。**

---

# 9. 第四问题：Kz = 0.6 偏弱

当前反馈：

\[
v_{z,fb}
=
K_z e_z
\]

如果：

```text
Kz = 0.6
```

高度落后：

```text
1m
```

反馈只增加：

```text
0.6m/s
```

对于高速陡坡恢复能力偏弱。

---

# 10. 建议提高 Kz，但不要让 Feedback 成为主力

首轮：

```text
Kz:
0.6 → 1.0
```

再观察：

```text
Z overshoot
Z oscillation
```

如果稳定，可尝试：

```text
1.2 ~ 1.5
```

但控制原则仍然是：

```text
Feedforward：
负责提前爬升

Feedback：
负责纠正残余误差
```

而不是：

```text
等掉高以后全靠 P 拉回来。
```

---

# 11. 建议增加 Z Feedforward Gain

当前：

\[
v_{z,ff}
=
-\frac{dz}{ds}v
\]

可以加入：

\[
K_{ff,z}
\]

变成：

\[
v_{z,ff}
=
-K_{ff,z}
\frac{dz}{ds}v
\]

推荐首轮：

```text
Kff_z = 1.15 ~ 1.2
```

如果仍存在明显爬升滞后：

```text
1.3
```

短期测试甚至可以：

```text
1.4
```

但不建议一开始就过大。

---

# 12. 为什么 Kff > 1 有意义

理论路径要求：

```text
vz = 3m/s
```

但基础 VEL 控制可能存在：

```text
响应滞后
实际速度跟踪不足
姿态耦合
```

因此：

```text
Kff_z = 1.2
```

会提前给：

```text
3.6m/s
```

命令。

靠：

```text
z feedback
```

在接近正确高度后收回来。

---

# 13. 现在最关键的不是继续猜，而是做垂直能力标定

当前：

```text
vz_up_safe = 3m/s
```

只是一个假定值。

Speed Scheduler 用它计算：

\[
v_{slope}
=
\eta
\frac{v_{z,up,safe}}
{|dz/ds|}
\]

如果这个值不是真实能力：

```text
整个坡度限速都是错的。
```

---

# 14. 必须建立真实 vz_available(vxy)

目标：

> 测出不同水平速度下，无人机真正能稳定获得多少垂直速度。

测试矩阵：

| XY Speed | vz_cmd |
|---:|---:|
| 0 | 1 |
| 0 | 2 |
| 0 | 3 |
| 0 | 4 |
| 0 | 5 |
| 5 | 1 |
| 5 | 2 |
| 5 | 3 |
| 5 | 4 |
| 5 | 5 |
| 10 | 1 |
| 10 | 2 |
| 10 | 3 |
| 10 | 4 |
| 10 | 5 |

记录：

```text
实际 dz/dt
```

---

# 15. 建议单独实现 Z Capability Probe

新增：

```text
z_capability_probe.py
```

功能：

```text
固定一个水平速度
↓
依次发送多个 vz_cmd
↓
每个档位保持若干秒
↓
记录 pose.z
↓
计算实际垂直速度
↓
自动输出 CSV
```

---

# 16. 每个档位建议测试流程

例如：

```text
vx = 5m/s
vz_cmd = 3m/s
```

执行：

```text
Ramp in:
1s

Stable:
2~3s

Ramp out:
1s
```

取：

```text
中间稳定段
```

估计：

\[
v_{z,actual}
=
-\frac{dz}{dt}
\]

注意：

```text
pose_gt.z:
NED，向下为正
```

所以：

```text
实际上升速度
=
-dz/dt
```

---

# 17. 最终得到能力表

例如测试结果可能是：

| XY Speed | vz_cmd | actual climb |
|---:|---:|---:|
| 0 | 4 | 3.9 |
| 5 | 4 | 3.6 |
| 10 | 4 | 2.8 |
| 10 | 5 | 3.1 |

然后拟合：

```text
vz_available(vxy)
```

---

# 18. 推荐能力模型

第一版不用复杂。

可直接做：

```text
piecewise lookup table
```

例如：

```yaml
vz_available:
  - vxy: 0
    vz_up: 4.0

  - vxy: 5
    vz_up: 3.6

  - vxy: 10
    vz_up: 3.0
```

中间：

```text
线性插值。
```

---

# 19. Speed Scheduler 不再使用固定 vz_up_safe

旧：

```text
vz_up_safe = 3.0
```

新：

```python
vz_safe = vz_available(current_vxy)
```

然后：

\[
v_{slope}
=
\eta
\frac{v_{z,safe}}
{|dz/ds|+\epsilon}
\]

---

# 20. 这样 10m/s 才能真正合理使用

目标：

```text
v_cruise = 10m/s
```

如果坡度很缓：

```text
v_slope > 10
```

则：

```text
保持 10m/s。
```

如果坡度较陡：

```text
v_slope = 7.5
```

则：

```text
自动降到 7.5m/s。
```

如果再陡：

```text
v_slope = 5
```

则：

```text
降到 5m/s。
```

但始终：

```text
边走边爬
```

而不是：

```text
停车上升。
```

---

# 21. 爬升期间禁止触发停车策略

继续保持：

```text
NORMAL / TRACKING
```

逻辑：

```text
Z error 小
→ 正常速度

Z error 中等
→ 适当降速

Z error 较大
→ 进一步降速

但正常情况不设 XY=0
```

只有：

```text
Emergency
```

才允许：

```text
停止前进。
```

---

# 22. 建议新的 Z Error Speed Scale

例如：

```text
|z_err| < 0.3m
→ 1.0×

0.3~0.6m
→ 0.9×

0.6~1.0m
→ 0.75×

1.0~1.5m
→ 0.6×

>1.5m
→ 0.5×
```

而不是：

```text
误差大
→ 0m/s。
```

---

# 23. 对陡坡要增加 Preview

即使放开 Z 通道，

高速时还是应该提前开始爬升。

建议：

```text
Z Preview Distance
=
10 ~ 30m
```

随速度变化。

例如：

\[
L_z
=
L_0
+
T_zv
\]

推荐：

```text
L0 = 8m
Tz = 1.5~2.0s
```

如果：

```text
v = 10m/s
```

则：

```text
Lz ≈ 23~28m
```

---

# 24. Feedforward 应使用 Preview Slope

不要只使用：

```text
当前 s 的 dz/ds
```

改为：

```text
s + L_slope
```

附近的未来坡度。

例如：

```text
L_slope = 5~15m
```

这样：

```text
还没进入坡
↓
vz_ff 已经开始建立
```

---

# 25. 不建议 z_ref 直接跳到未来高度

避免：

```text
z_ref = z(s+25m)
```

这种直接提前跳目标。

推荐：

```text
z_ref:
仍然对应当前路径位置

vz_ff:
使用未来坡度
```

这样：

```text
位置参考不乱
+
速度前馈提前动作
```

---

# 26. 当前 Persistent Gate Map 必须确认是否真正启用

当前代码已经支持：

```text
Persistent Gate Map
```

但启动参数必须检查：

```text
use_gate_map = true
```

否则：

```text
最新 YOLO / Gate Map
```

虽然功能写好了，

实际 Planner 可能仍然主要使用：

```text
静态 YAML Gate。
```

---

# 27. 正式运行建议显式设置

建议 launch 明确：

```text
use_gate_map := true
```

并输出启动日志：

```text
[GateMap] ENABLED
```

不要只靠默认值。

---

# 28. Gate Map 与 Z Planner 的输入优先级

建议：

```text
1. Hard Persistent Gate
2. Static verified Gate
3. Soft Gate
4. Route fallback
```

错误 / 不稳定 Gate：

```text
不能直接拉动 Z。
```

---

# 29. 当前阶段不要继续大改视觉

Yaw：

```text
已经接通。
```

Speed Scheduler：

```text
结构已经合理很多。
```

Gate Map：

```text
框架已经有。
```

因此当前第一优先级：

> **测清楚 Z 通道的真实能力。**

不要同时再改：

```text
LiDAR
Gate Detector
复杂局部规划
```

否则问题会变得难定位。

---

# 30. 第一轮参数建议

建议先测试：

```text
z_rate_max = 4.0 m/s
```

```text
vz_up_limit = 4.0 m/s
```

```text
vz_accel_limit = 6.0 m/s²
```

```text
Kz = 1.0
```

```text
Kff_z = 1.2
```

```text
v_cruise = 10 m/s
```

但：

```text
v_slope
```

仍然允许自动把陡坡速度降下来。

---

# 31. 第二轮参数

如果第一轮稳定：

```text
z_rate_max = 5.0
```

```text
vz_up_limit = 4.5 ~ 5.0
```

```text
vz_accel_limit = 8.0
```

```text
Kz = 1.2
```

```text
Kff_z = 1.2 ~ 1.3
```

---

# 32. 不建议一开始就做的参数

不要直接：

```text
Kz = 3
vz_up_limit = 8
vz_accel_limit = 15
```

否则容易：

```text
Z overshoot
Pitch / attitude disturbance
Gate FOV 波动
上下振荡
```

应逐级放开。

---

# 33. 建议专门测试最陡坡段

继续使用：

```text
start_gate
end_gate
start_s
```

直接从：

```text
陡坡前 20~30m
```

开始。

不要每次完整跑全场。

---

# 34. Test A：只验证 z_rate_max

保持：

```text
v = 4~5m/s
```

只修改：

```text
z_rate_max:
1.5 → 4
```

观察：

```text
z_ref 是否不再明显落后。
```

---

# 35. Test B：验证 vz limit

再改：

```text
vz_up_limit:
3 → 4
```

观察：

```text
vz_cmd 是否真的超过 3m/s。
```

---

# 36. Test C：验证 vz accel

再改：

```text
vz_accel_limit:
3 → 6
```

观察：

```text
坡前 vz 是否建立得更快。
```

---

# 37. Test D：验证 Kff

加入：

```text
Kff_z = 1.2
```

检查：

```text
是否能在进入坡之前提前向上。
```

---

# 38. Test E：最后再上 10m/s

顺序：

```text
5m/s
↓
7m/s
↓
8m/s
↓
10m/s
```

每次记录：

```text
z_ref
z_actual
vz_cmd
actual climb
v_slope
```

---

# 39. 必须记录的日志

建议每：

```text
0.1 ~ 0.2s
```

记录：

```text
time

path_s
v_xy_cmd
v_xy_actual

z_target_raw
z_ref

z_actual
z_error

dz_ds_current
dz_ds_preview

vz_ff_raw
vz_ff_gain
vz_fb

vz_target
vz_after_clamp
vz_cmd

actual_dz_dt

vz_available_estimate

v_slope
v_curve
v_tracking

final_speed
final_cap_reason
```

---

# 40. 最关键的一条日志：Z LIMIT REASON

建议输出：

```text
Z_LIMIT=NONE
```

或者：

```text
Z_LIMIT=Z_RATE_MAX
```

```text
Z_LIMIT=VZ_UP_LIMIT
```

```text
Z_LIMIT=VZ_ACCEL
```

这样看到拉不上去时，

立即知道到底：

```text
哪个环节被卡住。
```

---

# 41. 推荐打印格式

例如：

```text
[Z]
s=174.2
z_ref=-9.1
z=-8.2
err=0.9

dzds=-0.28

ff=2.80
fb=0.90
target=3.70

clamp=3.70
cmd=3.25

actual=2.45

limit=VZ_ACCEL
```

这样非常容易调试。

---

# 42. 如果软件放开后仍然拉不上去

如果：

```text
z_rate_max 已经足够大
vz_up_limit 已经足够大
vz_accel_limit 也足够
Kff 也有
```

但：

```text
实际 dz/dt
```

仍然明显低于命令，

说明瓶颈已经不是：

```text
Planner / Controller。
```

而是：

```text
VEL 基础飞控 / 机体动力学。
```

---

# 43. 判断是否需要切 PWM

例如：

```text
vx = 0
vz_cmd = 5
```

实际只有：

```text
2m/s
```

或者：

```text
vx = 10
vz_cmd = 5
```

实际长期只能：

```text
1.5~2m/s
```

且软件层已经确认没有 clamp，

则：

> **继续调 VEL 意义不大。**

下一阶段应考虑：

```text
PWM Controller
```

---

# 44. PWM 阶段暂时只预留

当前不马上切 PWM。

只有完成：

```text
VEL 垂直能力标定
```

以后再决定。

判断标准：

```text
如果 VEL 能满足大部分坡度：
继续 VEL

如果 VEL 明显成为性能瓶颈：
进入 PWM。
```

---

# 45. 最终 Z 控制结构

```text
Persistent Gate Map
        ↓
3D Reference Curve
        ↓
z(s), dz/ds
        │
        ├─────────────┐
        ↓             ↓
Current z_ref     Preview slope
        ↓             ↓
Z Feedback       Z Feedforward
        │             │
        └──────┬──────┘
               ↓
          Kff / Kz
               ↓
          vz_target
               ↓
      Physical VZ Limit
               ↓
      Vertical Accel Limit
               ↓
           vz_cmd
               ↓
        vel_body_cmd
```

同时：

```text
Measured vz_available(vxy)
        ↓
Speed Scheduler
        ↓
v_slope
        ↓
XY Speed
```

---

# 46. 本阶段验收标准

认为本阶段完成，需要满足：

- `z_ref` 不再被 1.5m/s 的旧限制明显拖后；
- `vz_cmd` 可以按需要超过 3m/s；
- `vz` 能更快建立；
- Feedforward 在坡前开始起作用；
- 正常陡坡不再出现“门前停住再升”；
- 能测得真实 `vz_available(vxy)`；
- Speed Scheduler 使用实测垂直能力，而不是固定 `vz_up_safe=3`；
- 平缓区仍可接近 10m/s；
- 陡坡根据真实物理能力合理降速；
- 不再从坡底掉出赛道；
- 如果仍无法拉高，能够明确判断是 VEL 飞控瓶颈，而不是继续盲目调参数。

---

# 47. 一句话总结

当前不要继续大改：

```text
Gate 感知
Yaw
整体路径规划
```

而应该：

> **先彻底查清并放开 Z 软件限幅，再实测 VEL 模式真实垂直能力。**

当前最值得立刻做的修改：

```text
z_rate_max:
1.5 → 4~5

vz_up_limit:
3 → 4~5

vz_accel_limit:
3 → 6~8

Kz:
0.6 → 1.0 左右

Kff_z:
增加到约 1.2
```

然后通过：

```text
vz_available(vxy)
```

让 Speed Scheduler 真正知道：

> **当前这个坡，10m/s 到底能不能爬得上去。**

如果能：

```text
保持高速。
```

如果不能：

```text
只在这一段自动降速。
```

如果连低水平速度下 VEL 都拉不上去：

```text
下一阶段再切 PWM。
```
