# RMUA 2026：上坡掉高修复与 10 m/s 高速飞行方案

## 0. 当前问题

目前系统已经能够：

- 沿 XY 路径稳定飞行；
- 识别多个 Gate；
- 通过双目获取 Gate XYZ；
- 构造 Gate Chain；
- 根据 Gate 生成 Z 方向参考。

但在赛道出现明显上坡时，无人机会出现：

```text
XY 仍然高速向前
↓
Z 轴来不及跟上
↓
飞机实际高度低于赛道 / Gate 高度
↓
从坡下面进入非法区域
↓
冻结 / 结束
```

现在还希望将巡航速度进一步提高到：

```text
10 m/s
```

因此不能继续使用：

```text
当前位置 → 当前 z_ref → P 控制
```

这种纯反馈高度控制。

高速版本必须增加：

```text
① 高度前视
② Z 速度前馈
③ 垂直能力约束
④ 上坡掉高保护
⑤ 速度平滑加减速
⑥ 陡峭 Gate Z 二次验证
```

---

# 1. 为什么高速上坡一定更容易掉高

假设三维路径：

```text
Gate A
sA, zA

Gate B
sB, zB
```

路径 Z 坡度：

\[
k_z =
\frac{z_B-z_A}{s_B-s_A}
\]

如果水平沿路径速度：

\[
v_s
\]

那么为了严格跟随坡度，理论所需的 NED Z 变化速度：

\[
\dot z_{required}
=
k_z v_s
\]

而 `vel_body_cmd.vz` 的正方向与 NED Z 正方向相反，因此实际垂直速度命令的前馈方向为：

\[
v_{z,ff}
=
-k_zv_s
\]

这说明：

> **速度越快，同一段坡所要求的垂直速度也越大。**

---

# 2. 用当前 Gate 数据看 10 m/s 的问题

当前已经采集的一组 Gate 中，绝大多数相邻 Gate 的高度变化比较缓。

但有一段非常明显：

```text
Gate 6:
s ≈ 168.94
z ≈ -7.18

Gate 7:
s ≈ 180.04
z ≈ -10.64
```

路径距离约：

```text
Δs ≈ 11.1 m
```

Z 变化约：

```text
|Δz| ≈ 3.46 m
```

坡度绝对值：

\[
|k_z|
\approx
\frac{3.46}{11.1}
\approx
0.312
\]

如果直接以：

```text
10 m/s
```

通过，则需要约：

\[
|v_z|
\approx
10\times0.312
\approx
3.12m/s
\]

也就是说：

> **如果当前 Z 控制最大只能给 0.5～1 m/s，那么这一段在 10 m/s 下从数学上就不可能跟上。**

所以不能简单：

```text
cruise_speed = 10
```

然后全场固定 10 m/s。

正确方式：

> **目标巡航速度设为 10 m/s，但根据未来坡度自动判断当前是否允许真的跑到 10 m/s。**

---

# 3. 第一优先级：确认陡坡 Gate Z 是真的

Gate 6 → Gate 7 的坡度明显比前面的很多区段大。

这可能是：

```text
A. 地图确实突然大幅上坡
```

也可能是：

```text
B. Gate 7 双目深度 / 匹配存在误差
```

因此新增：

```text
Steep Gate Validation
```

凡是满足：

\[
\left|\frac{\Delta z}{\Delta s}\right|
>
k_{steep}
\]

例如：

```text
k_steep = 0.15
```

则不能单次观测后立即作为硬锚点。

必须：

```text
远距离观测
↓
中距离重新观测
↓
近距离再次观测
↓
World XYZ 多帧融合
↓
确认 Z 稳定
```

只有确认：

```text
sigma_z / MAD_z 足够小
```

才：

```text
hard_anchor = true
```

否则先作为：

```text
soft_anchor
```

防止错误 Z 把无人机直接带向地面 / 场外。

---

# 4. 多 Gate Z 曲线继续保留

不要根据单个下一 Gate 突然改变 Z。

使用未来多个有效 Gate：

```text
G0
G1
G2
G3
...
```

构造：

```text
z_path(s)
```

推荐：

```text
PCHIP
```

或者平滑分段曲线。

优点：

```text
通过所有可信 Gate 高度
+
不容易产生普通三次样条的过冲
+
可以直接求 dz/ds
```

最终 Altitude Planner 应提供：

```text
z_path(s)
dz_ds(s)
```

---

# 5. 最大改变：Z 轴增加 Preview

现在的问题之一是：

```text
飞机走到坡上
↓
才发现 z_ref 已经变高
↓
再开始爬升
```

这在 10 m/s 下太晚。

应该改成：

> **根据未来路径提前爬升。**

定义：

```text
Z Preview Distance
```

\[
L_z =
L_{z0}
+
T_zv
\]

例如：

```text
Lz0 = 8 m
Tz  = 1.5 ~ 2.5 s
```

如果：

```text
v = 10 m/s
```

则：

```text
Lz ≈ 23 ~ 33 m
```

也就是说：

> 在进入上坡前 20～30 m 就开始准备高度变化。

---

# 6. Preview Reference

当前路径进度：

```text
s_now
```

高度反馈基准仍然是：

```text
z_path(s_now)
```

但同时观察：

```text
z_path(s_now + Lz)
```

得到：

```text
future_z
```

如果未来明显需要上升：

```text
future_z < current_path_z
```

（NED 中更负代表更高）

则提前进入：

```text
CLIMB_PREPARE
```

---

# 7. 推荐不要简单把 z_ref 直接跳到未来高度

不能：

```text
z_ref = z(s+30m)
```

否则飞机可能提前升太多。

更好的做法：

```text
当前高度反馈
+
未来坡度前馈
```

即：

\[
v_z
=
v_{z,fb}
+
v_{z,ff}
\]

其中：

\[
v_{z,fb}
=
K_z(z_{actual}-z_{path}(s))
\]

而：

\[
v_{z,ff}
=
-\frac{dz}{ds}v_s
\]

---

# 8. 再增加 Preview Feedforward

为了补偿控制器滞后，可在前馈中使用前视坡度：

\[
k_{preview}
=
\frac{dz}{ds}(s+L_{slope})
\]

然后：

\[
v_{z,ff}
=
-k_{preview}v_s
\]

推荐：

```text
L_slope = 5 ~ 15 m
```

这样：

```text
坡还没真正到脚下
```

就已经开始给向上的垂直速度。

---

# 9. Z Feedback + Feedforward

正式 Z 控制：

\[
e_z=
z_{actual}-z_{path}(s)
\]

\[
v_{z,fb}=K_ze_z
\]

\[
v_{z,ff}
=
-\frac{dz}{ds}_{preview}v_s
\]

最终：

\[
v_z
=
v_{z,fb}
+
v_{z,ff}
\]

再限制：

\[
-v_{z,max}
\le
v_z
\le
v_{z,max}
\]

这比单纯：

```text
vz = Kz * error
```

更适合高速三维路径跟踪。

---

# 10. 单独测出真正可用的 vz_max

现在不要直接假定：

```text
vz_max = 1 m/s
```

应做一个独立测试。

关闭 XY：

```text
vx = 0
vy = 0
```

分别测试：

```text
vz = 1
vz = 2
vz = 3
vz = 4 m/s
```

观察：

```text
实际 pose.z 变化率
控制是否稳定
是否明显超调
```

最终得到：

```text
vz_up_safe
vz_down_safe
```

例如如果实测：

```text
向上安全连续速度 ≈ 3.5 m/s
```

那么算法可采用：

```text
vz_up_limit = 3.0 m/s
```

保留一定余量。

注意：

> 这是高速方案非常关键的实测参数。

---

# 11. 10 m/s 必须受“爬坡能力”约束

如果路径坡度：

\[
|k_z|
\]

当前可用安全垂直速度：

\[
v_{z,safe}
\]

则水平路径速度必须满足：

\[
v_s
\le
\eta
\frac{v_{z,safe}}
{|k_z|+\epsilon}
\]

其中：

```text
η = 0.7 ~ 0.85
```

保留 15～30% 垂直控制余量。

---

# 12. 示例：Gate 6 → Gate 7

坡度约：

```text
0.312
```

假设实测安全向上速度：

```text
vz_up_safe = 3.0 m/s
```

取：

```text
η = 0.8
```

则：

\[
v_{xy,max}
\approx
0.8\times\frac{3.0}{0.312}
\approx
7.7m/s
\]

意味着：

```text
目标巡航可以是 10m/s
```

但这一段应该自动降到：

```text
≈7.5m/s
```

如果安全向上速度只有：

```text
2m/s
```

则这段最多约：

```text
5m/s
```

否则就是物理上跟不上。

---

# 13. 所以“10 m/s”应该定义成

```text
v_cruise_max = 10 m/s
```

而不是：

```text
v_always = 10 m/s
```

最终速度：

\[
v_{cmd}
=
\min(
v_{cruise},
v_{curve},
v_{z-slope},
v_{gate-quality},
v_{safety}
)
\]

平坦直路：

```text
10 m/s
```

明显上坡：

```text
自动 5~8m/s
```

必要时：

```text
更低
```

---

# 14. 上坡掉高保护必须比“死亡边界”更早触发

定义高度滞后：

\[
e_z
=
z_{actual}-z_{path}(s)
\]

NED 下：

```text
e_z > 0
```

表示：

> 飞机比参考路径低。

推荐三级保护。

---

# 15. Level 0：正常

```text
e_z < 0.25 m
```

允许：

```text
正常目标速度
```

---

# 16. Level 1：轻微掉高

```text
0.25 < e_z < 0.5 m
```

执行：

```text
XY speed × 0.8
```

并保证：

```text
vz 使用完整前馈。
```

---

# 17. Level 2：明显掉高

```text
0.5 < e_z < 0.8 m
```

执行：

```text
XY speed × 0.4 ~ 0.5
```

优先爬升。

---

# 18. Level 3：危险

```text
e_z > 0.8 ~ 1.0 m
```

执行：

```text
forward speed = 0 ~ 1 m/s
```

只做：

```text
CLIMB_RECOVERY
```

直到：

```text
e_z < 0.3 m
```

再恢复高速。

这样不会：

```text
明明已经低于坡面
还继续 10m/s 往前冲
```

最终掉出地图。

---

# 19. 更好的保护：预测未来高度误差

当前时刻高度还正常，不代表 1 秒以后正常。

预测：

\[
z_{predicted}
=
z_{actual}
+
\dot z_{actual}T_p
\]

路径未来高度：

\[
z_{future}
=
z_{path}(s+v_sT_p)
\]

计算：

\[
e_{pred}
=
z_{predicted}-z_{future}
\]

例如：

```text
Tp = 1 ~ 2 s
```

如果预测：

```text
1.5 秒以后会低 1m
```

则现在就减速 / 提前爬升。

这对 10m/s 特别重要。

---

# 20. CLIMB_PREPARE 状态

当：

```text
未来 20~30m
```

出现明显上坡：

```text
dz/ds < -threshold
```

进入：

```text
CLIMB_PREPARE
```

行为：

```text
保持较高 XY 速度
+
提前增加向上 vz_ff
```

而不是等真正上坡才反应。

---

# 21. CLIMB_PRIORITY 状态

如果：

```text
实际高度已经明显落后
```

进入：

```text
CLIMB_PRIORITY
```

行为：

```text
XY 自动减速
Z 使用较大控制权限
```

恢复后：

```text
回到 NORMAL
```

---

# 22. Gate 矩形链也用于验证坡度

当前已经有：

```text
未来多 Gate Queue
```

因此可以检查：

```text
G0 → G1 → G2 → G3
```

是不是形成合理空间曲线。

例如：

```text
G0.z=-7
G1.z=-7.2
G2.z=-14
G3.z=-7.8
```

这种明显孤立异常：

```text
不能直接相信 G2。
```

通过：

```text
局部坡度
+
前后 Gate 趋势
+
多帧观测
```

进行剔除。

---

# 23. 特别处理“连续上坡”

如果未来 3 个 Gate：

```text
z0 > z1 > z2
```

（NED 中越来越负）

则认为：

```text
Continuous Climb
```

提前形成一条：

```text
连续高度曲线
```

而不是：

```text
每过一道门再重新设下一高度。
```

---

# 24. 加速度参数：10 m/s 不要瞬间切换

官方 `vel_body_cmd` 的加速度参数最大：

```text
8 m/s²
```

并且官方明确提醒：

> 大幅速度变化会造成明显姿态波动。

因此 10m/s 测试不要：

```text
0 → 10
```

瞬间跳变。

推荐：

```text
accel_cmd = 5 ~ 6 m/s²
```

先测试。

这样：

```text
0 → 10m/s
```

需要约：

```text
1.7 ~ 2 s
```

比较合理。

官方允许最高 8m/s²，可在后续稳定后继续提高。

---

# 25. 增加速度 Ramp

即使给模拟器 acceleration 参数，也建议算法层再做：

```text
Speed Ramp
```

例如：

```text
当前 4m/s
目标 10m/s
```

每一周期只允许逐步增加。

定义：

\[
|dv/dt|
\le
a_{plan}
\]

例如：

```text
a_plan = 4 ~ 6m/s²
```

避免目标速度来回跳变。

---

# 26. Z 方向也做 Rate Limit

不要让：

```text
vz
```

每一帧：

```text
0.5
→ 3.5
→ 0.2
```

建议：

```text
vertical_accel_limit
```

例如：

```text
2 ~ 4 m/s²
```

逐步调大。

这样相机 / Gate Z 小抖动不会直接造成飞机剧烈上下动作。

---

# 27. 10 m/s 首轮参数建议

先尝试：

```text
v_cruise_max = 10.0 m/s
```

但：

```text
v_min = 2.5 m/s
```

---

高度：

```text
Kz = 0.5 ~ 0.8
```

具体依据实测调节。

---

前视：

```text
z_preview_time = 2.0 s
z_preview_min = 8 m
```

10m/s 时：

```text
Z Preview ≈ 28m
```

---

速度：

```text
xy_accel_plan = 5 m/s²
vel_cmd_accel = 6 m/s²
```

稳定后可再提高。

---

Z 安全：

```text
z_error_slow1 = 0.25 m
z_error_slow2 = 0.5 m
z_error_stop  = 0.9 m
```

---

坡度速度余量：

```text
slope_eta = 0.8
```

---

# 28. 推荐速度调度公式

```python
v_cruise = 10.0

# 弯道限制
v_curve = curvature_speed_limit(curvature)

# 垂直能力限制
v_slope = slope_eta * vz_safe / max(abs(dz_ds_preview), eps)

# Gate 数据质量限制
v_quality = gate_quality_speed_limit()

# 高度掉队保护
v_altitude = altitude_error_speed_limit(z_error, predicted_z_error)

v_target = min(
    v_cruise,
    v_curve,
    v_slope,
    v_quality,
    v_altitude
)

v_cmd = speed_ramp(
    current_speed,
    v_target,
    accel_limit
)
```

---

# 29. 推荐 Z 控制伪代码

```python
s = path_progress

z_ref = z_profile(s)

dz_ds_preview = z_profile.derivative(
    s + slope_preview_distance
)

# 当前路径速度
v_s = current_path_speed

# 反馈
z_error = z_actual - z_ref
vz_fb = Kz * z_error

# 前馈
vz_ff = -dz_ds_preview * v_s

# 总垂直速度
vz_target = vz_fb + vz_ff

vz_target = clamp(
    vz_target,
    -vz_down_limit,
    vz_up_limit
)

vz_cmd = rate_limit(
    previous_vz_cmd,
    vz_target,
    vertical_accel_limit
)
```

---

# 30. 预测掉高逻辑

```python
preview_time = 1.5

future_s = s + v_s * preview_time

future_z = z_profile(future_s)

predicted_z = (
    current_z
    +
    current_z_dot * preview_time
)

predicted_error = (
    predicted_z
    -
    future_z
)
```

如果：

```text
predicted_error > threshold
```

立即：

```text
降低 XY 速度
+
保持 / 提高向上前馈。
```

---

# 31. 上坡期间不要过度依赖 Gate Center 瞬时 Z

Gate Chain 中：

```text
远处 Gate
```

双目 Z 本来就比近处更容易抖。

所以：

```text
远处 Gate:
Soft Anchor

接近以后：
重新观测

稳定后：
Hard Anchor
```

Altitude Profile 使用：

```text
已经验证的 Gate
+
软 Gate 趋势
```

而不是一帧深度。

---

# 32. Gate Z 可信度调速

如果前方某段高度曲线依赖：

```text
低 support / 高 sigma_z
```

则即使几何上允许：

```text
10m/s
```

也自动限制：

```text
4~6m/s
```

等靠近重新确认以后再提速。

高速必须建立在：

```text
路径可信
```

的前提上。

---

# 33. 最值得单独测试的区段

不要每次从头跑。

当前建议首先测试：

```text
Gate 6 → Gate 7
```

因为它对：

```text
高度前视
Z Feedforward
Slope Speed Limit
```

要求最高。

如果这里跑通：

```text
其它大部分缓坡段会容易很多。
```

---

# 34. Gate 6→7 测试顺序

### Test 1

```text
v_cruise = 4m/s
```

确认：

```text
Z Profile 本身正确。
```

---

### Test 2

```text
v_cruise = 6m/s
```

记录：

```text
z_ref
z_actual
vz_ff
vz_fb
```

---

### Test 3

```text
v_cruise = 8m/s
```

检查：

```text
Slope Speed Limit 是否开始发挥作用。
```

---

### Test 4

```text
v_cruise_max = 10m/s
```

允许系统自己决定：

```text
这一段究竟跑 10
还是因为 Z 能力降到 6~8。
```

---

# 35. 不建议直接强制 Gate 6→7 10m/s

如果真实坡度确实约：

```text
0.312
```

而实测安全垂直速度达不到：

```text
约 3.1m/s + 控制余量
```

那么强制 10m/s：

> **必然会高度落后。**

算法正确的行为不是强行跑 10，而是：

```text
其它平缓段 10m/s
陡坡自动降速
坡顶以后重新回到 10m/s
```

这才是真正高平均速度。

---

# 36. 调试时间优化

为了减少测试时间：

```text
支持：
start_gate
end_gate
start_s
```

例如：

```text
start_gate = 6
end_gate = 8
```

直接专项测试陡坡。

同时记录：

```text
rosbag / CSV
```

不要每次只靠肉眼观察。

---

# 37. 建议记录的数据

至少：

```text
time
path_s

v_target
v_actual_xy
v_slope_limit

z_ref
z_actual
z_error

future_z
predicted_z_error

dz_ds
dz_ds_preview

vz_fb
vz_ff
vz_cmd

gate ids
gate quality

flight_state
```

---

# 38. 建议画三张图

## 图 1

```text
s - Z
```

画：

```text
Gate Z
Z Profile
Actual Z
```

---

## 图 2

```text
time - velocity
```

画：

```text
Target XY Speed
Actual XY Speed
Slope Speed Limit
```

---

## 图 3

```text
time - vertical control
```

画：

```text
vz_fb
vz_ff
vz_cmd
```

这样可以马上判断：

```text
是路径错
还是预判太晚
还是 vz 不够
还是速度太快。
```

---

# 39. 后续 PWM 控制预留

官方速度接口属于基础飞控，官方说明：

```text
大幅速度变化时可能产生姿态波动。
```

如果未来出现：

```text
路径 / 前馈都正确
垂直能力也足够
但 8~10m/s 时仍明显掉高 / 振荡
```

则可以判断：

> **VEL 基础飞控本身开始成为瓶颈。**

届时再考虑：

```text
PWM Controller
```

自己做：

```text
位置
↓
速度
↓
姿态
↓
推力
```

当前阶段暂时不做。

---

# 40. 当前总体架构

```text
Multi-Gate Vision
       ↓
Ordered Gate Chain
       ↓
Gate Quality Validation
       ↓
3D Reference Builder
       ↓
z_path(s)
dz/ds
       │
       ├──────────────┐
       ↓              ↓
XY Path Preview    Z Preview
       ↓              ↓
Curvature Limit   Z Feedforward
       │              │
       └──────┬───────┘
              ↓
      Speed Feasibility
              ↓
      target speed ≤10m/s
              ↓
       Speed Ramp
              │
              ├───────── Z Feedback
              │
              ↓
       Nominal 3D Velocity
              ↓
      Obstacle Avoidance
        （当前 bypass）
              ↓
        World → Body
              ↓
        vel_body_cmd
```

---

# 41. 本阶段完成标准

需要满足：

- 上坡前能提前产生 Z 控制动作；
- 不再等到高度已经落后才补偿；
- Z 控制包含 Feedforward + Feedback；
- 能根据 `dz/ds` 计算垂直需求；
- 10m/s 作为最大巡航速度而不是强制恒速；
- 平坦路段能接近 10m/s；
- 陡坡根据实际垂直能力自动降速；
- 高度掉队超过阈值时会主动牺牲 XY 速度；
- 不再因为从坡下面飞出赛道而结束；
- Gate Z 陡变会进行多帧 / 多 Gate 二次验证；
- 可以从任意 Gate 区间快速启动测试；
- Gate 6→7 等最陡区段能够重复通过。

---

# 42. 最关键的修改

旧逻辑：

```text
前进
↓
发现自己高度低了
↓
P 控制慢慢补
↓
补不过来
↓
掉出坡底
```

新逻辑：

```text
看到未来 20~30m 要上坡
↓
提前产生向上 vz Feedforward
↓
计算当前 10m/s 是否来得及爬升
↓
来得及：
保持高速

来不及：
自动降低 XY 速度
↓
始终保持在三维赛道附近
↓
坡顶重新加速到 10m/s
```

---

# 43. 一句话方案

> **10 m/s 可以试，但必须把它作为“最大允许巡航速度”，而不是全场固定速度。**

真正决定当前能跑多快的应该是：

```text
道路曲率
+
未来 Gate Z 坡度
+
实际垂直速度能力
+
高度跟踪误差
+
Gate 数据可信度
```

对于上坡问题，最关键的两项升级是：

```text
Z Preview + Feedforward
```

和：

```text
Slope-aware Speed Limit
```

这样才能既提高平均速度，又避免继续从坡下面掉出赛道。
