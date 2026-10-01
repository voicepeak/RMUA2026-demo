> 历史归档，已由[当前交接文档](../../00_current_handoff.md)取代。本文中的“当前”、运行状态、测试数、参数和建议均属于记录当时；不能据此启动比赛或认定现版本已验收。旧代码路径及命令以当时工作目录为准。

# RMUA2026-demo：坡道拉升、穿门卡顿与控制链冲突完整解决方案

> 基线：`voicepeak/RMUA2026-demo` 当前 `main`（2026-09-29）
>
> 目标：解决以下三类核心问题：
>
> 1. 遇到陡坡时高度拉不起来；
> 2. 最新控制逻辑相比上一版出现穿门卡顿、不够丝滑；
> 3. `route_follower` 内存在 Z、速度、GateMap、重捕获等多套控制逻辑叠加，控制 authority 不唯一，调参越来越困难。

---

## 0. 总结结论

当前问题不建议继续通过“再加一个补偿器”解决。

现有控制链已经具备：

- `AltitudeProfile`：生成 `z_ref(s)`；
- `dz/ds` 前馈；
- `k_z` 高度反馈；
- `SpeedScheduler`：曲率 / 坡度 / tracking 限速；
- `vz_capability.yaml`：不同水平速度下的垂直能力；
- GateMap / GateChain；
- Yaw 前视；
- telemetry / trace。

真正需要做的是：

> **把重复控制合并，把“预测爬升”从一个新的 `vz` override，改成 Speed Scheduler 的前方可行性约束。**

最终原则：

```text
规划层：决定未来应该走哪里、爬多高、什么时候开始爬
    ↓
速度层：判断以当前动力学是否来得及完成
    ↓
跟踪层：只负责跟踪 reference
    ↓
安全层：只做最后兜底
    ↓
唯一 publisher
```

任何物理量只允许一个主 authority：

| 物理量 | 唯一主 authority |
|---|---|
| XY reference | Reference Planner |
| Z reference | Altitude Profile |
| 水平速度 | Speed Scheduler |
| `vz` | Z Tracking Controller |
| yaw | Yaw Controller |
| 最终安全裁决 | Safety Supervisor |
| AirSim `VelCmd` | 单一 publisher |

---

# 1. 当前代码中最关键的问题

## 1.1 Z 轴存在重复控制

当前正常 Z 控制已经有：

```text
AltitudeProfile
    ↓
z_ref
    ↓
z_err
    ↓
vz_fb = k_z * z_err

同时

dz/ds preview
    ↓
vz_ff = -k_ff_z * dzds * v

最终

vz_target = vz_fb + vz_ff
```

但后面又增加了：

```text
recon_climb
z_anticipate
vz_up/down clamp
vz acceleration clamp
```

其中最需要处理的是：

```python
if ng is not None and self.k_anticipate > 0.0:
    ...
    if need_up > max(0.0, vz_target):
        vz_target = min(
            self.k_anticipate * need_up,
            self.vz_up_limit
        )
```

这相当于在已经有：

```text
z_ref → FF + FB
```

之后又重新建立了一套：

```text
gate z → time-to-gate → vz override
```

因此 Z 轴实际上有两个控制器在追同一个目标。

### 直接后果

接近高门时可能出现：

```text
正常 z_ref 跟踪
    ↓
进入 anticipate 区域
    ↓
vz_target 被重新覆盖
    ↓
高度误差和水平限速随之变化
    ↓
水平速度下降
    ↓
time-to-gate 又变化
    ↓
anticipate 再重新计算
```

表现就是：

- 门前顿一下；
- 爬升速率突然变大；
- 水平速度突然掉；
- 穿门后立即切下一门，指令再次变化；
- 视觉上呈现“不丝滑”。

---

## 1.2 `z_anticipate` 使用 3D 距离估计到门时间，不够合理

当前：

```python
t_gate = d_g / v
```

但 `d_g` 是无人机到 Gate 的三维距离，而 `v` 主要是路线方向水平速度。

即：

```text
d_g = sqrt(dx² + dy² + dz²)
```

却用：

```text
水平速度
```

去除。

尤其在陡坡场景：

```text
水平距离已经很近
但高度仍差很多
```

此时 `d_g` 会被高度差撑大，导致：

```text
t_gate
need_up
vz_target
```

产生不直观变化。

正确做法不是修这个 `t_gate`，而是删除它作为主控制器的资格。

---

# 2. 第一阶段：先恢复“上一版的丝滑感”

这一阶段不要重构大架构，目的只是验证回归来源。

---

## 2.1 默认关闭 `z_anticipate`

### 修改

`route_follower.py`

将：

```python
self.k_anticipate = float(
    rospy.get_param("~k_anticipate", 1.0)
)
```

改为：

```python
self.k_anticipate = float(
    rospy.get_param("~k_anticipate", 0.0)
)
```

暂时保留代码，便于 A/B。

---

## 2.2 把相关参数全部暴露到 launch

当前一部分策略参数存在于 Python 默认值，却不在 `route_follower.launch` 中显式暴露。

建议新增：

```xml
<arg name="k_anticipate" default="0.0"/>
<arg name="z_anticipate_dist" default="25.0"/>

<arg name="z_lag_slow" default="1.5"/>
<arg name="z_lag_gain" default="5.0"/>

<arg name="recon_speed" default="2.0"/>
<arg name="recon_climb" default="0.6"/>
<arg name="recon_max_dist" default="30.0"/>
<arg name="recon_max_climb" default="4.0"/>
```

并传入：

```xml
<param name="k_anticipate" value="$(arg k_anticipate)"/>
<param name="z_anticipate_dist" value="$(arg z_anticipate_dist)"/>

<param name="z_lag_slow" value="$(arg z_lag_slow)"/>
<param name="z_lag_gain" value="$(arg z_lag_gain)"/>
```

目的：

> 所有会明显改变控制行为的参数，都必须能从 launch 一眼看到。

---

## 2.3 做严格 A/B

保持其余代码完全不变：

### A

```text
k_anticipate = 1.0
```

### B

```text
k_anticipate = 0.0
```

固定：

- simulator seed；
- route；
- gate map；
- cruise speed；
- YOLO / static gate 模式；
- yaw 参数；
- 起点。

记录：

```text
v_target
v_cmd
z_ref
z
vz_ff
vz_fb
vz_target
vz_cmd
next_gate_id
d_gate
CAP reason
yaw error
```

如果 B 明显恢复丝滑：

> 删除 `z_anticipate` 的主控制职责，后续不要再围绕它调 `k`。

---

# 3. 第二阶段：重做“坡道拉升”的正确逻辑

## 3.1 核心思想

不要：

```text
快到门了
↓
发现高度不够
↓
突然增加 vz
```

要：

```text
还在远处
↓
根据未来高度变化判断当前 10 m/s 是否来得及爬
↓
如果来不及，提前把 XY 速度平滑降下来
↓
仍然只使用现有 vz_ff + vz_fb
↓
飞机持续斜向爬升
↓
到门时自然到达正确高度
```

---

# 4. 新增 Climb Feasibility Preview

这是整个修改中最重要的一项。

建议新文件：

```text
route_follower/scripts/climb_feasibility.py
```

---

## 4.1 不只看当前位置局部 `dz/ds`

当前 `SpeedScheduler.slope_limit()` 使用：

```python
v_slope = eta * vz_available / abs(dzds)
```

思路没错，但只看当前或有限 preview 的局部坡度会有一个问题：

在坡真正开始之前：

```text
当前 dz/ds ≈ 0
```

于是：

```text
v_slope ≈ cruise
```

等飞机进入坡以后才开始减速，已经晚了。

所以需要一个：

> **前方累计高度需求约束。**

---

## 4.2 前向扫描算法

在未来：

```text
H = max(H_min, v * T_preview)
```

建议初值：

```text
T_preview = 3.0 ~ 4.0 s
H_min = 20 m
H_max = 50 m
```

例如 10 m/s：

```text
preview ≈ 30 ~ 40 m
```

这比当前 1.75 s Z preview 更适合“提前判断是否爬得上”。

---

## 4.3 对未来多个点计算爬升可行性

对：

```text
s_i = s_now + 5m
      s_now + 10m
      ...
      s_now + H
```

计算：

```python
ds = s_i - s_now
dz = z_ref(s_i) - z_now
```

NED 下注意符号。

我们关心的是需要的绝对垂直位移：

```python
dz_need = abs(dz)
```

对于每个未来点估计：

```text
需要垂直时间：

t_z = dz_need / vz_available
```

考虑响应时间：

```text
t_required = t_z + t_response
```

其中建议：

```text
t_response = 0.3 ~ 0.5 s
```

那么水平速度上限：

```text
v_cap_i = ds / t_required
```

最终：

```python
v_climb_preview = min(v_cap_i)
```

---

## 4.4 更严格的版本

因为：

```text
vz_available
```

本身取决于 `vxy`，建议做 2 次迭代：

```python
v_test = current_target

for _ in range(2):
    vz_available = capability.available(v_test)
    v_new = calculate_preview_cap(vz_available)
    v_test = min(v_test, v_new)
```

不需要复杂优化器。

---

## 4.5 上升和下降能力分开

当前 capability 主要以“上升能力”处理。

最终建议：

```text
vz_up_available(vxy)
vz_down_available(vxy)
```

分开标定。

因为：

```text
上升
下降
```

的动力学通常并不对称。

配置改为：

```yaml
up:
  - vxy: 0
    vz: 4.0
  - vxy: 5
    vz: ...
  - vxy: 10
    vz: ...

down:
  - vxy: 0
    vz: 3.0
  - ...
```

---

# 5. SpeedScheduler 改成唯一水平速度 authority

当前：

```text
SpeedScheduler
+
route_follower 中 MAP_HORIZON
+
Z_LAG
+
recon
```

都会修改 `v_target`。

应该统一。

---

## 5.1 SpeedScheduler 最终输入

建议：

```python
SpeedScheduler.target(
    curvature_preview,
    climb_preview,
    tracking_error,
    yaw_error,
    gate_alignment,
    map_visibility,
    recon_mode
)
```

---

## 5.2 所有限速统一变成 cap

### 物理硬限制

```text
v_cruise
v_curve
v_climb
v_map
v_safety
```

最终：

```python
hard_cap = min(
    v_cruise,
    v_curve,
    v_climb,
    v_map,
    v_safety
)
```

### Tracking 软限制

```text
gate miss
z tracking error
yaw error
```

生成：

```python
soft_cap
```

最终：

```python
v_target = min(
    hard_cap,
    max(normal_speed_floor, soft_cap)
)
```

注意：

> `normal_speed_floor` 只能保护普通 tracking soft slowdown，不能覆盖物理硬约束。

陡坡真的需要 2.5 m/s 才能爬得上：

```text
就必须允许 2.5 m/s
```

不能因为：

```text
normal_speed_floor = 4
```

强行维持 4 m/s。

---

# 6. 删除 route_follower 外部重复 Z_LAG 限速

当前主循环中还有：

```python
if e_z0 > self.z_lag_slow:
    v_zlag = ...
    v_target = min(v_target, v_zlag)
```

但 `SpeedScheduler.tracking_limit()` 已经会根据：

```text
z_err
z_worsening
pred_worse
```

降速。

这形成重复 authority。

---

## 6.1 建议

删除普通情况下的外层：

```text
Z_LAG speed cap
```

将其改成真正的：

```text
Safety Recovery
```

只有达到严重阈值才触发。

例如：

```text
|z_err| < 1.0 m
    → 正常 scheduler

1.0 ~ 2.0 m
    → scheduler tracking penalty

> 2.0 m 且仍持续恶化
    → safety recovery
```

Safety Recovery 只负责：

```text
强制限制水平速度
```

而不是日常调速。

---

# 7. 硬约束不能只经过普通加减速度斜坡

当前：

```python
v = speed_sched.step(
    v_prev,
    v_target,
    dt
)
```

如果突然检测到：

```text
v_hard = 3 m/s
```

但上一帧：

```text
v = 10 m/s
```

受 `a_down` 限制，实际 command 仍可能短时间高于物理安全 cap。

---

## 7.1 推荐结构

```python
v_smooth = scheduler.step(
    v_prev,
    desired_speed,
    dt
)

v_cmd = min(
    v_smooth,
    hard_cap
)
```

但真正理想情况不是靠这个突然截断。

应通过：

```text
前方 curve envelope
+
climb feasibility preview
```

提前把 hard cap 降下来。

最终 clamp 只做最后保险。

---

# 8. 曲率限速也要改成“前方制动包络”

当前：

```text
v_curve = sqrt(a_lat_max / kappa_now)
```

只看当前位置曲率。

这与坡道问题完全类似：

```text
进入急弯之前仍然高速
↓
进入急弯后才计算出低速
↓
减速来不及
```

---

## 8.1 做法

前方 20~40 m 采样：

```text
s+5
s+10
s+15
...
```

计算：

```text
v_curve_limit(s_i)
```

然后根据最大允许减速度：

```text
v_now² <= v_future² + 2*a_brake*ds
```

得到：

```python
v_now_cap_i = sqrt(
    v_future_i**2
    + 2 * a_brake * ds
)
```

对所有未来点取最小：

```python
v_curve_preview = min(v_now_cap_i)
```

这样：

> 转弯会提前减速，而不是转弯时才顿一下。

---

# 9. Z 控制器最终只保留 FF + FB

正常 TRACK 模式：

```python
vz_fb = k_z * (z - z_ref)

vz_ff = -k_ff_z * dzds_preview * v

vz_target = vz_ff + vz_fb
```

然后：

```python
vz_clamped = clamp(
    vz_target,
    -vz_down_limit,
    vz_up_limit
)

vz_cmd = accel_limit(
    previous_vz,
    vz_clamped
)
```

仅此一套。

---

## 9.1 删除正常 TRACK 中的 `z_anticipate override`

最终应完全删除：

```text
gate height
→ time-to-gate
→ direct vz override
```

需要“提前爬”时：

> 由 Reference Planner 提前给出未来 Z 轨迹，
> 由 Speed Scheduler 提前降低水平速度。

不是再创造一个 Z controller。

---

# 10. Recon 必须成为独立 mode，不能与正常 TRACK 叠加

当前：

```python
vz_target = vz_fb + vz_ff

if recon_active:
    vz_target += recon_climb
```

这仍然属于多个控制意图叠加。

---

## 10.1 建议状态机

```text
TRACK
RECON
HOLD
ABORT
```

### TRACK

```text
Reference Planner
Speed Scheduler
Z FF + FB
Yaw tracking
```

### RECON

明确切换：

```text
vxy = recon_speed
vz = recon_climb
yaw = search strategy
```

或者：

```text
保持当前位置附近的小范围前进搜索
```

但不要：

```text
正常 Z controller + recon_climb
```

同时生效。

---

# 11. GateChain：`z_suspect` 不允许直接删除可靠门

当前：

```python
g["z_suspect"] = ...
```

然后：

```python
anchors():
    return gates if not z_suspect
```

这意味着：

```text
只要坡度看起来过陡
↓
门直接从 Z profile 消失
```

这是坡道“拉不起来”的重要潜在原因。

---

## 11.1 正确语义

`z_suspect` 应表示：

```text
高度可信度下降
```

而不是：

```text
这个门不存在
```

---

## 11.2 按数据来源区分

### verified static gate

如果是人工/官方/已验证的静态门：

```text
即使坡很陡
也不能因为 slope 大就删除 anchor
```

最多：

```text
触发 climb feasibility slowdown
```

### 在线视觉 hard anchor

如果：

```text
z_suspect = true
```

可以：

```text
降低权重
要求更多 support
降低 sigma 阈值
与相邻门做 consistency 检查
```

### 在线 soft gate

只允许作为：

```text
趋势 guide
```

不能硬改变 Z 轨迹。

---

## 11.3 推荐字段

每个 gate：

```python
{
    "id": ...,
    "uid": ...,
    "source": "static/vision",
    "x": ...,
    "y": ...,
    "z": ...,
    "sigma_z": ...,
    "support": ...,

    "anchor_class": "VERIFIED/HARD/SOFT",
    "z_suspect": ...,
    "confidence": ...
}
```

AltitudeProfile 构建：

```text
VERIFIED
→ 永远保留

HARD + 非严重异常
→ 保留

SOFT
→ 只参与趋势，不作为强锚点
```

---

# 12. AltitudeProfile 不再做“突然 Gate 再融合一次”

当前：

```text
center(s)
↓
靠近下一门
↓
再次 blend 到 next_gate.z
```

而 `center(s)` 本身本来就是由 Gate anchors 构造的。

如果下一门已经是 profile 的 anchor：

```text
center(s)
+
gate blend
```

实际上又对同一个 Gate 做了一次控制。

---

## 12.1 推荐

如果 Gate 已属于可信 anchor：

```text
z_ref = profile.center(s)
```

就足够。

只在：

```text
新视觉门尚未进入正式 profile
```

的过渡阶段允许轻微 correction。

---

## 12.2 更合理的 Profile 更新

当新 GateMap 生效时：

```text
old_profile
new_profile
```

不要立即跳过去。

使用短距离或短时间 blend：

```python
z_ref =
    (1-beta) * old_profile(s)
    + beta * new_profile(s)
```

例如：

```text
0.3 ~ 0.5 s
```

完成切换。

这样不会因为在线地图突然刷新：

```text
z_ref
dz/ds
vz_ff
```

同时跳变。

---

# 13. OnlineGateCache 当前存在一个重要更新问题

已有 gate 更新时：

```python
old.update(g)
```

但 `changed=True` 主要在新增 gate 时设置。

因此可能出现：

```text
cache 中坐标已经变化
↓
ingest 返回 False
↓
planner 不 rebuild
↓
control 继续使用旧 profile
```

直到以后出现新门时才一起重建。

---

## 13.1 修复

在更新 existing gate 前记录：

```python
dx
dy
dz
ds
```

如果超过阈值：

```python
pos_changed = (
    hypot(dx,dy) > 0.15
    or abs(dz) > 0.10
    or abs(ds) > 0.20
)
```

则：

```python
changed = True
```

---

## 13.2 但不要每个视觉抖动都 rebuild

推荐：

```text
小变化
→ 只 cache EMA

累计变化达到阈值
→ pending update

稳定 N 帧
→ commit
```

---

# 14. 静态门附近的视觉门不能一律丢弃

当前 OnlineGateCache 对静态门附近：

```text
|Δs| < 8 m
```

直接：

```text
continue
```

这个策略可以防止重复门，但也意味着：

> 视觉永远无法纠正静态门坐标。

更合理：

```text
附近静态门
→ 尝试关联 existing static UID
→ 作为 measurement update

明显不是同一门
→ 才创建 online gate
```

---

# 15. GateMap 要分成“地图”和“任务状态”

当前已经引入：

```text
completed_gate_ids
```

方向是对的，但还应进一步拆分。

---

## 15.1 不要把 MISS / SKIP 也叫 completed

当前为了控制不死锁：

```text
PASS
MISS
SKIP
```

都会推进 `gate_idx`。

控制逻辑可以推进，但任务结果不能混在一个集合里。

改成：

```python
resolved_gate_ids = set()
passed_gate_ids = set()
missed_gate_ids = set()
skipped_gate_ids = set()
```

### 控制索引

使用：

```text
resolved
```

避免卡住。

### 成绩/任务状态

使用：

```text
passed
```

绝不能让：

```text
MISS/SKIP
```

等价于完成。

---

# 16. Gate crossing 判定进一步从 route-s 改成真实 Gate plane

当前穿门判定已经比早期进步：

```text
利用前后 pose 插值 crossing point
再算 gate center error
```

但触发 crossing 的条件仍主要依据：

```text
route progress s 穿过 gate.s
```

最终最好改成：

```text
trajectory segment
与
Gate plane
真实相交
```

---

## 16.1 有 Gate orientation 时

Gate：

```text
center C
normal n
```

前后位置：

```text
P0
P1
```

计算：

```python
d0 = dot(P0-C, n)
d1 = dot(P1-C, n)
```

满足：

```text
d0*d1 <= 0
```

说明跨越平面。

插值求：

```text
P_cross
```

再转换到 Gate 局部坐标：

```text
u
v
```

判断：

```text
|u| < half_width
|v| < half_height
```

---

## 16.2 没有 orientation 时

用：

```text
route tangent at gate.s
```

作为近似 Gate normal。

这样至少 Gate pass 不再完全依赖 route-s。

---

# 17. XY 穿门卡顿：门心拉拽不能在近门突然加强

当前：

```text
route lookahead point
+
gate center blend
```

这本身可以保留。

问题是 blend 如果过快，会产生目标点横向跳变。

---

## 17.1 改为 smoothstep

不要线性：

```python
alpha = ...
```

改为：

```python
alpha = smoothstep(alpha)
```

然后：

```python
w = xy_converge * alpha
```

---

## 17.2 门后不要瞬间切下一门

增加：

```text
gate_exit_blend_distance = 3 ~ 5 m
```

在刚过门的几米内：

```text
当前轨迹切线
和
下一门方向
```

做连续切换。

避免：

```text
Gate N
↓
PASS
↓
Gate N+1 瞬间成为唯一目标
↓
target direction 突变
```

---

# 18. Yaw 保持前视，但不要与 XY target 完全绑定

Yaw 的目标应该：

```text
未来路线 / Gate Chain
```

而不是只看：

```text
当前 pursuit target
```

建议继续保持：

```text
yaw_lookahead ≈ 20~25 m
```

并保留 yaw acceleration limit。

如果未来门高度上升同时伴随转弯：

```text
Speed Scheduler
```

应同时考虑：

```text
curve
climb
yaw alignment
```

取最严格 cap。

---

# 19. `CommandArbiter` 当前职责名不副实

目前它实际上只决定：

```text
XY route
vs
avoidance
```

并没有仲裁：

```text
Z
yaw
speed
safety
```

两个选择：

### 方案 A：改名

```text
XYCommandArbiter
```

### 方案 B：真正做 Final Command Arbiter

输入：

```python
{
    route_xy,
    z_cmd,
    yaw_cmd,
    avoidance,
    safety,
    mode
}
```

输出：

```python
FinalCommand(
    vx,
    vy,
    vz,
    yaw_rate
)
```

推荐 B。

---

# 20. `route_follower.py` 必须减负

当前主文件接近 1000 行，已经同时承担：

- route projection；
- gate map callback；
- gate state；
- speed planning；
- Z tracking；
- recon；
- stuck detection；
- pass scoring；
- command publish；
- telemetry；
- reset；
- safety glue。

建议最终结构：

```text
route_follower/
├── route_follower.py
│   └── ROS orchestration only
│
├── planning/
│   ├── reference_planner.py
│   ├── altitude_profile.py
│   ├── gate_chain.py
│   └── climb_feasibility.py
│
├── control/
│   ├── xy_tracker.py
│   ├── z_controller.py
│   ├── yaw_controller.py
│   ├── speed_scheduler.py
│   └── command_arbiter.py
│
├── mapping/
│   ├── online_gate_cache.py
│   ├── gate_map.py
│   └── gate_task_state.py
│
├── safety/
│   └── safety_supervisor.py
│
└── diagnostics/
    ├── telemetry.py
    └── gate_trace.py
```

---

# 21. `route_follower.py` 最终主循环应该只有这些步骤

```python
def control_loop():

    state = get_fresh_state()

    if not state.valid:
        safety_hold()
        return

    mission.update(state)

    reference = planner.plan(
        state,
        gate_map,
        mission
    )

    limits = feasibility.evaluate(
        state,
        reference
    )

    v_target = speed_scheduler.target(
        state,
        reference,
        limits
    )

    tracking_cmd = tracking_controller.compute(
        state,
        reference,
        v_target
    )

    final_cmd = safety_supervisor.apply(
        state,
        tracking_cmd,
        limits
    )

    publisher.publish(final_cmd)

    telemetry.log(...)
```

主循环内不要再出现：

```text
临时 if 某个门
临时 if 某个坡
临时再 override 一次 vz
临时再 override 一次 v
```

---

# 22. 时间处理必须修

当前很多地方使用：

```python
dt = 1 / control_rate
```

但真实 ROS / simulator 可能：

```text
掉帧
卡顿
仿真变慢
```

导致实际 dt 不是固定值。

---

## 22.1 使用 pose timestamp

```python
dt = pose_stamp - last_pose_stamp
```

然后 clamp：

```python
dt = clamp(dt, 0.01, 0.2)
```

---

## 22.2 Pose timeout

新增：

```text
pose_timeout = 0.3 s
```

如果：

```text
now - latest_pose_stamp > pose_timeout
```

立刻：

```text
vx=0
vy=0
vz=0
yaw=0
```

并记录：

```text
POSE_STALE
```

---

# 23. STUCK 检测也要使用真实 dt

当前：

```text
位置变化 / 固定 dt
```

容易在 simulator 卡顿时误判。

改成：

```text
actual displacement / actual pose dt
```

---

# 24. 必须保证只有一个 velocity publisher

运行正式控制时：

```bash
rostopic info /airsim_node/drone_1/vel_body_cmd
```

必须确认：

```text
Publishers:
 * /route_follower
```

不要同时运行：

```text
start_to_goal
z_probe
z_capability_probe
其他测试控制节点
```

---

## 24.1 加 preflight check

建议 `run_experiment.py` 启动 controller 前检查：

```text
vel_body_cmd publishers
```

如果已有 publisher：

```text
直接拒绝启动
```

而不是只 warning。

---

# 25. 最新代码建议初始参数

这不是最终比赛参数，只是“去重之后”的初始 baseline。

```yaml
cruise_speed: 10.0
max_speed: 12.0

normal_speed_floor: 3.0~4.0

a_lat_max: 6.0

a_up: 3.0~4.0
a_down: 5.0~6.0

lookahead_base: 8.0
lookahead_kv: 0.7

yaw_lookahead: 25.0
k_yaw: 1.2
yaw_rate_max: 1.0
yaw_accel_limit: 2.0

k_z: 0.8~1.0
k_ff_z: 1.0~1.2

z_rate_max: 4.0

vz_up_limit: 4.0
vz_down_limit: 3.0
vz_accel_limit: 6.0

slope_eta: 0.75~0.85

gate_blend_start: 20~25
gate_blend_full: 6~8

# 关键
k_anticipate: 0.0
```

---

# 26. 新增 preview 参数

建议：

```yaml
climb_preview_time: 3.5
climb_preview_min: 20.0
climb_preview_max: 50.0
climb_preview_step: 5.0
climb_response_time: 0.4

curve_preview_time: 3.0
curve_preview_min: 20.0
curve_preview_max: 40.0
curve_preview_step: 3.0

pose_timeout: 0.3

map_update_xy_threshold: 0.15
map_update_z_threshold: 0.10
map_update_s_threshold: 0.20
map_commit_stable_frames: 3

profile_switch_time: 0.4
gate_exit_blend_distance: 4.0
```

---

# 27. 推荐修改文件清单

## P0：马上改

### `route_follower.py`

1. `k_anticipate` 默认改 0；
2. 删除正常 TRACK 中 `z_anticipate` 对 `vz_target` 的 override；
3. `Z_LAG` 移出普通调速；
4. Recon 改成独立 mode；
5. 使用实际 pose dt；
6. 增加 pose timeout；
7. pass/miss/skip 账本分离；
8. 减少主循环 policy if。

### `route_follower.launch`

1. 暴露所有行为参数；
2. `k_anticipate=0`；
3. 增加 preview / timeout / map update 参数。

### `gate_chain.py`

1. `z_suspect` 不再直接删除 VERIFIED/HARD anchor；
2. 仅作为 confidence / safety 信息。

### `online_gate_cache.py`

1. existing gate 坐标更新超过阈值时 `changed=True`；
2. 静态门附近观测改为关联更新，不是一律 discard；
3. 增加 stable UID / commit threshold。

---

## P1：核心解决坡道

### 新增 `climb_feasibility.py`

实现：

```text
未来高度需求扫描
+
vz capability
+
水平速度上限
```

### `speed_scheduler.py`

加入：

```text
v_climb_preview
v_curve_preview
v_map
```

并成为唯一水平速度 authority。

---

## P2：架构整理

### 新增

```text
gate_task_state.py
safety_supervisor.py
reference_planner.py
z_controller.py
xy_tracker.py
```

逐步把 `route_follower.py` 改成 orchestration。

---

# 28. 推荐开发顺序

## Phase A：恢复丝滑 baseline

只做：

```text
k_anticipate = 0
```

其余全部不动。

跑：

```text
5 m/s
7 m/s
10 m/s
```

记录基线。

完成条件：

```text
门前不再明显突然拉升/顿挫
```

---

## Phase B：确认坡道根因

对失败陡坡记录：

```text
Gate ID
Gate source
z_suspect
anchor pair
z_ref
z
dzds
vz_ff
vz_fb
vz_target
vz_cmd
vz_actual
v_target
v_cmd
CAP
```

首先判断：

```text
A. Gate 是否被 z_suspect 删除
B. z_ref 是否正确提前上升
C. z_ref 正确但 vz 跟不上
D. vz 指令足够但飞机实际爬不上
```

这四种情况不能混着调。

---

# 29. 坡道诊断决策树

```text
坡拉不起来
    |
    +-- z_ref 没升？
    |      |
    |      +-- 是 → Profile / GateChain / GateMap 问题
    |
    +-- z_ref 升了，vz_target 小？
    |      |
    |      +-- 是 → dzds / Kff / Z controller 问题
    |
    +-- vz_target 很大，vz_cmd 小？
    |      |
    |      +-- 是 → limiter / accel / vz_up_limit 问题
    |
    +-- vz_cmd 大，实际 vz 小？
           |
           +-- 是 → AirSim / 动力能力 / vxy 太高
                          |
                          +-- SpeedScheduler 提前降水平速度
```

---

# 30. 每次测试必须画的 6 张曲线

不要只看视频。

必须输出：

## 图 1

```text
s - z_ref
s - z_actual
```

## 图 2

```text
s - vz_ff
s - vz_fb
s - vz_target
s - vz_cmd
s - vz_actual
```

## 图 3

```text
s - v_cmd
s - v_target
s - v_curve
s - v_climb
s - v_tracking
s - v_map
```

## 图 4

```text
s - yaw_target
s - yaw_actual
```

## 图 5

```text
s - gate lateral error
s - gate vertical error
```

## 图 6

```text
时间 - controller mode
时间 - CAP reason
时间 - gate ID
```

---

# 31. A/B 验证矩阵

| Test | anticipate | 新 climb preview | GateMap | 目的 |
|---|---:|---:|---:|---|
| A | ON | OFF | OFF | 当前行为 |
| B | OFF | OFF | OFF | 验证卡顿是否来自 anticipate |
| C | OFF | ON | OFF | 验证坡道速度规划 |
| D | OFF | ON | ON | 验证完整感知闭环 |

不要一次同时改：

```text
Z
XY
Yaw
GateMap
Vision
```

否则无法定位。

---

# 32. 验收指标

## 丝滑性

记录：

```text
max |Δv_cmd / dt|
max |Δvz_cmd / dt|
max |ΔyawRate / dt|
```

以及 Gate 前后：

```text
[-10 m, +5 m]
```

区间的速度变化。

---

## 高度

每个 Gate：

```text
vertical error
```

目标：

```text
不出现系统性低于门中心
```

并且陡坡前：

```text
z_ref 提前变化
```

而不是门口才猛变。

---

## 水平

每个 Gate：

```text
lateral error
```

并记录：

```text
PASS
MISS
SKIP
```

分开统计。

---

## 稳定性

固定 seed 多跑：

```text
>= 5 次
```

再换多个 seed。

不能只看一次“刚好过了”。

---

# 33. 最终理想控制架构

```text
                      Pose / IMU
                          |
                          v
                  State Estimator
                          |
                          v
Vision ---------> Gate Map / Task State
                          |
                          v
                  Reference Planner
                /         |         \
              XY         Z(s)       Yaw
              |           |          |
              |       dz/ds          |
              |           |          |
              +-----------+----------+
                          |
                          v
              Feasibility Preview
                /                 \
          curve envelope       climb envelope
                \                 /
                 \               /
                  v             v
                   Speed Scheduler
                          |
                          v
                  Tracking Controller
                 /        |         \
               vx/vy      vz       yawRate
                          |
                          v
                  Safety Supervisor
                          |
                          v
                   Command Arbiter
                          |
                          v
                  ONE VelCmd Publisher
                          |
                          v
                        AirSim
```

---

# 34. 最重要的四条修改原则

## 原则 1

**不要再给 `vz` 增加第三套控制器。**

正常模式：

```text
vz = FF + FB
```

足够。

---

## 原则 2

**坡道问题优先通过提前限制水平速度解决。**

因为：

```text
vxy 越大
→ 到门时间越短
→ 需要的平均 vz 越大
```

如果飞机垂直能力已经到极限：

```text
继续增加 vz command 没意义
```

必须降低：

```text
vxy
```

---

## 原则 3

**可靠 Gate 的“坡陡”不是错误。**

不能：

```text
坡度大
→ z_suspect
→ 删除 anchor
```

应该：

```text
坡度大
→ 保留 anchor
→ 降速确保能跟上
```

---

## 原则 4

**最终每个控制量只能由一个模块负责。**

否则永远会出现：

```text
这一层加速
下一层限速
第三层又 override
```

最后只能靠碰运气调参数。

---

# 35. 我建议直接实施的最终版本

短期先保留目前已经验证过的：

```text
Route Projection
Gate Chain
Altitude Profile
Yaw 前视
vz capability
Telemetry
Gate Trace
Online Gate Cache
```

删除 / 改造：

```text
z_anticipate
重复 Z_LAG
普通模式 recon_climb 叠加
z_suspect 直接删除可靠 anchor
多处散落的 speed override
```

新增：

```text
Climb Feasibility Preview
Curve Braking Envelope
Gate Task State
Safety Supervisor
Map/Profile Smooth Commit
```

最终期望效果：

```text
直道：
10 m/s 稳定巡航

进入弯道前：
提前平滑减速 + 提前 yaw

进入陡坡前：
提前根据垂直能力降低 vxy
同时 vz_ff 开始持续上升

坡中：
保持连续斜向爬升

门前：
只做小量 XY/Z 修正
不停车
不突然猛拉

过门：
短距离 exit blend
平滑转向下一门
```

这才是后续能够继续提高到高速全程飞行的稳定基础。

---

# 36. 推荐实施优先级

```text
P0
├── k_anticipate = 0 做 A/B
├── 检查唯一 VelCmd publisher
├── z_suspect 不删除 verified gate
└── OnlineGateCache existing update changed 修复

P1
├── Climb Feasibility Preview
├── Curve Preview / braking envelope
├── SpeedScheduler 单一 authority
├── 删除普通 Z_LAG 重复控制
└── Recon 独立 mode

P2
├── Profile 平滑切换
├── Gate plane 真实过门判定
├── Task State 与地图解耦
├── 实际 dt + pose timeout
└── route_follower.py 模块拆分

P3
├── 多 seed 回归
├── 速度从 3→5→7→10 m/s 分级
└── 最后再优化比赛耗时
```

---

# 37. 当前最值得先做的代码改动

如果只允许先提交一个 PR：

> **PR 1：Control Authority Cleanup**

内容只包括：

1. `k_anticipate=0`；
2. `z_suspect` 不删除 verified gate；
3. existing OnlineGateCache 更新触发 `changed`；
4. launch 暴露所有关键控制参数；
5. 检查单 publisher；
6. 增加 telemetry 字段，但不新增新的控制算法。

先把当前版本重新恢复到：

```text
稳定、可解释、可复现
```

再提交：

> **PR 2：Preview Feasibility Speed Planning**

实现：

```text
climb preview
curve preview
统一 SpeedScheduler
```

这样风险最低，也最容易判断每一次修改到底改善了什么。


---

# 38. 不能忽略的全程地图覆盖问题

当前仓库的全程审查已经指出：

```text
route_1_3 路线长度约 1424.87 m
默认静态 Gate 只有 10 个
最后静态 Gate 的 route progress 约 223.24 m
```

也就是说静态 Gate 高度锚点只覆盖路线前约：

```text
15.67%
```

同时默认 launch：

```text
use_gate_map = false
```

因此如果“后续坡拉不起来”发生在最后静态 Gate 之后，需要优先判断：

```text
到底是控制器跟不上
还是 planner 根本不知道后面的正确高度
```

这两类问题必须分开。

## 38.1 全程运行必须满足其一

### 方案 A：完整静态 Gate / 高度地图

在比赛场景固定、Gate 坐标可靠的情况下：

```text
全程 Gate
+
全程高度 anchors
```

预先完整提供。

### 方案 B：在线 GateMap

如果后续 Gate 必须靠视觉发现：

```text
use_gate_map:=true
```

并保证：

```text
视觉检测
→ Gate Tracker
→ persistent GateMap
→ OnlineGateCache
→ Reference Planner
```

整条链真正运行。

仅启动 YOLO：

```text
不等于控制器正在使用 YOLO GateMap
```

---

## 38.2 没有未来可靠高度信息时不要继续高速

当：

```text
hard horizon
soft horizon
```

都耗尽时，应明确进入：

```text
RECON
```

而不是默认：

```text
保持当前高度 + 10 m/s 继续飞
```

RECON 必须是有限状态：

```text
TRACK
 ↓ lost horizon
RECON
 ├── reacquired → TRACK
 └── max distance/time reached → HOLD/ABORT
```

---

## 38.3 全程验收不要只看控制器 PASSED

必须分开：

```text
controller PASS
independent geometric PASS
official simulator result
```

因为控制器内部的 route progress、规划 Gate 和真实门洞可能共用同一套数据。

最终成绩验收必须以：

```text
官方结果
+
独立 Gate plane geometry evaluator
```

为依据。

---

# 39. 最终完整排查顺序

遇到“某个坡拉不起来”时严格按以下顺序：

```text
1. 当前是否只有一个 VelCmd publisher？
        ↓
2. 下一 Gate 是否真实存在于当前 GateChain？
        ↓
3. Gate 的 Z 是否正确？
        ↓
4. 是否被 z_suspect 删除出 AltitudeProfile？
        ↓
5. z_ref 是否在坡前提前变化？
        ↓
6. dz/ds preview 是否合理？
        ↓
7. v_climb_preview 是否提前降低水平速度？
        ↓
8. vz_ff / vz_fb 是否产生足够指令？
        ↓
9. vz_target 是否被 limiter 截断？
        ↓
10. vz_cmd 已足够但实际 vz 仍不足？
        ↓
11. 查 vz_capability / AirSim 动力学
```

如果第 2～5 步失败：

> 属于地图 / 规划问题，不要继续调 PID。

如果第 7 步失败：

> 属于速度规划问题。

如果第 8～9 步失败：

> 属于 Z controller / limiter 问题。

如果第 10 步失败：

> 属于动力能力问题，此时正确动作是进一步降低水平速度，而不是继续增加 `vz` 指令。
