# RMUA 2026：转弯/爬升骤降速与 Yaw 不转向修复方案

## 0. 本轮只解决两个问题

### 问题 1

```text
转弯 / 爬升
↓
速度迅速下降
↓
甚至接近停止
```

### 问题 2

```text
路线已经开始转弯
↓
算法也声称预测了未来 Gate / 弯道
↓
但无人机实际 yaw 几乎不变
↓
前视相机仍朝旧方向
↓
后续 Gate 离开 FOV
↓
YOLO / 双目丢 Gate
↓
无人机继续冲出赛道
```

本轮目标：

> **让无人机保持连续前进，只有“真的动力学跟不上”时才适当降速；同时让机头真正提前跟着未来路径转动。**

目标巡航速度仍设为：

```text
10 m/s
```

但不是强制任何情况下都 10 m/s。

---

# 1. 问题一：为什么转弯 / 爬升时速度会突然掉下来

当前速度不是由一个控制器决定，而是由多个限制共同决定。

典型结构：

```text
v_cruise
v_curve
v_slope
v_alt
extra_cap
gate_bad_near
↓
取最小值
↓
再通过 a_plan 做速度斜坡
```

这种设计本身没有错。

真正的问题是：

> **某些限制触发条件太宽、太早、太保守。**

---

# 2. 当前最可疑的限制：gate_bad_near

当前逻辑类似：

```text
距离 Gate < 25 m
+
lat / vert 瞬时误差超过阈值
↓
速度直接限制到约 1.5 m/s
```

这个条件非常容易在：

```text
转弯
上坡
远距离 Gate
Gate 还没有真正对齐
```

时触发。

但距离 Gate 还有：

```text
20~25m
```

时存在较大横向 / 高度误差本来就是正常的。

因为无人机还有足够距离完成：

```text
平滑修正。
```

所以：

```text
“当前误差大”
```

不应该等价于：

```text
“马上降到 1.5m/s”。
```

---

# 3. 删除 gate_bad_near 的“瞬时误差限速”

旧：

```python
if d_gate < 25 and (lat_err > limit or vert_err > limit):
    speed_cap = 1.5
```

建议删除。

改成：

> **预测在 Gate Plane 处是否真的会穿偏。**

---

# 4. 用“预测穿门误差”代替“当前误差”

当前飞机距离 Gate：

```text
20m
```

不能只看现在：

```text
lat_err
vert_err
```

而应该根据：

```text
当前速度
参考轨迹
未来曲线
```

预测：

```text
飞机到达 Gate Plane 时的位置。
```

得到：

```text
predicted_lat_error
predicted_z_error
```

只有：

```text
预测真的会穿出 Gate Aperture
```

才降速。

否则：

```text
继续高速沿轨迹修正。
```

---

# 5. Gate 限速新规则

建议：

```text
预测能安全穿门
→ 不因 Gate 降速

预测轻微偏离
→ 10 → 7~8 m/s

预测明显会穿偏
→ 5~6 m/s

真正接近门且仍无法修正
→ 3~4 m/s
```

正常控制不再出现：

```text
1.5m/s
```

这种极低速度。

除非：

```text
Emergency / Recovery。
```

---

# 6. 第二个可疑项：v_alt

如果当前：

```text
只要 z_error 变大
```

就快速压低 XY 速度，

上坡时非常容易形成：

```text
开始上坡
↓
z_error 增大
↓
XY 降速
↓
飞行器姿态重新变化
↓
Path / Gate 更新变慢
↓
又继续降速
```

最终看起来像：

```text
到坡口停下来。
```

---

# 7. v_alt 只应该处理“实际掉队”，而不是“正在爬坡”

必须区分：

```text
A. Path 正在要求上升
B. 飞机真的没有跟上
```

如果：

```text
vz_cmd 已经正确向上
+
未来高度误差可恢复
```

就不应该大幅降速。

只有：

```text
actual z error
持续增大
+
predicted z error
也继续恶化
```

才启动速度限制。

---

# 8. 推荐高度限速逻辑

例如：

```text
|z_err| < 0.4m
→ 不限速

0.4~0.8m
且误差没有继续恶化
→ 不限速或仅 0.9×

0.4~0.8m
且误差持续恶化
→ 0.8×

0.8~1.2m
且预测继续恶化
→ 0.6×

>1.2m
且接近安全边界
→ 0.4×
```

但：

```text
不直接停车。
```

---

# 9. 第三个可疑项：v_slope

当前：

```text
v_slope =
slope_eta * vz_up_safe / |dz/ds|
```

思想正确。

但是必须保证：

```text
dz/ds
```

来自：

```text
可信的未来 3D Path
```

而不是：

```text
某一帧错误 Gate Z。
```

否则：

```text
一个错误的陡坡
↓
v_slope 瞬间很小
↓
速度突然下降。
```

---

# 10. v_slope 必须使用“可信坡度”

只有以下 Gate 才能参与：

```text
Hard Anchor
```

即：

```text
多帧稳定
World Z 稳定
四角 / 几何通过
sigma_z 合格
顺序正确
```

Soft Gate：

```text
只提供趋势
```

不能产生非常低的：

```text
v_slope cap。
```

---

# 11. extra_cap 建议删除或合并

如果当前：

```text
|dz/ds| > 0.15
↓
extra_cap = 6m/s
```

同时又有：

```text
v_slope
```

那么本质是：

```text
同一个“坡度问题”
被两个限速器重复处理。
```

建议：

```text
删除 extra_cap
```

只保留：

```text
v_slope。
```

避免控制逻辑越来越难解释。

---

# 12. v_curve 也不要因为“转弯”直接大幅降速

目前类似：

\[
v_{curve}
=
\frac{v_{cruise}}
{1+k_{curve}\kappa}
\]

可以用，

但更推荐从：

```text
允许的横向加速度
```

计算。

理论：

\[
a_{lat}
=
v^2\kappa
\]

所以：

\[
v_{curve}
=
\sqrt{
\frac{a_{lat,max}}
{|\kappa|+\epsilon}
}
\]

这样：

```text
真正急弯
→ 降速

大半径弯道
→ 仍然可以高速。
```

---

# 13. 速度限制只保留三个主要来源

建议精简成：

```text
v_cruise
v_curve
v_slope
v_tracking
```

最终：

```python
v_target = min(
    v_cruise,
    v_curve,
    v_slope,
    v_tracking
)
```

其中：

```text
v_tracking
```

统一处理：

```text
预测 Gate 穿偏
+
实际 Z 明显掉队
+
Cross-track 明显失控
```

不要再增加很多：

```text
1.5
6
0.5×
```

互相叠加的特殊 case。

---

# 14. 给正常运行设置“非紧急最低速度”

如果：

```text
不是 Emergency
不是 Recovery
```

建议：

```text
v_target >= 4 m/s
```

第一轮可以用：

```text
normal_speed_floor = 4.0
```

这样即使几个软限制同时触发：

```text
也不会突然像停住一样。
```

注意：

> 如果真实坡度在动力学上必须低于 4m/s 才能通过，则不能强行使用这个 Floor。

因此 Floor 只适用于：

```text
Soft limitation。
```

真正的：

```text
physical feasibility limit
```

仍可低于 4m/s。

---

# 15. Hard Limit 与 Soft Limit 分开

推荐：

## Hard Limit

来自：

```text
真实垂直能力
真实曲率动力学限制
安全边界
```

可以突破：

```text
normal_speed_floor。
```

## Soft Limit

来自：

```text
Gate 当前误差
视觉置信度
一般 tracking error
```

最低：

```text
4m/s。
```

这样不会因为视觉一抖：

```text
速度从 10 直接掉到 1.5。
```

---

# 16. Speed Target 必须做平滑变化

即使：

```text
v_target
```

从：

```text
10 → 6
```

也不要一帧直接切换。

使用：

```text
acceleration limit
deceleration limit
```

建议：

```text
a_up = 3~5 m/s²
a_down = 4~6 m/s²
```

例如：

```text
10 → 6
```

用：

```text
0.7~1.0s
```

完成。

---

# 17. 问题二：为什么无人机实际 yaw 没有变化

这个问题优先级非常高。

当前必须区分：

```text
算法有没有算 yaw
```

和：

```text
yaw 有没有真的发给模拟器。
```

这是两回事。

---

# 18. 第一检查：VelCmd.yawRate 是否真的非零

转弯时运行：

```bash
rostopic echo /airsim_node/drone_1/vel_body_cmd
```

重点看：

```text
yawRate
```

正确情况：

```text
直线：
yawRate ≈ 0

接近右弯：
yawRate > 0 或 < 0

弯中：
持续非零

转正：
逐渐回到 0
```

如果转弯全过程：

```text
yawRate = 0
```

那么问题已经找到：

> **Yaw Controller 没有真正接进最终 VelCmd。**

---

# 19. 当前公开 route_follower 中存在明确风险

当前公开版本的发送逻辑仍类似：

```python
cmd.vx = vx
cmd.vy = vy
cmd.vz = vz
cmd.yawRate = 0.0
```

因此即使：

```text
yaw_controller.py
```

算出了：

```text
yaw_rate_cmd
```

如果最终：

```text
publish_cmd()
```

没有把它传进去，

无人机永远不会转头。

---

# 20. publish_cmd 必须修改

由：

```python
def publish_cmd(self, vx, vy, vz):
```

修改为：

```python
def publish_cmd(
    self,
    vx,
    vy,
    vz,
    yaw_rate
):
```

然后：

```python
cmd.yawRate = yaw_rate
```

主循环：

```python
yaw_rate_cmd = yaw_controller.compute(...)

self.publish_cmd(
    vx_b,
    vy_b,
    vz,
    yaw_rate_cmd
)
```

---

# 21. 第二检查：是否有多个 Publisher

运行：

```bash
rostopic info /airsim_node/drone_1/vel_body_cmd
```

必须确认：

```text
只有一个控制 Publisher。
```

否则可能：

```text
Yaw Controller 发布非零 yawRate
↓
另一个旧节点马上发布 yawRate=0
↓
最终看起来无人机不转。
```

---

# 22. Yaw Target 应该来自未来路径，不是当前位置

当前位置：

```text
P
```

取未来：

```text
20~30m
```

参考轨迹点：

```text
Q
```

目标 yaw：

\[
\psi_{target}
=
atan2(
Q_y-P_y,
Q_x-P_x
)
\]

这样机头：

```text
提前看向弯道。
```

---

# 23. Yaw Lookahead 建议 20~30m

当前位置 Lookahead：

```text
8~15m
```

Yaw Lookahead：

```text
20~30m
```

因为前视相机需要：

```text
提前看到弯后的 Gate。
```

不是：

```text
飞机已经进弯
才转头。
```

---

# 24. Yaw Rate 控制

误差：

\[
e_\psi
=
wrap(
\psi_{target}-\psi
)
\]

第一版：

\[
yawRate
=
K_\psi e_\psi
\]

再：

```text
clamp。
```

建议：

```text
K_yaw = 1.0 ~ 1.5
yaw_rate_max = 0.8 ~ 1.2 rad/s
```

先保守测试。

---

# 25. Yaw Rate 也做斜坡

不要：

```text
0
→
1.2rad/s
```

瞬间变化。

增加：

```text
yaw_accel_limit
```

例如：

```text
1.5 ~ 3 rad/s²
```

让：

```text
机头连续转动。
```

---

# 26. Yaw Target 数据源优先级

推荐：

```text
1. Future Valid Gate Chain tangent
2. XY Route tangent
3. Current velocity heading
```

如果：

```text
未来 Gate Chain 可靠
```

就看：

```text
Gate Chain 的未来方向。
```

如果暂时丢 Gate：

```text
继续看 XY Route。
```

---

# 27. 视觉只做小幅 FOV 修正

如果未来 Gate 在图像右侧：

```text
u_gate > cx
```

可以给：

```text
yaw
```

一个小修正。

最终：

\[
yawRate
=
K_{path}e_{yaw}
+
K_{vision}e_{img}
\]

其中：

```text
Kvision << Kpath
```

这样：

```text
路径决定机头主方向
视觉只负责让 Gate 更靠近画面中心。
```

---

# 28. 不要让视觉直接控制位置转弯

当前避免恢复：

```text
Gate Attraction。
```

无人机位置：

```text
仍然沿 Reference Curve。
```

视觉：

```text
影响 Gate Map
+
小幅影响 yaw。
```

这样轨迹保持平滑。

---

# 29. Yaw 与速度调度必须联动

如果：

```text
机头 yaw 跟不上未来曲线
```

高速继续前冲会：

```text
相机丢 Gate。
```

所以增加：

```text
yaw_tracking_quality
```

例如：

```text
|yaw_error| < 10°
→ 不限速

10°~20°
→ 0.9×

20°~30°
→ 0.8×

>30°
→ 0.6×
```

但最低：

```text
4~5m/s
```

而不是：

```text
直接停。
```

---

# 30. 为什么这样比 gate_bad_near 更合理

以前：

```text
Gate XY / Z 误差大
↓
认为危险
↓
直接降到 1.5m/s
```

但真正的问题可能只是：

```text
机头没有转过去。
```

新逻辑：

```text
未来路径开始弯
↓
Yaw 提前转
↓
相机继续看到 Gate
↓
Gate Map 保持稳定
↓
Reference Curve 正常更新
↓
速度只根据真实动力学需求适当变化
```

---

# 31. 转弯状态的理想行为

```text
还有 30m 到弯道
↓
Yaw Target 已开始变化
↓
yawRate 非零
↓
机头提前转向
↓
前视 YOLO 看到弯后的 Gate
↓
XY Path 开始转弯
↓
保持 7~10m/s
↓
顺滑过弯
```

而不是：

```text
XY 已转
↓
机头没转
↓
Gate 丢失
↓
gate_bad
↓
速度掉到 1.5
↓
继续冲偏。
```

---

# 32. 上坡状态的理想行为

```text
未来 Z 曲线开始上升
↓
vz_ff 提前出现
↓
一边向前一边爬升
↓
v_slope 判断 10m/s 是否物理可行
```

如果可行：

```text
保持接近 10m/s。
```

如果不可行：

```text
例如降到 7m/s。
```

但不要因为：

```text
当前 z_err 稍大
```

就：

```text
掉到 1.5m/s。
```

---

# 33. 新速度调度器建议

```python
v_cruise = 10.0

v_curve = curve_physics_limit(
    curvature,
    lateral_accel_limit
)

v_slope = vertical_physics_limit(
    dz_ds_preview,
    vz_up_safe,
    slope_eta
)

v_tracking = tracking_quality_limit(
    predicted_gate_error,
    predicted_z_error,
    yaw_error
)

hard_cap = min(
    v_cruise,
    v_curve,
    v_slope
)

soft_cap = max(
    normal_speed_floor,
    v_tracking
)

v_target = min(
    hard_cap,
    soft_cap
)

v_cmd = speed_ramp(
    v_current,
    v_target
)
```

---

# 34. 推荐初始参数

```text
v_cruise = 10 m/s

normal_speed_floor = 4.0~5.0 m/s
```

```text
yaw_lookahead = 25 m

K_yaw = 1.2

yaw_rate_max = 1.0 rad/s

yaw_accel_limit = 2.0 rad/s²
```

```text
position_lookahead =
10~15m
```

```text
a_up =
4m/s²

a_down =
5m/s²
```

---

# 35. 第一轮调试不要同时测试所有东西

## Test A：只测试 Yaw 输出链

速度：

```text
3~4m/s
```

临时不改速度策略。

观察：

```text
yaw_target
yaw_actual
yaw_error
yawRate_cmd
VelCmd.yawRate
```

要求：

```text
弯道 yaw 实际明显变化。
```

---

# 36. Test B：Yaw 转起来以后测试视觉

速度：

```text
4~6m/s
```

观察：

```text
visible_gate_count
future_gate_count
```

比较：

```text
固定 yaw
vs
未来路径 yaw
```

确认：

```text
转弯时 Gate 丢失明显减少。
```

---

# 37. Test C：删除 gate_bad_near 极低速

保持：

```text
最大 6m/s
```

检查：

```text
转弯 / 上坡是否还突然掉到 1.5m/s。
```

---

# 38. Test D：逐步加速

依次：

```text
6m/s
8m/s
10m/s
```

不要直接第一次就：

```text
10m/s 全场跑。
```

---

# 39. 必须加入日志

每 0.2~0.5s 至少记录：

```text
speed_actual
v_target

v_cruise
v_curve
v_slope
v_tracking
final_cap_reason

curvature
dz_ds

z_error
predicted_z_error

yaw_actual
yaw_target
yaw_error
yawRate_cmd
VelCmd.yawRate

visible_gate_count
future_gate_count
```

---

# 40. 最重要的 final_cap_reason

每次发生限速时：

```text
必须明确记录是谁在限速。
```

例如：

```text
CAP=SLOPE
10.0 → 7.2
```

或：

```text
CAP=CURVE
10.0 → 8.1
```

或：

```text
CAP=TRACKING
10.0 → 5.0
```

不要只打印：

```text
final speed = 1.5
```

否则下一次仍然很难判断。

---

# 41. Yaw 调试日志同样必须分层

输出：

```text
yaw_source=GATE_CHAIN
yaw_target=...
yaw_actual=...
yaw_err=...
yaw_rate_calc=...
yaw_rate_sent=...
```

其中：

```text
yaw_rate_calc
```

和：

```text
yaw_rate_sent
```

必须同时记录。

这样可以立刻判断：

```text
是没算出来
还是算出来但没发出去。
```

---

# 42. ROS 层检查

运行：

```bash
rostopic info /airsim_node/drone_1/vel_body_cmd
```

确认：

```text
Publisher 数量 = 1
```

然后：

```bash
rostopic echo /airsim_node/drone_1/vel_body_cmd
```

检查：

```text
yawRate
```

真正随弯道变化。

---

# 43. 本轮暂时不要继续增加新的复杂功能

暂时不做：

```text
LiDAR
后视双目
复杂局部规划
MPC
新的 Gate 吸附
```

先把：

```text
速度调度
+
Yaw 真正转起来
```

两个基础问题彻底解决。

---

# 44. 验收标准：问题 1

转弯 / 上坡时：

```text
速度不再无缘无故掉到 1~2m/s。
```

普通弯道：

```text
6~10m/s
```

明显坡道：

```text
根据实际垂直能力合理降速。
```

只有：

```text
真实物理不可行 / Emergency
```

才允许低于正常速度 Floor。

---

# 45. 验收标准：问题 2

进入弯道前：

```text
yaw_target 提前变化。
```

随后：

```text
VelCmd.yawRate 非零。
```

然后：

```text
yaw_actual 实际跟随变化。
```

最终：

```text
前视相机持续朝向未来路径
后续 Gate 不再因为机头固定而大量丢失。
```

---

# 46. 最终方案

本轮不要继续用：

```text
Gate 当前误差大
↓
大幅降速
```

作为主要修复方法。

真正应该变成：

```text
未来路径 / Gate Chain
        ↓
提前预测曲率和坡度
        ↓
Yaw 提前转头
        ↓
Camera 保持看向未来
        ↓
3D Path 持续有效
        ↓
只根据真实动力学需求限速
        ↓
保持连续高速飞行
```

一句话总结：

> **先把“机头真正转起来”解决，再把限速器从“误差触发型”改成“预测与动力学可行性型”。**

这样才能同时解决：

```text
转弯/爬升突然减速
+
转弯相机丢 Gate 后冲出赛道
```

两个问题。
