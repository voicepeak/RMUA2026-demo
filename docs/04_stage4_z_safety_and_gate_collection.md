# RMUA 2026：Z 轴合法高度与真实 Gate 采集小方案

## 1. 当前问题

目前 XY 路径跟踪已经基本稳定，但 Z 轴仍然存在两个未解决问题：

1. 不清楚道路中的合法高度范围；
2. 当前 `gates_1_3.yaml` 仍是占位数据，真实 Gate 中心高度还没有采集。

因此现在不应该继续扩大功能，而是先解决这个小问题：

> **先确定安全 Z 高度包络，再采集真实 Gate XYZ。**

---

## 2. 本阶段目标

本阶段只完成两件事：

```text
① 找到道路中的安全 Z 区间
② 采集真实 Gate Center XYZ
```

暂时不做：

- 视觉自动识别 Gate
- 双目深度估计
- LiDAR 高级建图
- 动态障碍预测
- 高级三维轨迹规划

---

# 3. 第一部分：确定安全 Z 高度包络

## 3.1 原因

当前飞机在 XY 上没有出赛道，但仍可能因为 Z 超出道路允许空间而进入非法区域。

因此首先需要回答：

> 在当前道路中心位置，飞机允许在哪个 Z 区间内稳定飞行？

---

## 3.2 测试方法

选择一个道路中心、XY 足够安全的位置。

固定：

```text
x = x_test
y = y_test
```

然后只改变 Z。

例如：

```text
z1
↓
悬停 2~3 秒

z2
↓
悬停 2~3 秒

z3
↓
悬停 2~3 秒
```

每次记录：

```text
目标 z_ref
实际 pose_gt.z
实际 vz
控制是否正常响应
是否出现冻结
是否出现明显掉高
是否进入异常力场
```

---

## 3.3 建议测试方式

不要一次跳很大。

建议：

```text
每次改变 0.2 ~ 0.5 m
```

逐步测试。

测试方向：

```text
从当前稳定高度开始
↓
逐渐往上测试

然后回到稳定高度
↓
逐渐往下测试
```

---

## 3.4 最终得到三个值

例如：

```text
z_legal_min
z_legal_max
z_cruise
```

其中：

```text
z_legal_min / z_legal_max
```

是实测合法范围。

但程序不要直接用极限值。

应该再留安全余量：

```text
z_safe_min
z_safe_max
```

例如：

```text
实测边界
↓
向内部缩 0.5 ~ 1.0 m
↓
得到工程安全范围
```

---

# 4. 加入 Z Safety Clamp

无论后续规划器给什么高度，都必须经过安全限制。

逻辑：

```text
Planner 给出 z_ref
        ↓
Z Safety Clamp
        ↓
安全 z_ref
        ↓
Z Controller
```

伪代码：

```python
z_ref_safe = clamp(
    z_ref,
    z_safe_min,
    z_safe_max
)
```

然后：

```python
ez = z_ref_safe - z_current

vz = Kz * ez

vz = clamp(
    vz,
    -vz_max,
    vz_max
)
```

这样即使 Gate 数据或者路径数据出现异常，也不会轻易把飞机送出合法高度。

---

# 5. 增加边界保护

建议额外增加软边界。

例如：

```text
距离上边界还剩 0.5 m
→ 禁止继续向上高速飞

距离下边界还剩 0.5 m
→ 禁止继续向下高速飞
```

逻辑：

```python
if z_current 接近 z_safe_min:
    禁止继续朝非法方向运动

if z_current 接近 z_safe_max:
    禁止继续朝非法方向运动
```

实际正负方向按照 NED 和 `vel_body_cmd` 实测结果确定。

---

# 6. 第二部分：采集真实 Gate XYZ

完成安全高度保护后，再进行 Gate 采集。

当前：

```text
gates_1_3.yaml
```

还是占位数据。

必须替换成真实 Gate Center。

---

# 7. Gate 采集原则

不要只记录：

```text
Gate XY
```

而应该记录：

```text
Gate X
Gate Y
Gate Z
```

也就是：

> **Gate 几何中心的完整三维坐标。**

---

# 8. Gate 采集流程

手动或低速控制飞机靠近检测门。

然后调整位置，使飞机尽量位于：

```text
检测门左右中心
+
检测门上下中心
```

示意：

```text
┌─────────────┐
│             │
│      ●      │
│     UAV     │
│             │
└─────────────┘
```

此时读取：

```text
pose_gt.x
pose_gt.y
pose_gt.z
```

然后执行：

```text
add_gate
```

保存该 Gate。

---

# 9. Gate 数据格式建议

```yaml
gates:

  - id: 1
    x: ...
    y: ...
    z: ...

  - id: 2
    x: ...
    y: ...
    z: ...

  - id: 3
    x: ...
    y: ...
    z: ...
```

如果后面要扩展，可以再加入：

```yaml
  - id: 1
    x: ...
    y: ...
    z: ...
    type: gate
    speed: 1.5
    approach_distance: 4.0
```

但当前阶段先保留 XYZ 即可。

---

# 10. Gate 采集注意事项

采集时：

```text
飞得慢
```

不要：

```text
高速穿过去再估计位置
```

因为高速状态下：

- pose 更新存在延迟；
- 人工观察不容易确定中心；
- Z 误差更大；
- 容易误采门前或门后的点。

推荐：

```text
缓慢靠近
↓
尽量停在门中心
↓
确认位置
↓
add_gate
```

---

# 11. Gate 采集后先检查 Z 序列

例如得到：

```text
Gate 1: z = ...
Gate 2: z = ...
Gate 3: z = ...
Gate 4: z = ...
```

检查：

```text
是否存在明显高度变化
```

如果 Gate Z 基本一致：

```text
后续高度规划可以比较简单
```

如果 Gate Z 变化明显：

```text
必须使用 3D 路径和 Z 插值
```

---

# 12. 暂时不要做视觉 Gate Detector

本阶段不建议立刻加入：

```text
Camera
↓
OpenCV / YOLO
↓
Gate Detection
```

原因：

当前问题不是：

> “不知道哪里有 Gate。”

而是：

> “没有可靠的真实 Gate XYZ 和合法 Z 区间。”

先把这两个确定后，系统就能进入稳定的 3D 路径规划。

视觉检测后续只作为：

```text
Gate Center 的局部修正
```

而不是当前主导航方案。

---

# 13. 本阶段推荐执行顺序

严格按照以下顺序：

```text
Step 1
选择道路中心安全位置

↓

Step 2
逐步改变 Z

↓

Step 3
找到合法 Z 上下边界

↓

Step 4
设置 z_safe_min / z_safe_max

↓

Step 5
代码加入 Z Safety Clamp

↓

Step 6
验证高度不会再越界

↓

Step 7
低速人工飞到每个 Gate 中心

↓

Step 8
add_gate 记录真实 XYZ

↓

Step 9
替换 gates_1_3.yaml 占位数据

↓

Step 10
检查所有 Gate 的 Z 高度分布
```

---

# 14. 本阶段验收标准

满足以下条件即可完成：

- 已实测道路合法 Z 范围；
- 已设置安全 Z 包络；
- 程序不会因为错误 `z_ref` 再轻易飞出高度边界；
- `vz` 有最大速度限制；
- 所有关键 Gate 已采集真实 XYZ；
- `gates_1_3.yaml` 不再使用占位数据；
- Gate Z 高度变化已经检查；
- 可以在安全高度内低速接近并穿过至少一个 Gate。

---

# 15. 下一步

完成本阶段后，再进入：

```text
Gate XYZ
    ↓
3D Reference Path
    ↓
Z 插值
    ↓
Approach → Gate Center → Exit
    ↓
连续穿门
```

如果后续发现 Gate 坐标会随场景或 seed 变化，再考虑加入：

```text
Camera Gate Detector
```

如果 Gate 坐标固定，则优先使用预存真实 Gate XYZ，视觉只做辅助校正。

---

# 16. 当前最核心结论

现在不要同时解决太多问题。

只做：

```text
① Z 合法高度范围
② Z Safety Clamp
③ 真实 Gate XYZ 采集
```

这三个完成后，再继续做三维路径。

也就是：

> **先保证“不会飞出高度边界”，再保证“知道每个门真正在哪里”。**
