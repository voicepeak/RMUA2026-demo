# RMUA 2026：双目 OpenCV Gate 识别、三维定位与 Z 轴接入方案

## 0. 当前真实状态

目前系统已经具备：

```text
XY 路径
→ Path Follower
→ World / Body 速度转换
→ vel_body_cmd
→ 能够稳定沿赛道 XY 方向运动
```

但还缺少两件关键能力：

```text
1. 没有真实 Gate 三维坐标
2. Z 轴仍然基本保持恒定，没有随 Gate / 赛道变化
```

目前 `gates_1_3.yaml` 中的 Gate 是占位数据，不能作为真实穿门位置使用。

模拟器本身不会通过 ROS 直接发布 Gate 坐标，但前视相机能够看到真实 Gate，因此当前最合适的方案是：

> **使用前视双目相机识别 Gate，在左右图像中匹配 Gate 框体，三角化得到 Gate Center XYZ，再把 Gate 作为 Z 轴和穿门规划的关键锚点。**

当前阶段使用 OpenCV 完成。

后续将 OpenCV 的“2D Gate Detector”替换成深度学习模型，但：

```text
双目三角化
坐标变换
Gate Manager
Z Planner
```

全部保留。

---

# 1. 为什么现在优先选择双目

RMUA 2026 官方模拟器提供前视双目：

```text
/airsim_node/drone_1/front_left/Scene
/airsim_node/drone_1/front_right/Scene
```

官方配置：

```text
分辨率：960 × 720
频率：20 Hz
FOV：60°
双目基线：300 mm
```

前视左右相机位置：

```text
front_left:
X = 0.175 m
Y = -0.15 m
Z = 0

front_right:
X = 0.175 m
Y = +0.15 m
Z = 0
```

两相机方向一致：

```text
Roll = 0
Pitch = 0
Yaw = 0
```

因此这是非常标准的平行双目结构。

双目的优势是：

```text
左图：
识别 Gate 在哪

右图：
识别同一个 Gate

左右视差：
得到真实距离

Gate 3D：
直接用于 Z 规划和穿门
```

这比单目只得到：

```text
Gate 在画面哪个位置
```

信息完整得多。

---

# 2. 一个非常重要的注意事项

## 不要直接取 Gate 图像中心的双目深度

Gate 中间是空的。

例如：

```text
┌──────────────┐
│              │
│       ×      │
│              │
└──────────────┘
```

图像中心：

```text
×
```

看到的可能实际上是：

```text
Gate 后面的建筑 / 道路 / 天空
```

因此如果直接：

```text
在 Gate 中心像素取 disparity
```

得到的深度可能是：

> Gate 后面的物体距离

而不是：

> Gate 本身的距离

所以推荐：

> **检测 Gate 框体四个角或四条边，对框体进行双目三角化，再由四个 3D 角点计算 Gate 的几何中心。**

这也是后续深度学习最好输出：

```text
四角 Keypoints / Gate Mask
```

而不只是普通 Bounding Box 的原因。

---

# 3. 当前 OpenCV Gate Detector 的目标

OpenCV 这一版不用追求“最终比赛最强识别”。

它的主要作用是：

```text
① 证明 Gate 可以稳定识别
② 获取真实 Gate XYZ
③ 建立双目 → 3D Gate 的完整链路
④ 为 Z 轴规划提供真实锚点
⑤ 建立以后深度学习可以直接替换的接口
```

---

# 4. OpenCV 检测总体流程

```text
Left Image
Right Image
     │
     ▼
时间同步
     │
     ▼
双目校正 / Rectify
     │
     ├───────────────┐
     ▼               ▼
Left Gate Detect   Right Gate Detect
     │               │
     └───────┬───────┘
             ▼
        Gate Matching
             ↓
      四角左右对应
             ↓
       Stereo Triangulation
             ↓
        Gate 3D Corners
             ↓
         Gate Center XYZ
             ↓
    Camera → Body → World
             ↓
          Gate Manager
             ↓
        gates_1_3.yaml
             ↓
        Z / Gate Planner
```

---

# 5. ROS 图像同步

双目必须使用尽量接近同一时刻的左右帧。

推荐：

```python
message_filters.Subscriber(...)
ApproximateTimeSynchronizer(...)
```

订阅：

```text
front_left/Scene
front_right/Scene
```

由于相机频率为 20 Hz，一帧周期约 50 ms。

可以先使用：

```text
同步允许误差：20 ~ 30 ms
```

然后打印左右图像时间戳确认。

---

# 6. ROS Image 转 OpenCV

使用：

```python
cv_bridge
```

例如逻辑：

```python
left_cv = bridge.imgmsg_to_cv2(left_msg, "bgr8")
right_cv = bridge.imgmsg_to_cv2(right_msg, "bgr8")
```

后续所有检测都在 OpenCV Mat / numpy image 上完成。

---

# 7. 第一版 Gate 视觉特征

根据当前模拟器画面：

> Gate 是明显的白色框体，并具有橙色边缘 / 标识。

因此第一版建议采用：

```text
颜色
+
边缘
+
矩形几何
```

三种信息联合检测。

不要只依赖一个特征。

---

# 8. Step A：HSV 颜色分割

先把：

```text
BGR
```

转换为：

```text
HSV
```

```python
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
```

分别生成：

```text
orange_mask
white_mask
```

---

# 9. 橙色区域

橙色通常比白色背景更容易区分。

因此建议：

```text
橙色 Mask
```

作为第一层候选 ROI。

不要现在直接写死一个 HSV 范围作为最终参数。

推荐做一个：

```text
HSV Trackbar 调参工具
```

在真实模拟器截图上调：

```text
H_min
H_max
S_min
S_max
V_min
V_max
```

最终再保存到 YAML。

例如：

```yaml
orange_hsv:
  h_min: ...
  h_max: ...
  s_min: ...
  s_max: ...
  v_min: ...
  v_max: ...
```

---

# 10. 白色区域

白色特征通常是：

```text
低 Saturation
高 Value
```

可以作为第二个 Mask。

作用不是单独决定 Gate，而是：

```text
橙色候选区域附近
+
存在明显白色框体
```

才提高 Gate 置信度。

这样可以减少把其他橙色物体误判成 Gate。

---

# 11. Morphology

颜色分割后通常会存在：

```text
噪点
断裂
小孔洞
```

使用：

```python
cv2.morphologyEx(...)
```

做：

```text
OPEN
去小噪声

CLOSE
连接断裂区域
```

第一版 Kernel 可以从：

```text
3×3
5×5
```

开始测试。

---

# 12. Contour 提取

从 Mask 中：

```python
cv2.findContours(...)
```

得到候选区域。

先做基础过滤：

```text
面积太小
→ 删除

面积太大
→ 删除

长宽比异常
→ 删除
```

只留下可能是 Gate 的 ROI。

---

# 13. ROI 内做边缘检测

在候选 Gate ROI 中使用：

```python
cv2.Canny(...)
```

得到白框 / 橙边的边缘。

然后可以使用：

```python
cv2.HoughLinesP(...)
```

寻找：

```text
上边
下边
左边
右边
```

或者：

```python
cv2.approxPolyDP(...)
```

寻找四边形。

---

# 14. Gate 几何约束

一个可靠 Gate Candidate 应满足：

```text
有明显四边形结构
```

并且：

```text
上下边大致平行
左右边大致平行
```

还可以增加：

```text
面积阈值
长宽比范围
轮廓完整度
角点角度接近矩形
```

不要要求：

```text
必须完美矩形
```

因为远处 Gate 有透视变形。

实际应该允许：

```text
梯形 / 透视四边形
```

---

# 15. OpenCV Detector 最终输出什么

不要只输出：

```text
bbox
```

建议输出：

```text
GateObservation
```

包含：

```text
center_u
center_v

corner_tl
corner_tr
corner_br
corner_bl

bbox
confidence
timestamp
```

即：

```text
2D 四角
+
中心
+
置信度
```

这是以后深度学习也要保持的统一接口。

---

# 16. 左右图 Gate 匹配

左右图分别检测以后，需要知道：

```text
左图 Gate A
```

和：

```text
右图哪个 Gate
```

是同一个 Gate。

双目校正后，同一个空间点应大致满足：

```text
v_left ≈ v_right
```

即：

> 左右图中的 Y 像素坐标接近。

可以使用以下信息做匹配：

```text
中心 v 接近
Gate 大小接近
Gate 形状接近
左右出现顺序一致
视差合理
```

匹配代价：

```text
cost =
w1 * |vL - vR|
+
w2 * size_difference
+
w3 * shape_difference
```

选择 cost 最小且低于阈值的 Candidate。

---

# 17. 为什么使用四角匹配而不是中心匹配

左右图分别获得：

```text
TL
TR
BR
BL
```

之后：

```text
Left.TL ↔ Right.TL
Left.TR ↔ Right.TR
Left.BR ↔ Right.BR
Left.BL ↔ Right.BL
```

这样可以得到：

```text
4 个真实 Gate 框体空间点
```

而不是只得到：

```text
1 个不可靠的空洞中心深度
```

---

# 18. 双目基础公式

若双目经过校正：

```text
u_L
u_R
```

为同一个空间点在左右图像中的横坐标。

视差：

\[
d=u_L-u_R
\]

深度：

\[
Z_c=\frac{f_xB}{|d|}
\]

其中：

```text
B = 0.30 m
```

官方图像宽度：

```text
W = 960 px
```

若 60° 为水平 FOV，则可以先估算：

\[
f_x =
\frac{960}{2\tan(30^\circ)}
\approx 831.4\ px
\]

这只能作为当前模拟器的初始内参估计。

正式代码最好把相机参数集中写入：

```yaml
fx:
fy:
cx:
cy:
baseline:
```

---

# 19. 初始内参估计

若假设：

```text
水平 FOV = 60°
square pixel
```

可以暂时使用：

```text
fx ≈ 831.4 px
fy ≈ 831.4 px

cx ≈ 480
cy ≈ 360

baseline = 0.30 m
```

由于模拟器相机本身为理想/近理想模型，这组参数适合快速验证。

后续应该通过：

```text
已知尺寸 / 已知距离目标
```

验证深度误差。

---

# 20. 推荐直接使用 cv2.triangulatePoints

比手写单点：

```text
Z = fB/d
```

更方便扩展。

OpenCV：

```python
cv2.triangulatePoints(
    P_left,
    P_right,
    pts_left,
    pts_right
)
```

其中：

```text
pts_left:
四个 Gate 角点

pts_right:
对应四个角点
```

得到齐次 3D 点后除以：

```text
w
```

即可得到三维坐标。

---

# 21. Gate 3D Center

得到四角：

```text
P_TL
P_TR
P_BR
P_BL
```

Gate Center：

\[
P_G=
\frac{
P_{TL}+P_{TR}+P_{BR}+P_{BL}
}{4}
\]

这样得到的是：

> **真实门框的几何中心**

而不是后方背景。

---

# 22. 同时计算 Gate 法向量

四角还能计算 Gate Plane。

例如：

\[
v_1=P_{TR}-P_{TL}
\]

\[
v_2=P_{BL}-P_{TL}
\]

法向：

\[
n=
\frac{
v_1\times v_2
}{
\|v_1\times v_2\|
}
\]

Gate Normal 后面非常重要。

因为可以生成：

```text
Approach Point
Gate Center
Exit Point
```

---

# 23. Gate 三点模型

有：

```text
Gate Center = G
Gate Normal = n
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
d = 2 ~ 4 m
```

后续无人机：

```text
Approach
   ↓
Gate Center
   ↓
Exit
```

真正穿过去。

---

# 24. Camera 坐标转换到 Body

OpenCV 常见 Optical Frame：

```text
Xc：向右
Yc：向下
Zc：向前
```

RMUA 无人机 Body / NED 近似：

```text
Xb：向前
Yb：向右
Zb：向下
```

因此在相机姿态为：

```text
Roll=Pitch=Yaw=0
```

时，方向映射大致为：

```text
Xb = Zc
Yb = Xc
Zb = Yc
```

必须在模拟器里用一个已知点验证一次。

不要只根据符号假设。

---

# 25. 双目 Pair 的参考位置

左右相机：

```text
X = 0.175
Y = ±0.15
Z = 0
```

双目 Pair 中心：

```text
X = 0.175
Y = 0
Z = 0
```

因此可以把双目三角化结果定义在：

```text
Stereo Center Frame
```

然后：

```text
P_body
=
R_bc * P_camera
+
[0.175, 0, 0]
```

---

# 26. Body → World

调试阶段可以使用：

```text
pose_gt
```

获得：

```text
World position
+
Body quaternion
```

然后：

\[
P_W
=
R_{WB}P_B+t_{WB}
\]

最终得到：

```text
Gate World XYZ
```

也就是现在真正缺失的数据。

以后正式比赛：

```text
pose_gt
```

换成：

```text
GPS + IMU / Localization
```

Gate Detector 和 Stereo 模块不需要修改。

---

# 27. Gate 保存格式

检测稳定以后写入：

```yaml
gates:

  - id: 1
    x: ...
    y: ...
    z: ...
    nx: ...
    ny: ...
    nz: ...
    valid: true
    source: opencv_stereo
    confidence: ...

  - id: 2
    x: ...
    y: ...
    z: ...
    nx: ...
    ny: ...
    nz: ...
    valid: true
    source: opencv_stereo
    confidence: ...
```

相比只保存 XYZ，建议把：

```text
normal
```

也保存下来。

后面穿门非常有用。

---

# 28. Gate 顺序怎么确定

不能简单按照：

```text
画面从左到右
```

排序。

应该把 Gate Center 的：

```text
XY
```

投影到当前已有：

```text
XY Route
```

得到：

```text
Path Progress s
```

然后：

```text
按照 s 从小到大排序
```

即可得到：

```text
Gate 1
Gate 2
Gate 3
...
```

这和比赛要求的顺序穿门逻辑天然一致。

---

# 29. 多个 Gate 同时出现在画面怎么办

当前画面可以同时看到多个 Gate。

推荐：

```text
全部识别
```

然后分别双目三角化。

再根据：

```text
Gate World XYZ
↓
投影 XY Route
↓
Path Progress s
```

决定它们在赛道中的顺序。

当前需要关注的 Gate：

```text
next_gate
```

就是：

```text
当前 path_progress 前方最近的有效 Gate
```

---

# 30. 如何避免一帧误检测就写入错误 Gate

不要单帧直接保存。

必须做：

```text
N 帧确认
```

例如：

```text
连续 5~10 帧
```

都看到同一 Gate。

计算每一帧：

```text
Gate XYZ
```

如果：

```text
XYZ 方差很小
```

才认为：

```text
Gate Stable
```

最后使用：

```text
中位数 XYZ
```

保存。

中位数通常比均值更抗异常值。

---

# 31. Gate Tracker

OpenCV Detector 后建议增加简单 Tracker：

```text
Candidate
↓
Nearest Neighbor / IoU
↓
Track ID
```

每个 Gate Track 保存：

```text
track_id
age
last_seen
confidence
xyz_history
```

只有：

```text
age > N
confidence > threshold
```

才交给 Gate Manager。

---

# 32. OpenCV 阶段的模块划分

建议新增：

```text
rmua_gate_vision/
```

包含：

```text
stereo_sync.py
gate_detector_opencv.py
gate_matcher.py
gate_triangulator.py
gate_tracker.py
gate_world_transform.py
gate_recorder.py
```

ROS 结构：

```text
Camera Left ─┐
             ├→ Stereo Sync
Camera Right ┘
                  ↓
          OpenCV Gate Detector
                  ↓
            Gate Matcher
                  ↓
           Triangulator
                  ↓
             Gate XYZ
                  ↓
              Tracker
                  ↓
          World Transform
                  ↓
           Gate Manager
```

---

# 33. 推荐 ROS 输出

不要让视觉模块直接修改：

```text
gates_1_3.yaml
```

视觉模块先发布：

```text
/rmua/gate_observations
```

内容：

```text
gate_track_id
camera_xyz
world_xyz
corners
normal
confidence
```

然后：

```text
gate_recorder
```

决定什么时候保存到 YAML。

这样以后更容易替换算法。

---

# 34. OpenCV Detector 伪代码

```python
def detect_gate(image):

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    orange_mask = threshold_orange(hsv)

    orange_mask = morphology(orange_mask)

    contours = cv2.findContours(orange_mask)

    candidates = []

    for contour in contours:

        if not valid_area(contour):
            continue

        roi = get_roi(contour)

        edges = cv2.Canny(roi)

        corners = find_gate_quad(edges)

        if corners is None:
            continue

        score = gate_geometry_score(corners)

        if score < threshold:
            continue

        candidates.append(
            GateObservation2D(
                corners=corners,
                center=mean(corners),
                confidence=score
            )
        )

    return candidates
```

---

# 35. 双目匹配伪代码

```python
left_gates = detect_gate(left_image)
right_gates = detect_gate(right_image)

pairs = []

for gl in left_gates:

    best = None
    best_cost = INF

    for gr in right_gates:

        cost = (
            w_y * abs(gl.center_y - gr.center_y)
            +
            w_size * size_diff(gl, gr)
            +
            w_shape * shape_diff(gl, gr)
        )

        if cost < best_cost:

            best_cost = cost
            best = gr

    if best_cost < stereo_match_threshold:

        pairs.append((gl, best))
```

---

# 36. 三角化伪代码

```python
points_left = gate_left.corners
points_right = gate_right.corners

points_3d = triangulate(
    points_left,
    points_right
)

gate_center_camera = median_or_mean(
    points_3d
)

gate_normal_camera = estimate_normal(
    points_3d
)
```

然后：

```python
gate_world = camera_to_world(
    gate_center_camera,
    current_pose
)
```

---

# 37. Z 轴怎么接进来

一旦得到真实：

```text
Gate 1 XYZ
Gate 2 XYZ
Gate 3 XYZ
```

就不再使用：

```text
固定 z
```

而使用：

```text
Gate 1.z
↓
插值
↓
Gate 2.z
↓
插值
↓
Gate 3.z
```

即：

```text
XY Path
+
Gate Z Anchors
↓
z_ref(s)
```

这样终于得到真正的：

```text
三维参考路线
```

---

# 38. Z Profile

Gate A：

```text
s_A
z_A
```

Gate B：

```text
s_B
z_B
```

当前路线进度：

```text
s
```

则：

\[
t=
\frac{s-s_A}{s_B-s_A}
\]

推荐：

\[
h(t)=3t^2-2t^3
\]

最后：

\[
z_{ref}
=
z_A+h(t)(z_B-z_A)
\]

这样高度变化连续平滑。

---

# 39. 当前 Gate 靠近时优先级最高

正常：

```text
z_ref = z_profile(s)
```

靠近 Gate：

```text
z_ref → gate.z
```

即：

```text
Route Z
+
Gate Z Correction
```

最终保证：

```text
真正穿过 Gate Center
```

---

# 40. 后面使用深度学习时，什么应该被替换

以后不要推翻整套方案。

只替换：

```text
gate_detector_opencv.py
```

当前：

```text
OpenCV
颜色 + 边缘 + 四边形
↓
Gate 2D Corners
```

未来：

```text
Deep Learning
↓
Gate 2D Corners / Mask
```

后面：

```text
Stereo Matching
Triangulation
3D Transform
Gate Manager
Z Planner
```

全部继续使用。

---

# 41. 深度学习最推荐的输出不是普通 Bounding Box

普通 Object Detection：

```text
[x1,y1,x2,y2]
```

可以识别：

```text
哪里是 Gate
```

但对于双目几何不够理想。

更推荐：

## 方案 A：Keypoint Detection

直接预测：

```text
TL
TR
BR
BL
```

四个 Gate 角点。

这和当前 OpenCV 输出接口完全一致。

非常适合双目三角化。

---

## 方案 B：Instance Segmentation

输出：

```text
Gate Frame Mask
```

再通过 Mask：

```text
拟合四边形
↓
得到四角
```

对遮挡情况下可能更鲁棒。

---

# 42. 后续模型建议方向

后面可以尝试：

```text
YOLO Pose / Keypoint 类模型
```

目标输出：

```text
Gate bbox
+
4 corners
+
confidence
```

或者：

```text
YOLO Seg / 其他实例分割模型
```

输出：

```text
Gate mask
```

再由 OpenCV 做：

```text
corner extraction
```

从工程角度，更推荐：

> **深度学习负责“看见并定位 Gate”，OpenCV / 几何算法负责精确三角化和坐标变换。**

而不是让神经网络直接输出：

```text
World XYZ
```

这样更加可解释，也容易调试。

---

# 43. OpenCV 数据还能成为深度学习数据来源

现在 OpenCV Detector 做出来以后，可以自动保存：

```text
image
+
OpenCV detection
```

作为：

```text
Pseudo Label
```

然后人工快速修正。

这样可以快速积累：

```text
Gate Dataset
```

后续训练深度学习模型。

建议保存：

```text
左图
右图
四角坐标
bbox
Gate ID
时间戳
```

---

# 44. 数据采集建议

模拟器可以改变：

```text
Seed
```

因此可以采：

```text
不同 Seed
不同距离
不同高度
不同 yaw
不同亮度
部分遮挡
Gate 很小时
多个 Gate 同时出现
```

数据集不要只包含：

```text
正对 Gate
近距离
无遮挡
```

否则深度学习容易过拟合。

---

# 45. 当前最小开发路线

## Stage 1

```text
只用左相机
```

实现：

```text
OpenCV Gate 2D Detection
```

目标：

```text
画面里能稳定画出 Gate 四角
```

---

## Stage 2

加入右相机。

目标：

```text
左右图能匹配同一个 Gate
```

---

## Stage 3

实现：

```text
Stereo Triangulation
```

目标：

```text
得到 Gate Camera XYZ
```

---

## Stage 4

使用：

```text
pose_gt
```

转换：

```text
Gate Camera XYZ
→ Gate World XYZ
```

---

## Stage 5

连续多帧稳定后：

```text
写入 gates_1_3.yaml
valid: true
```

---

## Stage 6

将真实 Gate Z 接入：

```text
z_ref(s)
```

取消固定 Z。

---

## Stage 7

完成：

```text
Approach
→ Center
→ Exit
```

连续穿 Gate。

---

## Stage 8

开始收集视觉数据。

---

## Stage 9

训练 Deep Learning Gate Detector。

---

## Stage 10

只替换：

```text
OpenCV Detector
→ DL Detector
```

其余架构不变。

---

# 46. 当前验收标准

## OpenCV 2D 阶段

- 能识别白框 / 橙边 Gate；
- 多 Gate 同时出现时能输出多个 Candidate；
- 能稳定得到四角；
- 误检不会非常严重；
- 连续帧检测位置稳定。

## Stereo 阶段

- 左右 Gate 可以正确匹配；
- 四角视差方向正确；
- 能得到合理 Gate 深度；
- Gate 四角深度互相接近；
- Gate Center 不会落到背景深度。

## 3D 阶段

- Gate XYZ 随无人机运动基本保持世界坐标稳定；
- 重复经过同一 Gate 得到的位置误差可接受；
- Gate 顺序可以根据 XY Route progress 排列。

## Z Planner 阶段

- Z 不再恒定；
- 能提前朝下一 Gate 高度调整；
- 穿门前 Z 接近 Gate Center Z；
- 可以连续通过多个 Gate。

---

# 47. 当前最终架构

```text
                    LEFT CAMERA
                         │
                    OpenCV / DL
                         │
                    2D Gate Corners
                         │
                         ├─────────────┐
                         │             │
                    RIGHT CAMERA      │
                         │             │
                    OpenCV / DL       │
                         │             │
                    2D Gate Corners   │
                         │             │
                         └──────┬──────┘
                                ↓
                         Stereo Matching
                                ↓
                         Triangulation
                                ↓
                       Gate Camera XYZ
                                ↓
                      Camera → Body → World
                                ↓
                           Gate Manager
                                │
                ┌───────────────┴───────────────┐
                ↓                               ↓
         Gate XYZ / Normal               XY Route Progress
                │                               │
                └───────────────┬───────────────┘
                                ↓
                         Altitude Planner
                                ↓
                            z_ref(s)
                                ↓
                           Z Controller
                                ↓
                               vz
                                │
          XY Path Follower ─────┤
                                ↓
                         vel_body_cmd
                                ↓
                               UAV
```

---

# 48. 当前结论

现在最合适的方案就是：

> **先用 OpenCV + 双目把真实 Gate 三维坐标搞出来。**

不要继续：

```text
固定 Z
```

也不要现在立刻跳到：

```text
深度学习
```

当前 OpenCV 的意义不是最终识别性能，而是尽快打通：

```text
相机看到 Gate
↓
识别四角
↓
左右匹配
↓
三角化
↓
真实 Gate XYZ
↓
Gate Z Anchor
↓
动态 Z Path
↓
穿门得分
```

等这条几何链路完全跑通以后：

```text
OpenCV Gate Detector
```

直接替换成：

```text
Deep Learning Gate Detector
```

即可。

**最重要的接口应该从现在就固定为：**

```text
Detector 输出 Gate 四角 / Mask
↓
Stereo 输出 Gate 3D
↓
Gate Manager 输出 World XYZ
↓
Planner 使用 Gate
```

这样现在做的工作不会因为后续换深度学习而推倒重来。

---

# 49. 官方参数依据

RMUA 2026 官方模拟器给出的前视双目参数包括：

```text
分辨率：960×720
频率：20 Hz
FOV：60°
基线：300 mm
```

模拟器 `settings.json` 中前视左右相机分别位于：

```text
X = 0.175 m
Y = -0.15 / +0.15 m
Z = 0
```

并且方向均为：

```text
Roll=0
Pitch=0
Yaw=0
```

因此双目三角化在这个模拟环境中是非常合适的路线。

参考：

- RoboMaster RMUA 2026 官方模拟器  
  https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/tree/RMUA2026-01

- 当前项目  
  https://github.com/voicepeak/RMUA2026-demo
