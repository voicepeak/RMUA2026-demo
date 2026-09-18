# RMUA 2026：YOLO Gate 识别与数据标注方案

## 0. 背景与结论

现有 OpenCV Detector（白/橙颜色 + 四边形）误检率高：亮建筑、远处发光字、天空块都会被当成 Gate。
原因是缺少“门是带洞的闭合框”这一结构约束。

本方案选择 **直接上 YOLO**：
- Detector 只负责输出 **Gate 2D 位置（四角 + bbox + confidence）**；
- 后面的左右匹配、双目三角化、坐标变换、Gate Chain、动态 Z 规划 **全部不变**；
- 因此只用替换一个 Detector，风险最小。

---

# 1. 统一接口（必须固定）

无论 OpenCV 还是 YOLO，Detector 统一输出：

```text
GateObservation:
  corners: [TL, TR, BR, BL]   # 图像像素 (u,v), 顺序固定
  center:  (u, v)
  bbox:    (x, y, w, h)
  confidence: float
  track_id: int (可选)
```

推荐让 YOLO 直接预测 **4 个关键点（Keypoints）**，天然对应四角，最适合双目三角化。
如果一开始标注四角太累，可以先只标 bbox，后续再升级到四角。

---

# 2. 为什么用四角 Keypoint 而不是只用 bbox

- Gate 中间是空的，不能取“中心像素”的深度（会取到门后的建筑）；
- 需要门框四角的真实三维点，才能：
  - 求门中心 = 四角平均；
  - 求门平面法向 = 四角叉乘；
  - 生成 Approach / Center / Exit 三点；
- 四角在左右图中一一对应，双目三角化最稳。

---

# 3. 数据采集

## 3.1 采集节点

新增 `gate_dataset_capture.py`（ROS）：

```text
订阅: front_left/Scene, front_right/Scene (同步), pose_gt
保存到: <dataset>/raw/
   <seq>_L.png
   <seq>_R.png
   poses.csv: seq, t, x, y, z, qx, qy, qz, qw
```

采集频率可用参数控制（例如 2 Hz），避免短时间存海量图片。

## 3.2 多样性要求

不要只采“正对、近距离、无遮挡”。应尽量覆盖：

```text
不同 seed
不同距离（远/中/近）
不同高度 / 俯仰
不同 yaw（斜视角）
不同亮度/阴影
部分遮挡
多个 Gate 同框
Gate 很小时
背景干扰（建筑、文字、天空）
```

建议首轮先采 **800~1500 张左图**（左右图成对保存）。

---

# 4. 标注方案

## 4.1 标注格式

推荐 **YOLOv8-pose**（4 关键点）。标签文件 `<seq>.txt` 每行一个 Gate：

```text
class  cx cy  w h   x1 y1 v1   x2 y2 v2   x3 y3 v3   x4 y4 v4
```

- `class`：0 = gate（只有一个类别）；
- `cx cy w h`：bbox 中心与宽高（归一化到 0~1）；
- `(xi yi)`：四个角点（归一化坐标）；
- `vi`：可见性，`2` 可见 / `1` 被遮挡 / `0` 不可见；
- **关键点顺序必须固定**：`1=TL(左上) 2=TR(右上) 3=BR(右下) 4=BL(左下)`。

> 注意：YOLO 的归一化坐标 = 像素 / 图像宽高（960 × 720）。

## 4.2 推荐标注工具

| 工具 | 说明 |
| --- | --- |
| **X-AnyLabeling**（推荐） | 支持 YOLO-Pose 关键点，桌面端，AI 辅助预标注 |
| **CVAT** | 网页端，支持 keypoints，适合多人协作 |
| **Roboflow** | 网页端，可导出 YOLOv8-pose 格式，带自动标注 |
| **labelme** | 关键点功能折中，导出后需脚本转 YOLO 格式 |

## 4.3 标注流程（推荐）

1. 跑模拟器 + 采集节点，边飞边存 `raw/`；
2. 用 X-AnyLabeling 打开 `raw/` 图片；
3. 按顺序点 4 个角：**左上 → 右上 → 右下 → 左下**；
4. 保存，导出为 YOLO-Pose 格式；
5. 用脚本按 8:2 划分 train/val，生成 `data.yaml`；
6. 训练 → 部署。

## 4.4 目录结构

```text
yolo_gate/
  images/train/  *.png
  images/val/    *.png
  labels/train/  *.txt
  labels/val/    *.txt
  data.yaml
```

`data.yaml`：

```yaml
path: /path/to/yolo_gate
train: images/train
val: images/val
kpt_shape: [4, 3]        # 4 个关键点, 每个 3 个值(x,y,visible)
flip_idx: [1, 0, 3, 2]   # 左右翻转时的关键点对应
names:
  0: gate
```

---

# 5. 训练

使用 Ultralytics YOLOv8-pose：

```bash
pip install ultralytics
yolo pose train model=yolov8s-pose.pt data=data.yaml epochs=100 imgsz=960 batch=8
```

要点：

```text
imgsz 用 960 (与相机同宽) 或 640 折中
先小模型 yolov8n/s-pose 验证，再放大
数据量少时多做增强（亮度/裁剪/透视/遮挡）
```

如果只标 bbox，则用普通检测：

```bash
yolo detect train model=yolov8s.pt data=data.yaml epochs=100 imgsz=960
```

---

# 6. 部署到 ROS

新增 `gate_yolo_node.py`：

```text
订阅: front_left/Scene (+ front_right)
推理: YOLO(.pt) -> bbox + 4 corners
发布: /rmua/gate_observations (JSON) 与 /rmua/gate_detection/left (可视化)
```

- 左右两图分别推理，再交给现有 `gate_stereo` 做匹配/三角化；
- 世界坐标、Gate Chain、动态 Z 规划 **不改**；
- confidence 阈值、NMS IoU 作为参数暴露，方便调。

---

# 7. 验收指标

```text
单帧:   precision > 0.95, recall > 0.90
连续:   门链 s 单调, 无误触发; 同门 world XYZ 重复观测 σ < 0.3 m
端到端: 10 门顺序通过率
```

---

# 8. 实施顺序

```text
Step 1  采集节点 + 跑模拟器存 raw 图片 (本方案第 3 节)
Step 2  标注 4 角 (本方案第 4 节)
Step 3  训练 YOLOv8-pose
Step 4  gate_yolo_node 替换 OpenCV Detector
Step 5  双目匹配/三角化/规划保持不变, 端到端复测
```

当前先做 **Step 1 + Step 2**：边跑边截图、给出可标注的数据。

---

# 9. 与现有系统的关系

```text
Detector(现在是 OpenCV / 将来是 YOLO)
        ↓  GateObservation(四角)
Gate Matcher / Triangulator / World Transform / Tracker
        ↓
Gate Chain / Altitude Profile / Command Arbiter
        ↓
vel_body_cmd
```

只替换最上面一层，其它不动。
