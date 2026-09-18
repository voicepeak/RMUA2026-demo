# YOLO Gate 识别（采集 / 标注 / 训练 / 部署）

本目录是 Gate 视觉识别的完整链路。OpenCV 版 Detector 见
`ros_ws/src/rmua_gate_vision/`；YOLO 版 Detector 见
`ros_ws/src/rmua_gate_vision/scripts/gate_yolo_node.py`。

## 目录

```text
yolo/
├── weights/best.pt          # 已训练权重 (yolov8s, 38图/115框, mAP50~0.75)
├── tools/xany_to_yolo.py    # X-AnyLabeling JSON -> YOLO detection 数据集
├── samples/                 # 标注格式样例 (L图 + 原生json + YOLO txt)
├── runs/                    # 训练曲线 / 混淆矩阵
└── data.yaml.example        # 数据集配置模板
```

## 1. 采集数据

用 `rmua_gate_vision/scripts/gate_dataset_capture.py`：边飞边存左右目成对图像 + pose。

```bash
source /opt/ros/noetic/setup.bash && source <ws>/devel/setup.bash
rosrun rmua_gate_vision gate_dataset_capture.py _out:=<dataset> _rate_hz:=5.0
# 生成 <dataset>/raw/<seq>_L.png, <seq>_R.png, poses.csv
```

要点：多 seed / 多距离 / 多高度 / 斜视角 / 遮挡 / 多门同框。

## 2. 标注（X-AnyLabeling, YOLO Detection）

- 类别只建一个：`gate`；
- **一道门一个矩形**（含门洞整体），远近都要标；太小(box 宽<15~20px)可不标；
- **只标本体，不标支撑柱**；
- 左右目图各自标（`*_L.png` 和 `*_R.png`）；
- Ctrl+S 保存为同目录同名 `.json`（原生格式）。

> 如需精确门中心，可升级为 4 角关键点导出 YOLO-Pose（接口固定 TL→TR→BR→BL）。

## 3. 转数据集

```bash
python3 tools/xany_to_yolo.py --src <raw或samples所在目录> --out dataset --val-seqs 225,240,255,270
# 生成 dataset/images{labels}/{train,val} 与 dataset/data.yaml
```

## 4. 训练（xal conda 环境含 torch/ultralytics）

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
pip install ultralytics
cd yolo
yolo detect train model=yolov8s.pt data=dataset/data.yaml \
  epochs=200 imgsz=960 batch=8 patience=50 close_mosaic=15 \
  freeze=10 fliplr=0.5 degrees=10 mixup=0.1 project=runs name=gate
```

> 小数据(30~40图)用于流程验证；Recall 偏低(≈0.5)。**建议标注 500+ 图**后重训，
> 可去掉 `freeze`、加大 epochs。

## 5. 部署（ROS 节点）

`gate_yolo_node.py` 用 xal 环境运行（含 torch），rospy 复用系统 ROS：

```bash
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
/home/huang/miniconda3/envs/xal/bin/python \
  <ws>/src/rmua_gate_vision/scripts/gate_yolo_node.py \
  _model:=<abs>/yolo/weights/best.pt \
  _route_file:=<ws>/src/route_follower/config/route_1_3.yaml \
  _imgsz:=960 _conf:=0.35
```

- 发布：`/rmua/gate_detection/{left,right,stereo}`（可视化）、`/rmua/gate_observations`（JSON）；
- 服务：`~save`（把稳定观测写成 gates_yaml）、`~clear`；
- 下游（双目匹配 / Gate Chain / 动态 Z / v4 控制）**不需要改**。

## 6. 与 OpenCV 版的关系

```text
Detector(OpenCV gate_detector_opencv.py 或 YOLO gate_yolo_node.py)
        ↓  统一接口 GateObservation(四角/bbox/中心/置信度)
gate_stereo -> 世界系 -> GateChain -> AltitudeProfile -> CommandArbiter -> vel_body_cmd
```

只替换最上面一层。
