# 固定 Fourier 网格后的 FP16 对照

2026-10-23，迭代 38。复用 [迭代 37](stereo_conversion_diagnosis.md) 已验证的同一份 ONNX，构建真实 TensorRT FP16 引擎并评测全部 12 组开发输入。结果仍未通过数值门槛，未进入在线默认路径或控制。

先重新取得官方规则中心附件列表并下载两份 PDF，名称、长度、SHA256 均与 `configs/official_rules_baseline.json` 一致；重新读取规则原文与[官方模拟器 README](https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/blob/RMUA2026-01/README.md)，版本仍为 12.0.0.5。完整重构设计第 5.2、13 节要求模型候选独立比较、记录整链路时延，不能用离线 GPU 时间代替实时感知验收。本轮不启动模拟器，不发布控制。

## 实际结果

运行目录 `runs/20261023T115137Z_fe9cdb1e1c1744f69d159f5d06abdc78`，源码为提交 `710652db18d37f59e1027c30fdbe648ba69718a5`。构建约 490.47s，构建和推理均设置 `NVIDIA_TF32_OVERRIDE=0`，TensorRT TF32 flag 关闭。ONNX SHA256 `c599bf9152d352b7a2ac1eab491113002a9ac49a19529df0b8ef90cbbdf49fed` 与上轮完全相同；新 FP16 引擎 SHA256 `17d409b57807783fa223af128369486af0e2de96698797244b93f5add0d05137`。

| 数值参考 | 通过组数 / 总组数 | 最低有效掩码 IoU | 共同有效像素差 P99 的最大值（原图 px） |
|---|---:|---:|---:|
| 原 PyTorch AMP | 10 / 12 | 0.87185187 | 1.36014080 |
| 原 PyTorch FP32 | 10 / 12 | 0.86446967 | 1.96202087 |
| 原 ONNX，ORT CUDA FP32 | 10 / 12 | 0.86446081 | 1.96231079 |
| 修复后 TensorRT FP32 | 10 / 12 | 0.86445917 | 1.96238708 |

门槛沿用 IoU≥0.9 且共同有效像素 P99≤1px，未放宽。四种参考失败的都是 0000/0001。相对原 AMP，0000 的 IoU/P99 为 0.87185187/1.21529968px；0001 为 0.94595254/1.36014080px。结果表明单纯开启 FP16 并未保住上轮恢复的 FP32 数值一致性，不能认为形状分支修复同时解决了低精度误差。还未定位具体低精度敏感层，不能直接归因于某个 normalization 或 attention 内核。

双向推理、缩放和传输中位数 95.15ms，加入灰度/掩码处理为 121.96ms；首组重复输出逐元素一致，全部原始输出有限。该耗时来自无模拟器争用的离线实验，不含 ROS 到达年龄、IPC、调度、规划和控制。即使小于 150ms，也不证明输出能在默认原始 lease 内返回。原 `trt_comparison.json`、manifest 的 `FAILED / evaluate returned 2` 保留。

## 归档与复现

```bash
python3 -m evaluation.seed_runner.trt_stereo \
  --source-run runs/20261023T075301Z_5c1a906b01644ee091c1d183006f422d \
  --precision fp16 --disable-tf32-globally --freeze-fourier-inputs \
  --reuse-onnx-run runs/20261023T113606Z_75b171efede644539586bc3d0a0b0c28
```

本轮独立解包并逐文件重算 305 个源码文件，运行前后和提交内容一致；原输入索引/载荷/参考等 39 文件重算摘要，ONNX 与引擎摘要均通过。原模型权重、上游源码和隔离运行环境未改变。另行对四种参考的原始 NPZ 重算交并比、共同有效误差和可获得的正反向误差；原 AMP 归档仅有 forward 原始值，reverse 不可得，未补造。审计脚本最初假定它有 reverse 并失败，修正字段选择后重新计算，GPU 实验本身未重跑或改写。

宿主 146 项、Noetic 143 项通过及 3 项跳过的验证沿用本轮父提交；本轮只新增说明和离线证据，没有修改可执行代码。详细结果、每组误差、文件摘要和复核信息见 [p2_stereo_frozen_fp16.json](evidence/p2_stereo_frozen_fp16.json)。大型原始数据、引擎和日志仍留本机 `runs/`。

下一步需要受控验证低精度敏感层与精度约束，并继续曝光/外参和独立几何标定。通过数值门槛后再评估默认在线候选；P1/P2 尚未验收，P3–P7 仍开放。
