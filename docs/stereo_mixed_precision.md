# 混合精度约束实验

2026-10-23，迭代 39。上一轮同图 FP16 有两组数值失败，本轮新增可选 `sensitive_fp32` 策略并实际重建引擎，验证归一化、归约和部分非线性计算使用 FP32 是否足够。结果为 11/12，仍未通过；保留失败，未接在线或控制。P1/P2 未验收。

开始前再次读取官方规则基线、上轮重新下载并验摘要的原件，以及重构设计第 5.2、13 节。只使用已留存的合法双目 RGB；不启动模拟器，不使用真值/debug/场景信息。默认 150ms 图像预算与既有数值门槛未变。

## 实现与依据

[NVIDIA 精度说明](https://docs.nvidia.com/deeplearning/tensorrt/10.x.x/inference-library/accuracy-considerations.html)将归约、Softmax 等列为低精度的潜在敏感计算，并建议对敏感计算使用较高精度。[逐层约束文档](https://docs.nvidia.com/deeplearning/tensorrt/10.x.x/inference-library/precision-control.html)说明，弱类型网络需要设置 layer precision/output type，并使用 `OBEY_PRECISION_CONSTRAINTS` 才会在无法满足约束时失败。以上是候选选择依据，不能代替本模型的根因实验。

新增 `evaluation/metrics/trt_precision_policy.py`，可选策略匹配 NORMALIZATION、REDUCE、SOFTMAX 和 SIGMOID/TANH 浮点层，约束输入/输出精度；归一化还设置 compute_precision。整数形状运算不匹配。写出每层约束并检查 setter 保留值，没有匹配或约束丢失就拒绝。构建入口默认策略仍为 `default`；混合策略只接受 FP16 构建，新增详细引擎 inspector 归档。

实际解析同一 ONNX 后匹配 92 层：38 个 normalization、16 个 reduce、6 个 softmax、32 个 sigmoid/tanh。SDK 为固定 TensorRT 10.13.3.9；两个接口预检分别遇到 `ILayer` 无 compute_precision 和 activation 属性名错误，日志保留。按固定 SDK 的实际 Python 接口显式转换为派生层类型，并用 `IActivationLayer.type` 读取激活类型，随后完整构建通过。不能把前两次预检称为通过。

## 12 组实际对照

实际场次 `runs/20261023T120600Z_e95743bdf0714d0cb6ceb42983aa5f3d`，完整构建约 371.92s。继续使用上轮同一份冻结 Fourier 输入的 ONNX，SHA256 为 `c599bf9152d352b7a2ac1eab491113002a9ac49a19529df0b8ef90cbbdf49fed`；新引擎 SHA256 为 `85100f3d423e59cf3969b6edb231da940bd3c92b0988322a2d46445f96602399`。全局 TF32 override 为 0，TensorRT TF32 flag 关闭，workspace 仍为 512MiB，aux streams 仍为 0。

| 数值参考 | 通过 / 总组数 | 最低有效掩码 IoU | 共同有效像素差 P99 最大值（原图 px） |
|---|---:|---:|---:|
| 原 PyTorch AMP | 11 / 12 | 0.84814296 | 0.57113159 |
| 原 PyTorch FP32 | 11 / 12 | 0.85143008 | 0.71272705 |
| 原 ONNX，ORT CUDA FP32 | 11 / 12 | 0.85139943 | 0.71289398 |
| 修复后的 TensorRT FP32 | 11 / 12 | 0.85141986 | 0.71274780 |

四种参考均只有 0000 失败。全部共同像素 P99 已低于 1px，但 0000 的掩码 IoU 仍低于 0.9。相对原 AMP，0001 从上轮 1.36014px 降至 0.18742px，0000 的 P99 从 1.21530px 降至 0.57113px，但 IoU 从 0.87185 降至 0.84814。共同像素误差改善不等于有效区域一致，不能只选改善项宣布修复。

双向推理/缩放/传输中位数约 117.45ms，加灰度/掩码约 144.89ms，比上轮约 121.96ms 的整体离线耗时更高。首组重复输出逐元素一致，原始输出全部有限。该耗时不包括 ROS 来源年龄、IPC、模拟器争用或控制，未证明默认 150ms lease。manifest 和原 AMP 评测保留 `FAILED / evaluate returned 2`。

## 新定位证据

仅将原 FP32 视差输出舍入 FP16 再转回 FP32，随后使用原过滤器重新计算左右一致性和纹理掩码：12/12 通过，最低 IoU 0.99455626，最大共同像素 P99 0.05611099px。此实验没有执行低精度模型，只支持“完整 FP16 偏差不能仅由最终输出舍入解释”，不能证明任何内部层是根因。固定 OpenCV 4.10.0.84、NumPy 1.26.4，24 份原始图像/参考 NPZ 前后摘要一致。

详细 inspector 的 metadata 可关联全部 92 个请求约束，完整 JSON 随实验保留。融合 metadata 不是每个操作内部计算的逐指令轨迹，不据此宣称已观测全部内部精度。

另有直接可见的差异：4 个融合几何节点包含 16 个 GridSample，列出的特征、归一化坐标和输出张量格式均为 Half。固定上游 `core/foundation_stereo.py` 在调用 `Combined_Geo_Encoding_Volume` 时显式把输入转为 `self.dtype=torch.float32`。因此下一轮应受控验证几何采样与坐标链的 FP32 约束；当前证据还不证明这就是剩余偏差的根因。

## 复现与验证

```bash
python3 -m evaluation.seed_runner.trt_stereo \
  --source-run runs/20261023T075301Z_5c1a906b01644ee091c1d183006f422d \
  --precision fp16 --precision-policy sensitive_fp32 \
  --disable-tf32-globally --freeze-fourier-inputs \
  --reuse-onnx-run runs/20261023T113606Z_75b171efede644539586bc3d0a0b0c28
```

最终可执行代码的宿主 146 项测试通过；Noetic 143 项通过、3 项 Git 测试跳过。策略的实际 SDK 接口、逐层匹配、严格约束和引擎输出由本次真实 GPU 构建/12 组推理验证，不用这些通用单测冒充模型数值验收。308 文件源码归档独立解包逐文件验 SHA256，运行前后及追加文档前的实际源码一致；39 份原输入摘要与 ONNX/引擎摘要重新核验。原上游源码、权重与运行环境未改。具体数据见 [p2_stereo_mixed_precision.json](evidence/p2_stereo_mixed_precision.json)。

这仍是离线候选实验。低精度数值、在线时效、曝光/外参、不确定度、VIO 与可信自由空间仍待验证；后续赛程和 P1 未完成项继续开放。
