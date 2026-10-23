# TensorRT 导出、性能与数值门限（2026-10-23，迭代 36）

沿用 [官方比赛要求](official_rules_review.md) 与设计 5.2/13 节，实际构建了两个 TensorRT 引擎，并在迭代 34 的 12 组同图输入上验证。FP16 有离线速度收益，但两种精度都未通过相对原 AMP 候选的数值兼容门限，因此没有接入在线入口、融合、FREE 或控制。P1/P2 尚未验收。

使用冻结的 NVlabs 源码 `476f4249561f7c79ca707326954f9255643412a6`、官方 C-FFS checkpoint SHA `7aee85948373da62b0503c2542507129a3e7cab9d97d10e6790d89512a7db214`。遵循 [官方单 ONNX 导出脚本](https://github.com/NVlabs/Fast-FoundationStereo/blob/476f4249561f7c79ca707326954f9255643412a6/scripts/make_single_onnx.py)：以可导出算子替换 cost volume，剥离模型内部图像归一化。运行时在 GPU 上、网络外的预处理步骤执行同一 ImageNet 归一化，再 pad/unpad；缩放仍为 INTER_AREA 0.5，4 次迭代、最大模型视差 192，原图恢复为最近邻/0.5。左右双向和原图 1px/双侧纹理门限保留。

隔离环境为 `runs/ffs_runtime/venv_trt`，基于原 46 包版本另建，增加 TensorRT-cu12 10.13.3.9、ONNX 1.17.0、protobuf 7.36.2。完整版本见 `configs/fast_foundation_stereo_trt_runtime.lock.txt`，原 ROS 与 PyTorch 环境未升级。按 [NVIDIA 10.13.3 发布说明](https://docs.nvidia.com/deeplearning/tensorrt/10.x.x/getting-started/release-notes-10/10.13.3.html) 核对 CUDA/驱动兼容，并实际验证 Builder 与 GPU 执行；使用 Python API 构建，不依赖 pip 包未提供的 trtexec。TensorRT libraries wheel 来自 NVIDIA 索引，版本对应文件 SHA 为 `bf2008eca911411aa93b852825ea992de33396451ea11713a6eb97c411d3b2e9`；PyPI 源描述与完整依赖随运行保存。

导出静态输入为两个 `(1,3,384,480)` FP32 张量，输出 `(1,1,384,480)` FP32。首组实际图像的原始 FP32 模型与导出替换前向逐像素差为零，ONNX checker 通过；这仅检查该样本下的替换，不替代引擎验证。两个构建复用同一 ONNX，SHA 为 `4efab5f9739c608a346e105aa57e488fbc47653dab2548ee2fca594f99502159`。TF32 禁用，aux streams=0，workspace 池上限 512MiB；该上限不是整个引擎内存上限。权重、引擎、输入、源码及 SDK 版本都记录哈希，SDK/GPU 绑定的生成文件留在本机忽略目录。

提前设定的工程兼容门限：两候选有效掩码 IoU≥0.9，同一有效像素的原图视差绝对差 P99≤1px，所有原始输出有限，首对重复逐字节一致。导出替换另检查 FP32 差值 P99≤0.05 模型像素、最大≤0.5。它们是数值诊断条件，不是比赛规则、真实深度准确度或安全认证。

| 离线实测，12 组 | 原 PyTorch AMP | TensorRT FP16 | TensorRT FP32 |
|---|---:|---:|---:|
| 双向网络/缩放/传输，中位数 | 129.85ms | 95.24ms | 236.76ms |
| 加原图灰度与掩码，中位数 | 153.45ms | 122.93ms | 262.33ms |
| 加原图灰度与掩码，样本 P95 | 165.68ms | 131.47ms | 272.90ms |
| 相对原 AMP 门限通过组数 | 参考 | 11/12 | 6/12 |
| 首组掩码 IoU | 参考 | 0.7963 | 0.8156 |
| 首组共同有效像素绝对差 P99 | 参考 | 1.966px | 1.699px |

两个引擎所有原始输出有限，首对重复一致；未通过样本保留，没有删除首组或放宽门限。原 PyTorch 时间来自相同硬件和图像的迭代 34，测量窗口不同，不是交错随机 A/B；各阶段计时仍不含 ROS、模拟器争用、规划或控制。名义 LiDAR 对照沿用相同候选点和两方法交集，不依据残差筛点。两者名义差相近也不能覆盖整幅图的数值失败。

首组 FP16 对照中，共同有效像素有 2,911 个绝对差>1px，全部位于图像下半部分。派生诊断图保留在该运行的 `first_disagreement.png`，不据此把区域分类成机体、反射或真障碍。稀疏参考、曝光、外参和可见性均未认证，不能直接判定哪个候选更准确。

为了区分精度变化与转换差异，另外生成了禁用 AMP/TF32/compile 的同图 PyTorch FP32 参考。首组原 AMP 对该参考的 IoU 为 0.8977、共同视差差 P99 为 0.907px，说明 PyTorch 自身切换精度也会改变掩码；TensorRT FP32 对同精度参考的 IoU 为 0.8202、共同差 P99 为 2.527px，偏差并未消失。现有证据不足以将全部差异归因于 FP16，更不能把 FP32 自动当成修复。下一步需用 ONNX Runtime/逐层输出定位 export 或引擎计算差异，并检查低约束区域的数值敏感性。

构建耗时约 FP16 493s、FP32 166s。构建日志的规划 activation memory 约 804MB/1597MB，属于引擎内部规划；运行报告中的 `peak_cuda_allocated/reserved` 只覆盖 PyTorch I/O 分配器，不包含 TensorRT SDK 的全部内存。没有用这些数据宣称参赛镜像大小、30s 冷启动或在线有效率达标。

入口（需保留上游模型及隔离依赖）：

```bash
python3 -m evaluation.seed_runner.trt_stereo --source-run runs/20261023T075301Z_5c1a906b01644ee091c1d183006f422d --precision fp16
# 可复用已校验的同一 ONNX，仍创建新的引擎/记录 UUID
python3 -m evaluation.seed_runner.trt_stereo --source-run runs/20261023T075301Z_5c1a906b01644ee091c1d183006f422d --precision fp32 --reuse-onnx-run runs/20261023T101913Z_3188020d49db42a191bf0ff0c269a2b8
```

权威运行：FP16 `runs/20261023T101913Z_3188020d49db42a191bf0ff0c269a2b8`，FP32 `runs/20261023T103010Z_327b73a5e5b84348acb440c798b999a0`，均完整计算后在 evaluate 阶段因兼容门限返回 2，保留 FAILED。两者同一 291 文件源码归档；另外的 FP32 参考使用 `runs/20261023T103809Z_3c9587293ea84e509ea34a79ae6e6122`。参考先导因缺显式 Inductor 缓存路径触发容器 UID 查找失败，已保留并修正后重跑，未修改系统用户或 HOME。

为参考添加 `mixed_precision=False` 可选参数，默认仍为 True。原环境中首对默认 AMP 的正反向组合 SHA 仍为 `cf7bb60db261696fc88751af463bbc2d69c4b87f25493cff8ab419158b598572`，与原基线逐字节相同。宿主 142 项单测通过，Noetic 139 通过、3 项 Git 测试跳过；另有两个实际引擎与完整 FP32 参考运行。小型证据见 [p2_stereo_tensorrt.json](evidence/p2_stereo_tensorrt.json)。

本轮不改变默认在线后端或 150ms 有效期。继续定位转换偏差、压缩成本并标定不确定度；P1 冲击/制动、P2 定位感知和 P3–P7 仍开放。
