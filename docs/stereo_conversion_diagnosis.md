# 双目转换偏差定位与固定形状导出

2026-10-23，迭代 37。按官方规则继续离线开发：只使用先前留存的合法 RGB 输入和合成算子输入，不运行模拟器，不发布控制或业务上报。P1/P2 仍未验收。

本轮将原 TensorRT FP32 大偏差定位到 EdgeNeXt 的 Fourier 位置编码分支，并增加可选导出优化：固定只依赖 `(B,H,W)` 的投影前网格，保留原学习投影、dtype 和内存步长。12 组原 PyTorch FP32 正反向原始视差逐元素零差、有效掩码相同；优化后的完整 TensorRT FP32 与原 PyTorch FP32 的最低掩码 IoU 为 0.99993066，共同有效像素绝对视差差 P99 的最大值为 0.00105286 原图 px。这是同精度转换对照结果，不是深度精度或在线验收。

原 AMP 基线与 FP32 自身存在差异。既有 AMP 兼容性门限保持 IoU≥0.9、P99≤1px，优化 FP32 场次仍以 `FAILED / evaluate returned 2` 保留；不能用新的 FP32 对照改写旧检查结果。默认在线路径仍是 PyTorch，图像默认预算仍为 150ms，没有生成可信 FREE 或接入控制。

## 排查证据

所有实际实验目录和 SHA256 见 [机器可读证据](evidence/p2_stereo_conversion.json)。完整载荷、引擎、日志留在忽略的 `runs/`，不随 Git 分发。

| 对照 | 实际结果与适用范围 |
|---|---|
| 原 ONNX，ORT CUDA，12 组 | 对原 PyTorch FP32 最低 IoU 0.99993582，最大共同像素 P99 0.00067139px；支持原导出基本一致。profile 为 CUDA 69168、CPU 1848 个事件，包含 CPU 形状辅助处理，不能声称全 GPU。 |
| 原图重建，构建/运行都设 `NVIDIA_TF32_OVERRIDE=0` | 仍有明显偏差；首组对 ORT IoU 约 0.82011、P99 约 2.527px。全局 TF32 开关单独不能解决。 |
| 输入归一化与缓冲区存活期 | 首组归一化顺序差为零；持有输入并同步后的输出与后端逐元素一致；仅覆盖首组。 |
| H=1 的 GridSample 独立实验 | 8 种实际几何形状，PyTorch/ORT/TensorRT 对独立 FP64 线性插值解析参考均通过 0.0005 绝对误差门限；最大误差约 3.43e-5，TRT 对 Torch 最大约 3.58e-7。合成随机种子 4097 不是模拟器种子。 |
| 特征前缀分段 | 首卷积差为零，紧接 InstanceNorm 最大差 1.91e-6；stage0 末最大差 3.87e-5，而 stage1 末为 4.53。 |
| 第一处 XCA 分段 | Fourier `token_projection` 输出最大差已达 1.82229，然后传播到 norm/XCA。支持优先修复形状分支；尚未判定具体 TensorRT 内核/常量折叠步骤。 |

分段实验会把中间张量标为输出，改变图优化，因此用于定位，不能当作原完整融合引擎的逐层观测。首个分段实验因 ORT 和 TRT 上下文同时占用 6GiB GPU 而 OOM，保留失败；改成 ORT 先算并保留 CPU 参考、释放上下文后再执行 TRT，后续实验通过。修正后的首场复用失败场已生成的子图/引擎，旧报告 `build_s` 是准备耗时，不能当作重建时间；后续代码已分别记录复用和构建时间。

## 导出修复和复现

固定 timm 1.0.20 源码的 `PositionalEncodingFourier.forward(shape)` 只取形状，用全一张量、累加、归一化和 sin/cos 生成网格，再执行学习投影。当前权重只有这一处模块。优化捕获的是投影前网格，未替换学习投影；不是场景地图、视差缓存或种子查表。输入 `[2,64,48,60]` 的原 channels-last 步长 `[184320,1,3840,64]` 被保留，网格 SHA256 为 `64d84a8d4c35d2a9f7a8f3bcd981db8e1594949d34f93726c377abc49383efb7`。非 tracing 调用拒绝不同形状，导出图/后端本来就固定为 `[1,3,384,480]`。

```bash
python3 -m evaluation.seed_runner.trt_stereo \
  --source-run runs/20261023T075301Z_5c1a906b01644ee091c1d183006f422d \
  --precision fp32 --disable-tf32-globally --freeze-fourier-inputs \
  --fp32-reference-run runs/20261023T103809Z_3c9587293ea84e509ea34a79ae6e6122
```

实际结果在 `runs/20261023T113606Z_75b171efede644539586bc3d0a0b0c28`。`fourier_parity.json` 保留全部 12 组零差检查，未通过就不导出。新 ONNX SHA256 `c599bf9152d352b7a2ac1eab491113002a9ac49a19529df0b8ef90cbbdf49fed`，FP32 引擎 SHA256 `5aafcf0ef07fa18b42c20019a4a91dd5f05d9039256a760a30b07cd7419eabd1`。原 AMP 评测和另行生成的 FP32/ORT 对照分开保存。构建约 164s；FP32 推理时间仍超过默认图像预算，没有解决在线时效。

ORT 固定为 1.20.2，新增独立 55 包锁文件，在既有容器的隔离 venv 内使用 CUDA12/cuDNN9。首场缺少 `libcudnn_adv.so.9` 导致 CUDA provider 不可用，拒绝 CPU 回退并保留失败。后续通过进程局部 `LD_LIBRARY_PATH` 指定现有驱动和固定 venv 的 NVIDIA 库目录解决，未改驱动或基础环境。版本兼容和 TF32 选项参考 [ORT CUDA 官方文档](https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html)，采样定义参考 [NVIDIA GridSample 文档](https://docs.nvidia.com/deeplearning/tensorrt/10.x.x/_static/operators/GridSample.html)。

## 验证与剩余工作

宿主 146 项单元测试通过，Noetic 143 项通过、3 项依赖 Git 的测试跳过。新增 4 项测试用手算数值检查插值、零填充、批次/通道隔离及非有限失败记录；实际 8 形状 GPU 参考实验独立通过。11 场源码归档按文件重新解包验 SHA256，原输入 39 文件及新 ONNX/引擎摘要重核通过。各场源码版本不同，证据逐场列出，未声称所有场次使用同一源码。

完整优化 FP32 场次后仅收紧宿主启动器的参考 manifest/NPZ 摘要与退出检查；模型、导出及数值后端代码没有再改。最终 303 文件源码快照在 `runs/20261023T114324Z_2d68cb7f5aa44ac8a4163bb736141546` 的第二轮算子实验中归档并验证。文档在实验完成后追加。下一轮验证同图的 FP16 数值和时延，再决定是否可进入只读在线实验；曝光/外参、VIO、可信自由空间和 P1 未完成项继续开放。
