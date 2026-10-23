# 学习双目与 SGBM 离线对照（2026-10-23，迭代 34）

按重构设计 5.2/5.4 与 Sprint C，跑通官方 Fast-FoundationStereo 实际 GPU 推理和独立稀疏对照。当前是离线像素视差研究结果，真实精度、整链路延迟及 P2 尚未验收。比赛约束继续遵循 [官方原件核对](official_rules_review.md)。

使用迭代 33 的开发种子 2110/2111 各 6 组同场输入，共 12 组 960×720 双目。推理只读取 RGB 图像和原图元数据；LiDAR/历史位姿由独立准备及评估进程读取。没有真值、静态门表或 debug 输入，没有控制发布、深度融合或 FREE 更新。

| 来源或配置 | 本轮版本 |
|---|---|
| [官方代码](https://github.com/NVlabs/Fast-FoundationStereo/tree/476f4249561f7c79ca707326954f9255643412a6) | `476f4249561f7c79ca707326954f9255643412a6`，工作树干净 |
| [官方 C-FFS 权重](https://huggingface.co/nvidia/c-fast-foundationstereo/tree/53a84e28a368f37e699776250215fcb450d6a109) | `model_best_bp2_serialize.pth`，71,105,659 bytes |
| 权重 SHA256 | `7aee85948373da62b0503c2542507129a3e7cab9d97d10e6790d89512a7db214` |
| 硬件 | RTX 3060 Laptop，6GiB；容器已有 595.91.07 驱动库 |
| 环境 | 独立 CPython 3.12.11、Torch 2.6.0+cu124、OpenCV 4.10.0；完整版本见 `configs/fast_foundation_stereo_runtime.lock.txt` |
| 推理 | INTER_AREA 缩至 480×360，pad32 至 480×384，4 次迭代，模型最大视差 192，FP16 |

RGB 0–255 输入由上游模型内部归一化。另跑一次交换左右并同时水平翻转的推理，再翻回得到右图正视差；两张视差最近邻恢复原图后除以 0.5，单位为原图像素。分别记录原始正值、左右一致性和加入双侧 5×5 纹理标准差≥2 的共同门限。共同门限使用原图 1px 一致性阈值，边界拒绝，无效值为 NaN；门限不是标定置信度。

bp2 checkpoint 缺少当前代码读取的 `args.normalize`。仅在缺失时补 `True`，报告记录兼容分支；依据 [NVIDIA TAO 官方 FFS 配置说明](https://docs.nvidia.com/tao/tao-toolkit/7.3.0/text/cv_finetuning/pytorch/depth_estimation/fast_foundation_stereo.html) 对 bp2 的 `gwc_feature_normalize: true` 要求，冻结上游导出脚本亦默认 True。没有按本批残差选值。源码 LICENSE 与模型协议分别保留，本轮仅研究评估，未加入发布镜像；上游研究检查点的 RTX3090 性能表不能当成本轮 C-FFS 实测。

参考由原始 LiDAR、三份 ESKF 历史头位姿和固定名义安装几何重新投影。要求左右在图内、稀疏 z-buffer 可见和名义视差为正，共 6,305 点；选择不读取任一方法的视差或残差。每种方法分别要求采样四邻域有效，另比较两者同一参考点交集，没有使用旧 SGBM `comparison_valid` 缩小参考范围，也不删除大残差。两方法使用相同 OpenCV、单 CPU 线程。SGBM 搜索 128 原图像素，学习模型为 384，搜索范围不同。

| 12 组统计 | SGBM | C-FFS |
|---|---:|---:|
| 共同门限有效图像比例，范围 | 8.88–17.94% | 19.54–31.09% |
| 共同门限有效图像比例，中位数 | 17.18% | 27.39% |
| 各自有效参考点，总数 | 1,684 | 2,230 |
| 相同交集参考点，总数 | 1,412 | 1,412 |
| 交集名义视差绝对差，中位数 | 0.297px | 0.233px |
| 交集名义视差绝对差，P90 | 0.558px | 0.606px |
| 计算耗时，中位数 | 282.35ms | 153.45ms |
| 计算耗时，样本 P95 | 285.85ms | 165.68ms |

学习模型原始正值覆盖 98.64–99.99%，左右一致性后 30.98–43.64%。各组各自有效参考点的绝对差 P90 仍约 2.37–10.88px。交集的较小中位差不能证明新增覆盖安全；稀疏投影仍有遮挡、外参/曝光/逐点时间等未验证因素，以上不是 ground-truth 精度或泛化结论。

学习计时包括双向网络、缩放/padding、CPU/GPU 传输、原图灰度及掩码；SGBM 包括灰度、双向匹配及掩码，未另做独立预热。均不含图像读取、ROS、模拟器争用、规划。相关开发样本的分位数只作描述。加载约 1.51s，学习模型独立预热约 1.15s，已有编译缓存；不是冷进程启动或 30s 出发验证。CUDA 峰值 allocated/reserved 为 469,507,584 / 580,911,104 bytes，不含模拟器/驱动全部开销。

首对在全部 12 组后重复，正反输出组合 SHA256 均为 `cf7bb60db261696fc88751af463bbc2d69c4b87f25493cff8ab419158b598572`。原始 stamp、valid_until、epoch 保留，离线计算不续期；耗时还须叠加输入年龄，不能宣称在线有效或 10–20Hz 整链路。

复现入口（需保留本机 captures 与隔离环境）：

```bash
python3 -m evaluation.seed_runner.learned_stereo \
  --source-run runs/20261023T065455Z_abd6fdcb40e8464ab03367bc3e81d3fe \
  --source-run runs/20261023T065832Z_cee28fe232c844808f28c88ba0ba63a8
```

每次独立 UUID 保留 main 精确源码归档、上游 Git 归档、依赖 freeze、实际驱动文件哈希、权重/输入哈希及各阶段日志。环境在忽略目录 `runs/ffs_runtime/`：uv 0.8.22 的 PyPI wheel SHA 为 `8efec4ef5acddc35f0867998c44e0b15fc4dace1e4c26d01443871a2fbb04bf6`，安装独立 CPython 3.12.11，Torch/torchvision 使用官方 cu124 索引，其余包来自 PyPI。移植时按完整 lock 安装到独立 venv。Noetic Python3.8 只用于准备/评估，Python3.12 推理使用直接 docker exec 的独立 PYTHONPATH，不能经 `enter_env.sh` 启动。

当前容器系统 libcuda 是空占位，有效匹配库位于已有 `/opt/rmua-driver`。运行器用 `TRITON_LIBCUDA_PATH` 指向该目录，可由 `RMUA_TRITON_LIBCUDA_PATH` 覆盖；实际库哈希记录在 manifest。未更换驱动或重启。准备缓存权限、错误库链接、缺 normalize 的三个早期失败，以及成功先导/旧 NumPy 回归失败均保留。旧 NumPy 的 NaN 比较问题修正为先筛有限值，再做范围比较，最终重跑。

权威运行为 `runs/20261023T075301Z_5c1a906b01644ee091c1d183006f422d`。273 文件源码归档及上游归档独立复核；源码、原始输入、权重及 vendor 均运行前后一致。宿主 133 项单测通过；Noetic 130 通过、3 项 Git 测试跳过。新增测试覆盖颜色顺序、缩放单位、正反坐标、纹理/一致性拒绝、NaN/Inf/极端有限值、相同交集和不按残差删点。小型证据见 [p2_learned_stereo.json](evidence/p2_learned_stereo.json)，运行后仅追加证据与说明。

下一步测量有界只读在线队列与同 GPU 模拟器争用，并继续曝光/外参/遮挡和不确定度验证。P1 冲击根因与控制验收仍开放。
