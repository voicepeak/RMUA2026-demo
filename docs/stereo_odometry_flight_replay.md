# 双目相对运动的飞行记录对照（迭代 46）

2026-10-27。复用先前独立采集的正式双目 Scene 图像和飞行实验，不启动新的控制场次。`evaluation.pipeline.flight_stereo_motion` 分为两个独立命令：`estimate` 只读图像、采集索引和名义相机设置并先保存估计；之后 `evaluate` 才读取隔离的 debug 真值进行离线比较。真值不进入 VO 或飞控。

左右图像以时间差不超过 5ms 一对一匹配，未配对帧保留；采集是 latest-only 的有损记录。真值仅在相邻记录间隔不超过 50ms 时插值，不外推。两个开发场次均为小范围悬停或低速运动，不代表比赛高速与工厂场景。

| 已录制场次 | 配对图像 | 相邻对估计 | 拒绝 | 平移误差 p50 / p90 / 最大 | 旋转误差 p50 / p90 / 最大 |
|---|---:|---:|---:|---|---|
| seed1016 PWM | 28 | 23 | 4 | 0.051 / 0.177 / 0.306 m | 0.0025 / 0.0083 / 0.0155 rad |
| seed2024 VEL | 23 | 21 | 1 | 0.037 / 0.080 / 0.103 m | 0.0019 / 0.0037 / 0.0050 rad |

PWM 三张左图和三张右图未能按时配对；五次估计拒绝均为 `invalid PnP reprojection`，没有作为成功统计。实际比较的相邻对真值位移中位数分别约 0.084m 和 0.018m，最大约 0.316m 和 0.285m；因此部分平移误差与运动量同量级。PNG 和时间来源、逐帧数据、输入摘要、拒绝及估计与评测文件摘要见 [机器可读证据](evidence/p2_stereo_flight_replay.json)。本机保存两阶段的完整估计和评测；首次 PWM 处理约 1.72s，其余耗时及算法是离线诊断，不证明实时性。

该结果表明上一轮静止场景的结果不足以支持平移用于定位；不能以较小旋转误差掩盖平移误差，也不能直接加入飞控状态。下一步检查匹配、三角化和 PnP 在运动期间的误差来源，用独立的飞行记录验证候选后再考虑带延迟和相关性的滤波融合。宿主 199 项测试通过，Noetic 196 项通过、3 项跳过。此次不构成 P1/P2 或比赛验收。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m evaluation.pipeline.flight_stereo_motion estimate runs/20261020T082058Z_e20f9b9fcb094e0bb6463b65555c7871 /tmp/rmua_flight_estimates.json
python3 -m evaluation.pipeline.flight_stereo_motion evaluate /tmp/rmua_flight_estimates.json runs/20261020T082058Z_e20f9b9fcb094e0bb6463b65555c7871/evaluation_truth.jsonl /tmp/rmua_flight_comparison.json
```
输出文件须不存在，避免覆盖先前结果。
