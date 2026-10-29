# 本地 ROS 桥接时序诊断

用于调查控制输入接收回调与命令转发之间的并发存活期疑点。默认 concurrency 变体只包装冻结插件导出的 `ROTOR_PWM_Callback` 和 `PublishAsyncCommands`，透传原引用参数，不解引用参数。显式 native-input 变体额外读取已识别 PWM 存储的 16 字节；所有变体均不读取游戏对象、赛道布局或定位真值。

`observe` 记录不同线程的重叠入口；`serialize` 用互斥串行执行这两个原函数。两种模式都会扰动时序，**所有结果均标记为插桩诊断，不具备比赛验收资格**。互斥本身导致重叠计数为零，不能据此认定它修复了原物理冲击。

原 `.so`、主程序和规则配置不改写。共享库通过显式诊断启动参数及 LD_PRELOAD 在模拟器进程内加载，不在飞行算法容器中加载，不进入部署镜像。默认启动明确关闭此路径。运行清单记录源码/构建摘要并复核原文件；不存在实际探针调用或记录写入失败时诊断不算完成。

```bash
bash tools/build_bridge_probe.sh
OPENBLAS_NUM_THREADS=1 python3 -m unittest tests.integration.test_bridge_probe -v
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 2041 --hover-seconds 14 --evaluate-truth --stationary-bootstrap --bridge-probe observe
```

改为 `--bridge-probe serialize` 可运行互斥诊断。它不是比赛环境补丁，也不能作为 P1 通过的证据。一次控制完成、一次重叠或一次正常复测都不能单独证明根因。

构建使用本机 Noetic 容器的 ABI，先固定源码副本，再原子替换输出共享库，避免覆盖正在映射的库文件。原生 mock 测试只操作自有假库，验证透传、不同线程重叠与互斥，不依赖或启动模拟器。

记录里的时间来自 CLOCK_MONOTONIC；concurrency/lifetime 不记录 PWM 载荷。没有最终 exit 记录时累计调用数视为下界；已有重叠事件仍可用于证明对应调用区间重叠。线上载荷核查另由 `command_trace.py` 执行，缺失记录会如实失败。

初步两场结果及限制见 `docs/evidence/p0_bridge_concurrency_probe.json`。两场构建不同，不构成冻结对照；不能把插桩结果混入正式可靠性统计。

冻结三组对照已通过 `python3 -m evaluation.seed_runner.bridge_comparison` 运行 18 场，协议为 configs/bridge_comparison_v1.json。开发种子每模式重复三次、轮换组内顺序，每场源码归档与模拟器摘要复核。结果与边界见 docs/bridge_concurrency_investigation.md；原始环境故障未关闭，P1 未验收。

`bash tools/build_bridge_probe.sh lifetime` 构建额外命令存活期包装；默认 concurrency 变体不含这些符号。用 `--bridge-probe lifetime` 显式启动，仅记录对象地址和桥接交接/释放时序，仍透传原调用，不读取载荷或游戏对象。运行后产生 bridge_lifetime_audit.json，只把严格发生在借用后、Command 交接前的释放标记为存活期问题，不声称观察了消费端实际读取或证明物理冲击因果。

```bash
OPENBLAS_NUM_THREADS=1 python3 -m unittest tests.integration.test_bridge_lifetime_probe -v
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.bridge_comparison --protocol configs/bridge_lifetime_comparison_v1.json
```

已有 8 场冻结对照与一次探索记录见 docs/evidence/p0_bridge_lifetime.json。分析器还可离线执行 `python3 -m evaluation.metrics.bridge_lifetime runs/<run-id>`。复现只用于本地调查，不进入部署镜像。

`--bridge-probe native-input` 显式构建额外 Init/原生回调包装，保留 lifetime 追踪。仅对原桥接已识别的 PWM Command，通过只读 `/proc/self/mem` 有界读取四个 float32，记录原生回调前后快照与实际调用区间；未知回调原样透传且不读取。原回调、参数与结果均原样转发。此诊断会改变调度，不能把入口快照当成消费端实际读取，也不进入部署镜像。

`native-serialize` 使用同一 native-input 构建，并对两个桥接原函数加互斥；用于同构建诊断对照，仍不证明异步消费后的对象存活或实际电机输入，也不具备比赛验收资格。

```bash
OPENBLAS_NUM_THREADS=1 python3 -m unittest tests.integration.test_native_input_probe -v
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 2041 --hover-seconds 14 --evaluate-truth --stationary-bootstrap --bridge-probe native-input
```

运行生成 native_input_audit.json，比较发送端 float64 转 float32 后的逐字节结果。离线入口为 `python3 -m evaluation.metrics.native_input runs/<run-id>`，会写该审计文件；重分析历史版本应另存结果，保留旧审计与源码归档。延迟仅汇总控制窗口内快照。探索证据、计时更正及限制见 docs/native_input_observability.md。
