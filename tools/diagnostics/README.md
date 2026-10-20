# 本地 ROS 桥接时序诊断

用于调查控制输入接收回调与命令转发之间的并发存活期疑点。它只包装冻结插件导出的 `ROTOR_PWM_Callback` 和 `PublishAsyncCommands`，透传原引用参数，不解引用参数，不读取游戏对象、赛道布局或定位真值。

`observe` 记录不同线程的重叠入口；`serialize` 用互斥串行执行这两个原函数。两种模式都会扰动时序，**所有结果均标记为插桩诊断，不具备比赛验收资格**。互斥本身导致重叠计数为零，不能据此认定它修复了原物理冲击。

原 `.so`、主程序和规则配置不改写。共享库通过显式诊断启动参数及 LD_PRELOAD 在模拟器进程内加载，不在飞行算法容器中加载，不进入部署镜像。默认启动明确关闭此路径。运行清单记录源码/构建摘要并复核原文件；不存在实际探针调用或记录写入失败时诊断不算完成。

```bash
bash tools/build_bridge_probe.sh
OPENBLAS_NUM_THREADS=1 python3 -m unittest tests.integration.test_bridge_probe -v
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 2041 --hover-seconds 14 --evaluate-truth --stationary-bootstrap --bridge-probe observe
```

改为 `--bridge-probe serialize` 可运行互斥诊断。它不是比赛环境补丁，也不能作为 P1 通过的证据。一次控制完成、一次重叠或一次正常复测都不能单独证明根因。

构建使用本机 Noetic 容器的 ABI，先固定源码副本，再原子替换输出共享库，避免覆盖正在映射的库文件。原生 mock 测试只操作自有假库，验证透传、不同线程重叠与互斥，不依赖或启动模拟器。

记录里的时间来自 CLOCK_MONOTONIC，参数与指令载荷不记录。没有最终 exit 记录时累计调用数视为下界；已有重叠事件仍可用于证明对应调用区间重叠。完整载荷核查另由 `command_trace.py` 执行，缺失记录会如实失败。

初步两场结果及限制见 `docs/evidence/p0_bridge_concurrency_probe.json`。两场构建不同，不构成冻结对照；不能把插桩结果混入正式可靠性统计。

冻结三组对照已通过 `python3 -m evaluation.seed_runner.bridge_comparison` 运行 18 场，协议为 configs/bridge_comparison_v1.json。开发种子每模式重复三次、轮换组内顺序，每场源码归档与模拟器摘要复核。结果与边界见 docs/bridge_concurrency_investigation.md；原始环境故障未关闭，P1 未验收。
