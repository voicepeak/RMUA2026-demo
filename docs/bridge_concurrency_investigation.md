# 本地 PWM 角速度冲击与 ROS 桥接并发调查

2026-10-20，官方 README 仍提供 12.0.0.5。遵循已核对比赛规则，原模拟器、ROS 插件、碰撞/超时设置保持不变，RpcEnabled=false；控制节点只读取 initial_pose、IMU、GPS，指令不超过 100Hz，PWM 与 VEL 不混发。此文是本地调查记录，尚未提交上游，也不是官方缺陷确认。

## 冻结实验

运行 `OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.bridge_comparison`。协议见 `configs/bridge_comparison_v1.json`：开发种子 2024/2041，每种子每模式重复 3 次，14s 悬停，开启同一静止初始化，参数不调节；三种模式的顺序按组轮换。每场均新建模拟器，控制结束后关闭自有场次。

| 模式 | 场次 | 角速度保护触发 | 控制完成 | 独立指令审计失败 | 控制与全部评测通过 |
|---|---:|---:|---:|---:|---:|
| 原始，未插桩 | 6 | 2 | 4 | 1 | 3 |
| observe | 6 | 3 | 3 | 1 | 2 |
| serialize | 6 | 0 | 6 | 0 | 6 |

控制完成和审计通过分别统计。两场漏录分别发生于完成控制的原始/observe 场次，共缺失三条指令记录（原始 seq598，observe seq139/224），不能称为物理故障，也不能从严格结果中剔除。所有五次角速度故障的线上载荷审计通过，独立姿态记录也确认旋转；不只是估计状态异常。

18 场使用同一份 205 文件源码内容与相同探针构建，逐场复核原模拟器、插件、配置和容器摘要。批次及每场的 19 个源码归档全部独立读取、逐文件重算 SHA256，与冻结清单一致。小型证据、每场目录及摘要见 `docs/evidence/p0_bridge_frozen_comparison.json`；完整清单和原始数据保留在本地 runs/batches/20261020T103124Z_5fdab8d727de4f54b368b7df33791976 及对应运行目录。源码包含当时尚未提交的实验工具；不能仅用父提交 9a6ce9e 代替归档内容。

## 当前证据支持什么

只读检查限于公开 ROS 桥接接口，不读取游戏对象、赛道布局或额外算法输入。固定插件 SHA256 为 `69d5ea3302bdd5f20b842ec3d3e84bafcb50ef3649ada42c4d346c9bcd389885`。

- `ROTOR_PWM_Callback`（0x1862f0）接收四个 double，转换为 float 参数并替换共享参数对象。
- `PublishAsyncCommands`（0x18dcc5）在 0x18dd9d 调用参数 shared_ptr 的 get()，随后用裸指针构造 Command，再在 0x18dddf 调用原生回调；该段未见持有参数所有权的局部副本或互斥。
- 时序探针确认两函数在不同线程重叠执行。三个 observe 失败场次中，故障附近最近的重叠入口分别早于对应姿态消息到达约 20.31/20.22/15.31ms。该间隔使用 CLOCK_MONOTONIC 与记录进程到达时间，是消息到达相关性，不是电机施加延迟或指针使用时刻。
- serialize 仅在两个原函数外加互斥，六场未出现角速度保护。它改变线程调度；少量零故障样本不能证明根因或普适修复，原始运行故障仍未关闭。

实际退出未产生探针最终 exit 记录，调用计数仅为下界；不能凭最后一个周期计数推断整场精确调用频率。宿主 68 项单元测试通过；Noetic 65 项通过、3 项需 Git 的宿主归档测试跳过。

静态所有权疑点、实际重叠和冻结对照共同加强了桥接并发方向的调查依据。尚无“同一参数对象被借用后销毁、再由消费端访问”的完整存活期追踪，不能宣称已经证明 use-after-free，也不能用低频 debug PWM 排除实际输入瞬变。

## 下一项验证及修复边界

下一步只在本地诊断中追踪该参数类型的 get/销毁和 Command 交接时序，透传原参数，核验同一对象的存活期；先用自有 mock 验证覆盖范围与透传，再判断实际是否出现提前销毁。继续保留未插桩对照。

若上游确认此处存在并发所有权问题，修复候选是：接收端在互斥下发布不可变参数所有者；发送端在相同保护下复制持有所有权的快照，再交给消费端。若消费是异步的，Command 必须持有该所有者直至消费完成。该建议尚未在官方实现验证，不作为当前项目完成项。

诊断库不进入部署镜像，不以更改官方模拟器完成 P1 验收。competition runtime_control_enabled 仍为 false。持续 IMU 失联处理、完整制动包络、正式权限与后续 P2–P7 均需继续实现和验证。

官方来源：[模拟器 README](https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/blob/RMUA2026-01/README.md)、[规则中心](https://bbs.robomaster.com/wiki/20204847/809873?source=7)。规则 PDF 摘要与逐项核对见 configs/official_rules_baseline.json 和 docs/official_rules_review.md。
