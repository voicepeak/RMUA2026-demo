> 历史归档，已由[当前交接文档](../../00_current_handoff.md)取代。本文中的“当前”、运行状态、测试数、参数和建议均属于记录当时；不能据此启动比赛或认定现版本已验收。旧代码路径及命令以当时工作目录为准。

# 工作包 1：运行归档与独立几何评估

对应 `16_full_course_audit.md` 的实施次序 1。本次交付记录与离线验收基础设施，尚未证明全程通过；地图/任务状态解耦、未知区监督、三维参考与前方制动包络仍在后续工作包。

## 运行归档

宿主机只准备归档（不启动控制器）：

```bash
cd /home/tianbot/RUMA-by-helinjun/repo
python3 tools/run_experiment.py --scene RMUA2026 --seed 123 \
  --model yolo/weights/best.pt \
  --simulator-config ../simulator/simulator_12.0.0.5/settings.json
```

每次创建独立 `../runs/<UTC时间_UUID>/`，保存源码、工具、模型和配置副本及 SHA-256、Git 提交和工作区状态（容器无 Git 时明确记为不可用，以逐文件哈希为准）、启动参数及源码/运行副本差异。`--launch-arg name:=value` 支持参数覆盖；覆盖的路线/门/引导/能力文件也会归档。

实际运行在已 source 的 ROS 环境中加 `--execute`。现有 Docker 的快捷入口：

```bash
bash tools/run_experiment.sh --scene RMUA2026 --seed 123 \
  --model /workspace/repo/yolo/weights/best.pt \
  --simulator-config /workspace/simulator/simulator_12.0.0.5/settings.json \
  --timeout 600 --execute
```

默认以 3 m/s 巡航和 3 m/s 软限速下限开始记录实验；原来的 `run_route.sh` 参数不变。启动前必须确保运行源码与 `repo/ros_ws/src` 一致、模拟器已启动且当前没有其他速度发布者。工具检查 ROS 实际解析的包路径，并等待 rosbag 订阅位姿成功后才启动控制器；预检失败会写入 `RUNNER_ERROR`，不接管已有控制器。

`--scene/--seed` 是操作者声明，工具**不启动或重置模拟器，也未验证实际 seed**。换场景不能直接沿用原静态门图。模型参数仅用于归档，不负责启动视觉节点；使用视觉时应先按现有入口启动视觉并显式设置 `--launch-arg use_gate_map:=true`。

运行目录包含：

| 文件 | 内容 |
| --- | --- |
| `manifest.json` / `inputs/` | 来源、哈希、输入快照、命令、状态和终止原因 |
| `topics_before.json` | 开始时可发现的话题及类型 |
| `params_before.yaml` / `params_running.yaml` | 启动前参数、控制器加载后的全局 ROS 参数 |
| `telemetry.bag` | `rosbag record -a` 全部话题：图像/标定、pose、IMU、地图、命令、日志、存在的官方话题 |
| `controller.log` / `recorder.log` | 控制进程与录制进程输出 |
| `poses.csv` | header 时间、XYZ、四元数、bag 时间、frame_id |
| `controller_events.jsonl` | 控制器 PASS/MISS/SKIP、结束原因，明确 `source=controller` |
| `controller_telemetry.jsonl` | 每个控制周期的高度参考、前馈/反馈、限速项、速度和 yaw |
| `summary.json` | 话题计数、控制器分类事件、独立评估；官方结果默认为 UNKNOWN |

所有传感器保留原始 header 与 bag 时间；记录本身不等于已经完成时间对齐或消除丢帧。全话题录制可能占用较多磁盘，需用 `recorder.log` 检查写盘/缓存告警。检测到录制进程退出会停止控制。结束时先停止控制、再关闭 rosbag，尽量保留最后命令；强制杀进程、断电或磁盘写满仍可能留下 `.bag.active`，不能作为完整运行记录。

控制器新增 `/rmua/controller/events` 和 `/rmua/controller/telemetry`（`std_msgs/String` JSON）。控制器判门仍使用原有 route.s 方法；新增的事件不升级它的可信度。`ROUTE_END`、`END_GATE_LIMIT`、`ROS_SHUTDOWN` 是控制器原因，`WALL_TIMEOUT`、`OPERATOR_INTERRUPT`、`CONTROLLER_EXIT`、`RUNNER_ERROR` 是运行器原因，均不等于比赛成功。正常停机增加零速度命令；没有硬件层送达确认。

结束后自动导出 CSV/JSON。也可在 ROS 容器内手动运行：

```bash
python3 /workspace/repo/tools/export_run.py /workspace/runs/<run_id>
```

当前运行时发现 pose、图像、IMU、end_goal 等话题，未发现已经确认语义的官方成绩接口。原始话题会被录入 bag；官方计分适配仍待确认，不能把 end_goal 坐标当作完成证明。

## 独立门洞几何

`tools/evaluate_run.py` 只读取独立门数据和 pose，不读取规划路线或控制器 PASS。输入 JSON 示例（**合成数据，仅说明格式，不是赛道真值**）：

```json
{
  "frame": "NED",
  "complete": true,
  "expected_gate_count": 1,
  "gates": [{
    "uid": "synthetic:gate0",
    "verified": true,
    "verification": "synthetic test fixture; replace with calibration evidence",
    "center": [0, 0, -3],
    "normal": [1, 0, 0],
    "axis_u": [0, 1, 0],
    "axis_v": [0, 0, 1],
    "half_width": 1.5,
    "half_height": 1.5
  }]
}
```

列表顺序就是预期门序。中心为原始测量门洞中心，三个轴须为单位正交向量，normal 指向正向飞行侧，孔径单位为米。`verification` 应指向校准/测量证据，`verified` 不能仅因 tracker 稳定而置真。工具检查字段与几何一致性，不验证操作者所填证据的真实性。总门数未知时使用 `expected_gate_count: null, complete: false`；未核验的门只需 UID 与 `verified: false`，输出 UNKNOWN。

```bash
python3 tools/evaluate_run.py --gates /path/to/verified_gates.json \
  --poses ../runs/<run_id>/poses.csv --output ../runs/<run_id>/geometry.json \
  --margin 0.25 --max-gap 0.25 --max-speed 30
```

相邻 pose 从平面负侧到非负侧时计算线段交点，投影到门内 u/v 轴。孔径两轴各扣除机体/估计余量，边界接触算 MISS。上述余量、最大采样间隔和最大物理速度是起始配置，需按机体和实测误差校准；实际姿态相关机体包络尚未建模。一个采样段内多门按交点时间排序；重复穿同一门只记第一次，反向不计正向 PASS。

时间倒退/重复、超时采样、超过速度阈值的跳变禁止跨段插值，并令该段运行的全程结论为 UNKNOWN（reset 应开启新 run）。未见正向穿越的门保持 UNKNOWN。只有完整、非空、经过核验的门序都按顺序通过，且轨迹连续时，才给出 `geometric_verdict=PASS`；它仍不能证明终点成绩、无碰撞或未越界，`race_success` 保持 UNKNOWN。单次重试如何计分需按已确认的比赛规则进一步适配。

## 验证

```bash
python3 -m unittest discover -s tools/tests -v
# 在已 source 的 ROS Noetic 环境中：
python3 tools/tests/smoke_recording.py
```

离线测试覆盖真实中心偏离路线、倾斜坐标轴、孔径余量、逆向/重复穿越、多门顺序、缺失证据、时间 reset、掉帧与瞬移。ROS 集成测试启动私有 master，发布合成位姿，验证真实控制器遥测、bag 收尾、参数快照和 CSV/JSON 导出，不连接已有仿真 master，也不代表飞行验收。

2026-09-29 实施验证：25 项离线测试通过（含 `online_gate_cache` 与 `gate_tracker` 回归）；
隔离 ROS 集成测试通过（包含结束事件入包、最后命令归零）；`rmua_ws` 的 `catkin_make` 通过。
运行源码与 `repo/ros_ws/src` 已同步，源码内容无差异（运行工作空间额外的 catkin 顶层 `CMakeLists.txt` 除外）。

本机合成测试证据保存在 `../../runs/synthetic_validation_20260929T024425Z/`，含 `summary.json`、`manifest.json`、bag 和参数快照；其 scene 明确标记为 `SYNTHETIC_TEST_ONLY`，不属于实际赛道飞行结果。

## 工作包 3 进展：在线门缓存（2026-09-29）

`online_gate_cache.py`（route_follower）把短期 tracker 观测沉淀为可跨 ID/丢检的稳定门锚点：
只接收 hard、支持度/σ 达标、未被静态门覆盖、且在当前位置前方的观测；已完成门（`completed_gate_ids`）
不再更新或复用；`gate_map_cb` 重建链后跳过已完成门而不是整体重置。控制器新增
`MAP_HORIZON` 限速：`v_visibility = sqrt(2*1.5*remaining)`，在可信高度图末端前保守减速/停车。
配套视觉修复：`gate_stereo` 只取门框外带视差（排除门洞内远门）、`gate_tracker` σ 改用最近 12 帧窗口、
关联加世界距离硬门限、hard 锚点不老化、`gate_yolo_node` 丢弃无时间对齐位姿的帧并按图像时间裁剪、
`stereo_keypoint_matcher` 改为一对一贪心匹配。

两轮实测（seed 123，offscreen，`use_gate_map:=true gate_map_min_support:=2 sigma_z_max:=0.6
cruise_speed:=5 normal_speed_floor:=2`）：

| 运行 | 通过门（静态+在线） | 最远 s | 终止原因 |
| --- | --- | --- | --- |
| `frames/fix2` | 10 + 7 | 378.5 m | `MAP_HORIZON`（可信锚点耗尽，停车等待） |
| `frames/verify`（重启模拟器后复现） | 10 + 12 | 419.1 m | `MAP_HORIZON` |

`frames/verify/anchors.json` 导出该次运行末帧 hard 锚点（18 个，含 x/y/z/s/support/σ，
`verified:false`，仅为运行观测数据，不能当核验门图）。`frames/verify_sheet.jpg` 为检测画面拼图。

### 追加修复（2026-09-29 晚，多轮 v2–v11 实测）

针对"卡死""过门减速""过门拔高""提速即撞"四类问题的落地修改：

1. **软视野 + 搜扫**：`MAP_HORIZON` 现在用最近 1 s 内所有有效 track（含 soft）延伸视野；
   完全无目标时进入 `RECON`（`recon_speed=2.0`、`recon_climb=0.6`、最多 30 m），
   慢速前进+爬升重捕获门，替代原来的永久悬停死锁。
2. **无门区高度外推 + soft 高度引导**：最后一段门间坡度限幅外推 `z_extrap_m=50`；
   soft track（support≥8、σz≤1.0、conf≥0.45）作为高度趋势引导（每段坡度≤0.5、最多 60 m），
   避免 hard 锚点断档时剖面被压平在坡顶撞地。
3. **Z 掉队硬限速**：`e_z > z_lag_slow(1.5 m)` 时速度压到 `5*1.5/e_z`，先爬升再前进。
4. **卡死检测**：指令 >0.4 m/s 但实际 <0.3 m/s 持续 2.5 s → `TERMINATION(STUCK)` 并停桨指令归零。
5. **复位语义**：检测 `s` 回退 >20 m 视为 reset，清空 `completed_gate_ids`/在线缓存并重建（可在途中复位重飞）。
6. **在线缓存**：增加 tracker 身份匹配（`by_track_id`）与静态门 8 m 去重，减少同门重复锚点。

多轮实测（seed 123，offscreen，`cruise_speed` 变化，其余同）：

| 巡航 | 通过门（静态+在线） | 最远 s | 结果 |
| --- | --- | --- | --- |
| 5 m/s | 18–22 | 416–433 m | 多数轮次贴剖面良好（误差<1 m），末端窄段偶发撞结构后 `STUCK` 安全停车 |
| 6 m/s | 11–18 | 281–396 m | 部分轮次在 s≈280/396 窄段撞结构 |
| 7 m/s | 11 | 257 m | 锚点断档，剖面偏低撞坡 |
| 8 m/s | 17–18 | 396–417 m | 后段撞结构 |
| 10 m/s | 15 | 348 m | 后段撞结构 |

结论：控制器安全性与航程已提升（不再永久卡死、可越过失速点外推飞行），但**可靠速度上限仍约 5 m/s**；
再提速的主要瓶颈是感知链路（右目相机在渲染负载下只有约 2 Hz、坡顶常出现左右目单侧漏检导致双目无法配对、
hard 锚点形成滞后），以及窄段高速通过余量不足。下一步应做感知侧改进（相机帧率、单目/单帧兜底、
锚点延迟压缩），而不是继续在控制端放参数。

已知限制：`/airsim_node/reset` 在飞行中途调用返回 `success:false`（reset 语义未确认，本轮以重启
模拟器取得干净基线）；`any_report`/`meter_report`/`trigger_port` 属于工厂巡检服务，仍未发现已确认
语义的官方比赛成绩接口，`official_result`/`race_success` 保持 `UNKNOWN`。

下一步：确认 reset 语义与官方成绩接口；用独立测量核验完整门序（含 s≈20 起始段在线门是否为真门）；
长航时锚点漂移检查；感知链路提速（相机帧率/单侧检测兜底）；再推进未知区监督与全程回归。
