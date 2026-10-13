# 暂停交接：跨种子适配续修（2026-10-13）

用户因额度耗尽明确要求“别跑了，先给交接文档”。已停止继续修改和测试，比赛控制/视觉/Runner/记录进程已退出，并停止模拟器容器rmua_noetic。不安排电脑休眠或关机。任务未完成，后续接手应继续修复，不能把以下结果称为全局适配通过。

## 版本及远端

工作区`/home/tianbot/RUMA-by-helinjun`，Git仓库`repo/`。分支`feature/seed-independent-navigation-20261013`；最近实现`de35b2b`已经推到origin同名分支，main没有合并候选。最近已提交实现：

- `5f243fa`：阴影白柱验证、最近相干双目视差分量、单侧顶帽配合完整对侧柱恢复中心；不产生完整开口几何。
- `de35b2b`：地图速度使用不超过执行检查的4m/s²保守制动能力，显式计入感知延迟和点云年龄；侦察距离预算不变。

该提交主机/ROS各596项测试通过；日志`experiments/global_adaptation_20261013/map_braking_host_tests.log`及`map_braking_ros_tests.log`。正常Git SSH有DNS故障，已成功使用单次参数经现有代理推送，不改系统网络：

```bash
git -c core.sshCommand='ssh -o ConnectTimeout=15 -o "ProxyCommand=nc -X connect -x 127.0.0.1:7897 %h %p"' push origin feature/seed-independent-navigation-20261013
```

## 最新实跑：均未通过

全部render、ClockSpeed1、LiDAR75000点/s、lidar_nav、legacy planner、coupled response，空历史地图，保留官方碰撞、边界和180秒限时。31与32代码及参数相同，仅种子不同。

| 轮次 | seed / 提交 | 最后有效结果 |
| --- | --- | --- |
| 30 | 456 / 5f243fa | 62分，179.408秒，1411.421米；180.008秒Finished，未通过第一阶段。 |
| 31 | 456 / de35b2b | 57分，179.079秒，1318.903米；180.030秒Finished，未通过。早期0–200米23.9秒，比30的27.8秒快，但600–800米反复制动耗时42.1秒，抵消收益。 |
| 32 | 789 / de35b2b | 50分，179.557秒，1155.932米；180.008秒Finished，未通过。入口通过，后段地图不足HOLD。 |

目录分别为`experiments/global_visible_20261013_seed456_30`、`...seed456_31`、`...seed789_32`。Launcher日志在`experiments/global_adaptation_20261013/seed456_30_launcher.log`、`seed456_31_launcher.log`、`seed789_32_launcher.log`。仅取最后非Finished HUD之前遥测，结束后移动/得分不作成绩。没有官方阶段切换，因此同轮视觉模型复用、车辆段、Factory inspection均未实跑验收。最新候选未重新验证123及独立留出种子。

## 未提交候选：有1项回归失败，禁止直接称为可运行版本

工作树保留修改，运行副本也已同步这些修改，但尚未飞行。代码差异另存`experiments/global_adaptation_20261013/pending_frame_support.patch`。修改涉及：

- `gate_stereo.py`、`gate_yolo_node.py`：只在检测门框区域加视差范围/边界余量计算全分辨率SGBM，保留原始坐标及视差；区域外无效。原占据双目路径保持完整。
- `gate_candidates.py`：拒绝贴近图像边缘的裁剪门框，防止把可见半框中点当物理中心。
- `gate_tracker.py`、`gate_map.py`：增加position_support，记录每帧通过两柱外观、有效两侧深度及物理尺寸筛选的中心证据；8次稳定测量可独立于YOLO分数建立位置锚点，不把geometry_valid改为true。
- `online_gate_cache.py`、`tools/race_seed.py`：接受上述独立位置支持，仍检查新鲜度、支持数及XYZ方差；对应测试保留。

当前主机和ROS均运行600项，**各失败1项**：`test_inner_detector_rectangle_can_find_surrounding_complete_posts`（`tools/tests/test_stereo_obstacles.py`约第75行）。日志`frame_support_host_tests.log`和`frame_support_ros_tests.log`。

已知原因：旧测试的内框(35,20,130,160)向外扩12%后，上界y=1；新增4像素边界规则将整个扩展候选拒绝，实际白柱仍完整位于20–180区域。尚未修复。可考察：原检测框贴边时直接拒绝；原框完整时，把有限扩展范围限制在图像4像素内，再重新验证两柱。必须避免把已被画面裁掉的框通过缩小bbox重新接受。不要删除测试或降低完整性要求来凑通过。修好后重新同步、跑相关回归，再启动新目录。

## 新发现及证据

32停住时位置[729.931,309.163,-70.833]，姿态近水平。前门位于画面下边，被截掉下部；旧中心算法返回世界[729.233,326.022,-65.447]，不能当完整门的可靠中心。对应真同步图`...seed789_32/stereo_pairs/1791898084.651860_left.png`及right/meta。`pair32_probe.log`记录bbox贴底720。轨迹122有157次重复观测、sigma_z约0.017，但置信度约0.552，未成硬锚；低方差不能证明裁剪中心正确。因此禁止简单把置信度门槛全局调低。

上坡/下坡的摄像头视野是后续要继续核查的问题。官方双目960×720、FOV60°，FX/FY831.4、基线0.30m；陡坡可能让门裁在画面边缘。尚未证明哪种方法能够可靠恢复所有路段，不修改官方相机、碰撞、计时来伪造通过。

29失败现场的完整前门，经新局部SGBM离线复核仍恢复24.096米中心，和全图结果约相差0.028米；日志`pair29_cropped.log`。这仅是单帧回归，不证明性能和飞行通过。对精确点云的离线性能复现见`profile_response.py`、`response_profile.log/json`，使用真实记录运动和模型；不能代替比赛结果。

## 恢复顺序与边界

1. 先读本交接，再读[39 续修证据](39_global_course_frame_and_bootstrap_20261013.md)与[38 原始种子依赖审查](38_seed_independence_audit_20261013.md)。
2. 检查上述未提交候选和1项失败；先修回归、验证裁剪门不会被重新接受，再评估深度提速和独立位置证据是否可靠。
3. 修改仓库`repo/ros_ws/src/`；运行包在`rmua_ws/src/`，必须同步。部分文件root所有，必要时通过Docker cp。运行中冻结代码，目录必须全新。下一轮编号33尚未使用。
4. 找到可完成第一阶段的同一冻结配置后，验证123、456、789及42/2026等未参与调参的种子；再验收1→3→5→8竞速段的官方目标变化及动态车辆。保留所有失败，不挑最高分。
5. 13迎面车漏检、legacy未知空间语义、外部桥指令失效契约、Factory inspection仍未解决/验收；全比赛不能宣称完成。

不按seed添加坐标、能力表或预录地图。已有修复统一现场建图、坐标刚体映射、缓存耗尽、末段截角及同轮切段输入；目前只是消除专用路径和部分入口问题，整体泛化仍未完成。

GPU恢复容器`rmua_noetic`已停止，后续run_sim.sh会启动。原容器`rmua_noetic_before_driver_recovery_20261012`、备份镜像、匹配595.91.07库以及所有旧记录保留；未重启电脑或换驱动。用户未跟踪草稿`docs/specs/RMUA2026_local_occupancy_avoidance_plan.md`未修改、未提交。早先用户说完成后休眠、不要关机；当前因额度暂停，不执行完成后的休眠。
