# 旧版冻结与目录迁移（2026-10-19）

本工作区：`beta/repo`，分支 `beta`。新实现：兄弟目录 `../../main`，分支 `main`。

旧源码及全部未提交修改已冻结于 `archive/pre-refactor-20261019`（316acf5）。
清理只影响当前检出目录，不改写 Git 历史。旧的 docs/archive、docs/validation、yolo/runs、yolo/samples 可从此标签恢复。

本机历史实验、帧、旧 runs、演示副本、迭代附件归档到 `/home/tianbot/RUMA-legacy-archive-20261019`。
原始数据仍占磁盘；本次移出开发目录，没有把可能有价值的实验数据永久删除。
保留模型、数据集、旧版 rmua_ws 及安装环境。`beta/experiments` 仅保留现有 Docker 容器需要的 driver_root。

根目录的 repo/rmua_ws/simulator/experiments 等软链接兼容旧的 `/workspace/...` 容器路径，不能随意删除。
旧版启动入口为 `../run_sim.sh`、`../run_route.sh`、`../run_yolo.sh`。同时仅运行一个版本的模拟器和控制器。

注意：repo/ros_ws/src 是编辑源，../rmua_ws/src 是迁移前实际运行快照，两者原本存在差异。
本次保留差异供复现，不自动把未验收编辑覆盖进运行包；这不代表旧版已通过新设计的比赛验收。
