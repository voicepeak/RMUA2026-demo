# 工作区迁移（2026-10-19）

本机有两个实际顶层目录：

```text
RUMA-by-helinjun/
  beta/
    repo/            Git beta 分支（保留旧源码及必要权重）
    rmua_ws/         旧版实际运行与编译快照
    simulator/       旧模拟器
    reference/       旧交接资料和原始设计
    experiments/     仅保留 Docker 所需 driver_root
    datasets/        旧版训练数据
    tools/           旧标注环境
    run_*.sh         旧启动入口
  main/              Git main 分支（独立 worktree）
    simulator/       独立复制，Git 忽略
```

根目录保留 repo、rmua_ws、simulator、experiments、frames、runs、datasets、tools 及旧启动脚本的兼容软链接，供容器原有 `/workspace/...` 路径使用。它们不产生第二份数据。Git 公共元数据位于 beta/repo/.git，移动或删除 beta/repo 前必须先修复 main worktree。

容器 rmua_noetic、镜像和 GPU 驱动修复绑定保持不变。main/tools/enter_env.sh 只复用 ROS 与 airsim 消息环境，不启动旧控制器。新版运行目录固定 `/workspace/main`，移动整个工程时需同步更新入口路径。

历史实验、图像、旧 runs、演示副本及 tracked 迭代附件归档在 `/home/tianbot/RUMA-legacy-archive-20261019`，约 86 GiB。这是移出开发目录，不是释放相同大小磁盘空间。Git 完整快照标签 archive/pre-refactor-20261019 保存旧版未提交代码，另有迁移前 bundle 和 patch。

beta 当前树移除了 docs/archive、docs/validation、yolo/runs、yolo/samples；保留所需模型、源码、测试和现用说明。Git 历史不改写，旧分支不删除。模拟器/模型/原始实验不上传 main。每个远端分支根目录对应其本地 Git 工作区内容，不在远端再次嵌套 beta/main 文件夹。

beta/repo/deployment/legacy 保存旧启动脚本副本，beta/ 下是本机入口。历史脚本存在强制停止旧模拟器的行为；main 启动脚本会在检测到已有实例后退出。两个版本顺序运行。
