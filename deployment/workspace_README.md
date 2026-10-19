# RMUA 工作区

- `beta/repo`：旧代码，Git `beta` 分支；目录说明在 `beta/repo/docs/migration_20261019.md`。
- `main`：重构工程，Git `main` 分支；实现进度和使用方式在 `main/README.md`。
- 两个 Git 工作区共用 beta/repo/.git；在对应目录执行 Git 命令。
- 根目录旧路径是软链接，为原 Docker 挂载和旧脚本兼容保留。
- 外部历史归档：`/home/tianbot/RUMA-legacy-archive-20261019`，原始数据未永久删除，也未释放同等磁盘空间。

常用入口：

```bash
cd main
./tools/build_interfaces.sh
./tools/enter_env.sh python3 -m unittest discover -s tests/unit -v
./tools/run_sim.sh 456 offscreen
# 另一个终端
./tools/audit.sh --seed 456 --duration 10
./tools/stop_sim.sh
```

当前只读审计基线已经实现；PWM 悬停和完整比赛功能按 main/docs/implementation_status.md 继续推进。
