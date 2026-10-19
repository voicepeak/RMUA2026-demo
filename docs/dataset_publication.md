# 旧实验公开数据发布

用户指定 Hugging Face 账号 Voicepeak、公开可见，计划仓库 `Voicepeak/rmua2026-legacy-data`。
上传前必须通过 `whoami()` 核实准确用户名；大小写采用登录账号的规范名称。当前状态以本机上传状态和远端 publication_status.json 为准，不能用脚本存在代表已经上传。

工具环境位于 `/home/tianbot/.local/share/rmua-hf-upload/venv`，与 ROS/Noetic 环境隔离。
原始归档：`/home/tianbot/RUMA-legacy-archive-20261019`。
本机计划、排除清单、索引和临时分片：`/home/tianbot/RUMA-hf-public-20261019`，不进入 Git。

```bash
# 本机交互登录，令牌不要写入脚本或聊天记录
/home/tianbot/.local/share/rmua-hf-upload/venv/bin/hf auth login
/home/tianbot/.local/share/rmua-hf-upload/venv/bin/hf auth whoami
cd /home/tianbot/RUMA-by-helinjun/main
# 已存在 plan.json 时不重复 prepare；继续 pack 或 upload
./tools/publish_legacy_data.sh prepare
./tools/publish_legacy_data.sh pack --shard-index 0
./tools/publish_legacy_data.sh upload --repo-id Voicepeak/rmua2026-legacy-data
```

选择实验、图像、运行记录和历史验证附件中的观测及元数据，跳过 Git、模型、驱动/安装包、编译缓存、代码副本、软链接及命中凭据模式的文本。具体排除文件只写入本机 exclusions.local.json。自动扫描不替代针对数据来源的判断。

按约 1 GiB 生成可复现 tar，保留观测相对路径；每个分片有带逐文件 SHA256 的压缩索引。一次仅暂存一个分片，上传并对照远端大小与 SHA256 后删除临时副本，**不删除原始数据**。失败后重新执行 upload 可恢复。

远端仓库公开创建。若同名仓库已经是私有，脚本退出，不擅自改变已有仓库可见性。远端状态在所有计划分片完成校验后才改为 COMPLETE。仓库在 UPLOADING 状态时只包含部分数据，不应当作完整备份。

依赖官方 huggingface_hub API；不在命令参数、日志或代码中接收/保存令牌。非 LFS 小文件会读取远端内容校验；LFS/Xet 文件对照服务端 SHA256 和大小。独立测试验证排除规则、源文件变更拒绝、不可覆盖计划、tar 及索引确定性和逐文件校验。

2026-10-19 准备结果：491,550 个文件，90,138,699,132 字节（约 84 GiB），86 个计划分片。扫描未命中已定义的凭据模式。当前未检测到登录凭据，因此尚未创建或上传远端仓库。
