# 旧实验公开数据发布

用户指定公开可见；凭据认证确认实际 Hugging Face 用户名为 `Vociepeak`（此前口述 Voicepeak）。仓库为 `Vociepeak/rmua2026-legacy-data`。
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
./tools/publish_legacy_data.sh upload --repo-id Vociepeak/rmua2026-legacy-data
```

选择实验、图像、运行记录和历史验证附件中的观测及元数据，跳过 Git、模型、驱动/安装包、编译缓存、代码副本、软链接及命中凭据模式的文本。具体排除文件只写入本机 exclusions.local.json。自动扫描不替代针对数据来源的判断。

按约 1 GiB 生成可复现 tar，保留观测相对路径；每个分片有带逐文件 SHA256 的压缩索引。一次仅暂存一个分片，上传并对照远端大小与 SHA256 后删除临时副本。用户现已授权逐分片清理：每个分片上传并复核通过后，由独立进程删除该分片已备份的本地原始文件，不等全部上传结束。失败后重新执行 upload 可恢复；已上传分片会直接复核远端，无须重新读取已删除的本地原始文件。

远端仓库公开创建。若同名仓库已经是私有，脚本退出，不擅自改变已有仓库可见性。远端状态在所有计划分片完成校验后才改为 COMPLETE。仓库在 UPLOADING 状态时只包含部分数据，不应当作完整备份。

依赖官方 huggingface_hub API；不在命令参数、日志或代码中接收/保存令牌。非 LFS 小文件会读取远端内容校验；LFS/Xet 文件对照服务端 SHA256 和大小。独立测试验证排除规则、源文件变更拒绝、不可覆盖计划、tar 及索引确定性和逐文件校验。

2026-10-19 准备结果：491,550 个文件，90,138,699,132 字节（约 84 GiB），86 个计划分片。扫描未命中已定义的凭据模式。当前未检测到登录凭据，因此尚未创建或上传远端仓库。

凭据认证已完成，公开仓库已创建并开始上传。认证凭据仅在上传进程内存中使用，不保存到 Git、计划清单或日志。上传进度与完整性以远端 publication_status.json 和 shards.json 为准。

## 后台上传与状态

`evaluation.dataset_capture.upload_service` 脱离交互终端运行，使用互斥锁防止重复启动，网络异常时重试并验证此前完成的分片。当前服务的 PID、状态及错误写入外部归档目录中的 `upload_service_status.local.json`，不进入公开数据仓库。日志文件权限 0600，并对凭据进行过滤。

```bash
cd /home/tianbot/RUMA-by-helinjun/main
./tools/data_upload_status.sh
```

第一个真实分片已在远端验证：data-00000.tar，1,058,334,720 字节，SHA256 `807a7ed76e93822145ccf22b070af9d07941a7ed8f52592550b071f7b227c1e8`。当前发布状态为 UPLOADING；完成标志只由所有分片校验成功后自动生成。

服务不保存认证凭据；机器重启或进程结束后恢复上传需要重新提供凭据。公开链接：https://huggingface.co/datasets/Vociepeak/rmua2026-legacy-data 。

## 逐分片清理（用户已授权）

用户最新要求“上传一个删一个”，替代此前全部上传后才清理的策略。独立的 cleanup_archive 后台进程使用 `--per-shard` 模式，不重启上传进程，也不需要认证凭据。

每发现一个已上传分片，清理进程固定远端 commit，核对该分片的公开回执、tar 服务端 SHA256/大小及逐文件索引。然后计算该分片对应本地文件的 SHA256；只删除与备份一致、且在删除前未再次变化的成员，并清理这些文件留下的空目录。全局 UPLOADING 状态下也可以清理已核实的分片，未上传的分片不受影响。

范围只包含已审核上传计划中的历史归档成员，约 83.95 GiB。新文件、内容有变化的文件、未上传的文件、Git 历史、模型、模拟器及环境不在清理范围。某分片缺少远端回执或校验不一致时，不删除该分片的本地文件。已修正历史 Docker 实验目录的所有者，以便处理已备份的 root 创建目录；不修改原始文件内容和 mtime。

清理状态、逐片回执与保留文件清单写入外部数据目录中的 local_cleanup_status.local.json、per_shard_cleanup_state.local.json、cleanup_preserved.local.jsonl。全量上传计划和上传回执继续保留，供恢复与校验使用。使用 `./tools/data_upload_status.sh` 查看上传和清理进度。机器重启会终止当前后台进程；可用独立上传环境重新执行 `python -m evaluation.dataset_capture.cleanup_archive --authorize-cleanup --per-shard` 恢复。
