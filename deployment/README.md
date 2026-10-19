# 独立部署开发镜像

先执行 `bash tools/build_image.sh`。默认镜像 `rmua-next:p1-development` 固定本机已有的官方 OSRF Noetic 基础镜像 digest，构建过程关闭网络，不复制旧工程、模拟器、模型、数据、日志或密钥。独立构建官方 PWM/VEL 消息，不依赖宿主 beta 工作区。默认入口自动运行白名单只读状态观察，无 GUI，也不设置 ROS_IP / ROS_MASTER_URI。

该镜像是工程基线，**不具备比赛 30s 离开起点及完整任务能力，不能当作参赛提交物**。competition.yaml 仍关闭控制；default observer 不主动结束运行。后续须在相同独立环境中完成飞行/任务、启动时限与正式输入权限验收，再更新比赛入口。

离线导入验证：

```bash
docker run --rm --network none rmua-next:p1-development python3 -c 'import rospy,numpy,cv2; from airsim_ros.msg import RotorPWM,VelCmd; from estimation.imu_gps import ImuGpsEstimator; print("standalone imports OK")'
docker image inspect rmua-next:p1-development --format '{{.Size}}'
```

外部 ROS master 的网络配置由运行方注入。开发实验依旧使用宿主的有界场景 harness，实验结束关闭仿真器的行为不能进入参赛镜像。

2026-10-19 本地构建通过：inspect 大小 1,163,189,792 bytes；镜像内 34 项单元测试通过，独立 ROS/NumPy/OpenCV/控制消息导入通过，network=none 的默认入口没有主动退出。证据见 docs/evidence/p0_deployment_image.json。该构建源码含未提交的部署改动，镜像 revision 带 dirty 标记；不能冒充干净发布镜像。

后续已用干净提交 16da241 构建不可变开发镜像 `rmua-next:16da241`，大小 1,163,205,947 bytes，镜像内 49 项单元测试通过；seed1017/1018 已实际完成独立镜像 PWM 实验，容器只挂载本轮输出目录，不挂载 beta 或宿主源码。本地执行示例：

```bash
OPENBLAS_NUM_THREADS=1 python3 -m evaluation.seed_runner.pwm_identification --seed 1018 --hover-seconds 10 --controller-image rmua-next:16da241 --evaluate-truth
```

宿主负责新建和关闭本地模拟器，镜像默认入口仍是未验收的只读观察器；该测试不构成比赛部署与 30s 出发验收。
