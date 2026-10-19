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
