pairing.py 提供不可变字节图像和有界左右图像配对，时间由调用方的 IMU 时钟约束。每侧最多四帧，寻找当前可用的最近头时间戳配对，默认差值不超过 5ms；过期、其他 epoch、重复和乱序帧不能进入输出，每帧最多消费一次。reset 必须来自可信的新 epoch，不能用来重复接受旧数据。

adapters/ros_noetic/stereo_images.py 严格解码带行填充的 mono8/rgb8/bgr8/rgba8/bgra8，不猜测未知编码。图像源按明确订阅的话题标识；本机 Image.header.frame_id 为空，不据此推断外参或 TF。

默认图像有效期 150ms 的两场只读实测未通过持续配对。显式 200ms 实验两场完成，仅说明较长图像年龄预算下的配对链路可运行；默认保持 150ms。头时间戳相同不证明曝光同步，不能把配对结果直接当作可信深度或飞行净空。

sgbm.py 新增离线诊断像素视差，使用两方向 SGBM、双线性反向一致性与双侧纹理检查；无效像素保留 NaN/UNKNOWN，不生成米制深度或 FREE。原输入有效期不因计算完成而延长。输出显式标记 rectification_verified=false、metric_depth_verified=false、acceptance_eligible=false。StereoPair 契约阻止跨 epoch 或人为延长源图像有效期。

两份真实输入的严格有效覆盖约 8.9%，单线程离线中位耗时约 275/328ms，不能支持当前在线深度链路。已跑通 SGBM 诊断基线，不是矫正、真实深度、学习模型、VIO 或运动补偿验收；见 docs/stereo_sgbm_baseline.md。

运行和失败证据见 docs/stereo_pairing_observation.md，整体验收状态见 docs/implementation_status.md。
