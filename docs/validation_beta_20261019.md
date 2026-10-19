# beta 冻结验证（2026-10-19）

在原 rmua_noetic 容器的 Python 3.8 / ROS Noetic 环境运行三组与原未提交修改相关的测试：

| 测试 | 通过 | 失败 |
|---|---:|---:|
| test_gate_tracker.py | 8 | 0 |
| test_online_gate_cache.py | 17 | 0 |
| test_stereo_obstacles.py | 19 | 1 |

失败：test_inner_detector_rectangle_can_find_surrounding_complete_posts，期望 1 个候选，实际 0。
此失败已明确记录在原 docs/40_paused_global_adaptation_handoff_20261013.md 第 39 行。
冻结标签与清理后版本之间 ros_ws/ 和 tools/ 无源码差异，未为通过测试而修改旧算法。
旧版是可追溯的实现对照，不宣称通过比赛或全套回归。
