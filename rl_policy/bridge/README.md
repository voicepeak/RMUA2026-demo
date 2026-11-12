# rl_policy_bridge

Standalone ROS Noetic bridge for the RMUA RL policy:

```
/airsim_node/drone_1/lidar  ->  80x20 body depth  \
/airsim_node/drone_1/debug/pose_gt -> 14-dim state  -> ONNX policy -> /airsim_node/drone_1/vel_body_cmd
```

The policy is `../policy/model_11100.onnx`. ONNX Runtime 1.16.3 headers and the shared
library are bundled under `third_party/onnxruntime`.

## Dependencies

- ROS Noetic with `airsim_ros` messages (source the official RMUA workspace)
- Eigen3, yaml-cpp, roscpp, sensor_msgs, geometry_msgs, std_msgs
- The bundled ONNX Runtime (or set `ONNXRUNTIME_ROOT`)

## Build

```bash
export RL_POLICY_ROOT=<repo>/rl_policy
source /opt/ros/noetic/setup.bash
source <official_rmua_ws>/devel/setup.bash
cd <repo>/rl_policy/bridge
catkin_init_workspace src
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

If ONNX Runtime lives elsewhere:

```bash
catkin_make -DONNXRUNTIME_ROOT=/path/to/onnxruntime
```

## Run

Start the official simulator first, then:

```bash
export RL_POLICY_ROOT=<repo>/rl_policy
roslaunch rl_policy_bridge rl_bridge.launch target_speed:=8.0
```

Use the official route path instead of the demo path:

```bash
roslaunch rl_policy_bridge rl_bridge.launch \
  path_file:=$RL_POLICY_ROOT/bridge/path_point/route_1_3_ned.yaml \
  target_speed:=8.0
```

Parameters: `target_speed`, `takeoff_height`, `hold_altitude`, `use_fixed_altitude`,
`use_path_z`, `yaw_gain`, `yaw_rate_max_deg`, `max_speed`, `va`.

## Safety / scope

- `debug/pose_gt` is development truth, not a competition localization source.
- Publishes `vel_body_cmd` (VEL). The official main line remains PWM; this is a
  diagnostic/training artifact.
- The bridge includes takeoff, altitude hold/guard, stale-input hover, yaw rate
  limiting, and velocity clamps. It has not passed competition acceptance.
- The current depth path is a single-scan LiDAR projection without motion compensation.
