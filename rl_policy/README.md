# RMUA RL policy (train branch)

This branch packages the trained RMUA vision policy, its deployment bridge, and the
reward-tuning changes used for the next training round. It is a development artifact:
it does not change the official PWM control line or simulator/scoring settings, and the
current bridge uses VEL plus `debug/pose_gt` for diagnostics only.

## Artifacts

| Path | Purpose |
| --- | --- |
| `policy/model_11100.onnx` | deployable ONNX policy (depth + state + hx -> body velocity) |
| `policy/model_11100.pt` | PyTorch checkpoint for resume/fine-tune |
| `bridge/` | standalone ROS Noetic package: LiDAR depth + ONNX -> `/vel_body_cmd` |
| `training/` | reward patch, config snapshot, and notes for the fine-tune pipeline |
| `evaluation/results.csv` | deterministic evaluation used to select the checkpoint |

## Policy interface

- depth: `[1, 1, 20, 80]`, normalized LiDAR panorama, far pixels clipped to 1.0
- state: `[1, 14]` = body velocity (3), target body velocity (3), body z in world (3),
  path offset norm (1), path offset direction (3), margin (1)
- hidden state: `[1, 192]` (SRU/GRU-style recurrent cell)
- output: `[1, 3]` body velocity
- VelCmd mapping used by the bridge: `vx=act[0]`, `vy=act[1]`, `vz=-act[2]`
  (VelCmd z is up-positive), yawRate in deg/s from path heading

## Deterministic evaluation

`1024` parallel environments, `600` steps, mean action, survival = reached the
150-step episode limit without collision/out-of-bound. Full rows are in
`evaluation/results.csv`.

| checkpoint | obstacle 2 | obstacle 3 | obstacle 4 |
| --- | --- | --- | --- |
| `model_11100` | 69.2% | 52.7% | 37.7% |
| `model_14999` (final) | 63.1% | 46.6% | 32.3% |

`model_11100` is better than the final checkpoint and is the one exported here.
Obstacle levels 3/4 were never trained by the curriculum; those columns are
zero-shot results.

## Provenance

- source: `https://github.com/HITSZcjh/rmua_rl.git`, commit `9ac9657334`
- training run: `2026-11-11_01-04-46`, PPO, 15000 iterations, batch 1024
- base model: `RL/checkpoint3699.pth` (in the source repo)
- reward changes for the next round: `training/patches/rl_offset_lateral_reward.patch`
- hashes and exact shapes: `metadata.json`

A continuous fine-tune/evaluation loop is running outside this repo. If it produces a
better global checkpoint, a later commit can replace the policy artifacts; this branch
currently pins `model_11100`.

## Development smoke test (diagnostic)

Two short runs in the official UE4 simulator used the bridge with `debug/pose_gt` and VEL:

- `path_point/1-3_center.yaml`, target 6 m/s: flew about 700 m at 8-9 m/s with sub-meter
  path error, then stopped at the map ceiling because that path is higher than the
  official route.
- `route_1_3_ned.yaml`, target 8 m/s: passed the start, flew about 200 m at 2-4 m
  altitude, then stopped on a track obstacle.

These are diagnostic smoke tests, not a competition or acceptance result.

## Usage

Build and run the bridge:

```bash
export RL_POLICY_ROOT=<repo>/rl_policy
source /opt/ros/noetic/setup.bash
source <official_rmua_ws>/devel/setup.bash
cd <repo>/rl_policy/bridge
catkin_init_workspace src
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch rl_policy_bridge rl_bridge.launch target_speed:=8.0
```

Start the official simulator first. The bridge subscribes to LiDAR and pose and publishes
`/airsim_node/drone_1/vel_body_cmd`.

## Known limitations

- Trained in the simplified `quadsim_cuda` environment; the official simulator has a much
  more aggressive velocity loop and a different track geometry. Expect a performance drop.
- Depth is a single-scan LiDAR projection; no motion compensation yet.
- The deployment bridge uses `debug/pose_gt` for state and is not a competition-legal
  localization path.
- The bridge contains a fixed-altitude/altitude-guard path for early testing; that is not
  part of the learned policy.
- VEL is used here because the policy outputs body velocity. The official main line stays
  on PWM; this branch is a training/diagnostic artifact.
- The policy's curriculum never advanced past obstacle level 2, so obstacle 3/4 results are
  zero-shot.
