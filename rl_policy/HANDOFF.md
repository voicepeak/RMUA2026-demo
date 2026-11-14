# Handoff: high-compute RL continuation

Branch: `train` on `voicepeak/RMUA2026-demo`.
The 24-hour laptop loop is stopped. The deployed policy remains `model_11100`;
the best strong-continuation stress test is kept as `model_15550.pt` but is not
better on survival.

## What is in this branch

| Path | Contents |
| --- | --- |
| `policy/model_11100.pt`, `.onnx` | best survival policy from the 15k RL run |
| `policy/model_15550.pt` | best strong-penalty continuation (61.8% obstacle 2) |
| `bridge/` | standalone ROS Noetic VEL bridge + ONNX Runtime + path points |
| `pipeline/` | portable training/evaluation loop, supervisor, ONNX eval |
| `training/` | code patch and reward/config presets |
| `evaluation/` | 24h results, cycle CSVs, strong-continuation CSV, global best |
| `README.md`, `metadata.json` | policy card, interface, hashes, limitations |

## Key result

- `model_11100`: **69.2%** survival at 1024 envs, path error 0.699m. A later
  re-evaluation measured 70.3%; it is the same weights.
- Strong continuation 14100 -> 16099: best `15550` at **61.8%**, path error
  0.431m. It briefly improved from 14000 but plateaued and fell; it did not beat
  11100.
- Moderate fine-tunes: survival 56-66% with path error 0.47-0.56m. They trade
  safety margin for path precision.
- Source `v12.onnx`: **76.6%** in the 64-env protocol (282 episodes); its
  1024-env number was not measured. It is the strongest reference.

Full tables: `evaluation/24h_results.md`, `evaluation/cycle0.csv`,
`evaluation/cycle1.csv`, `evaluation/strong_resume.csv`.

## Why the gap to the source repo

1. **Effective batch is 4x smaller.** The source config uses batch 4096; the
   laptop used 1024 because 3072/4096 OOM on a 6 GB GPU. For the same 15000
   iterations that is ~307M vs ~1.2B samples.
2. **The source `v12.onnx` has no training metadata.** The repo has no training
   log or iteration; `checkpoint3699.pth` is a weights-only base model. We cannot
   assume v12 was produced by the checked-in config.
3. **The curriculum stayed at obstacle 2.** Stochastic success EMA hovered around
   0.62, below the 0.7 advance threshold, so the run never trained obstacle 3/4.
4. **Evaluation protocols differ.** The 76.6% source number is 64 envs / ~280
   episodes; our 69-70% is 1024 envs / ~5000 episodes. Run both at 1024 before
   comparing.
5. **One seed.** RL has substantial seed-to-seed variance.

## Recommended high-compute plan

### 1. Environment

```bash
git clone https://github.com/HITSZcjh/rmua_rl.git
cd ruma_rl
git checkout 9ac9657
# install CUDA/PyTorch/Env/rsl_rl as in the source README
```

For the source-original baseline, use the unmodified source and
`training/configs/config_env_highcompute.yaml` (batch 4096). For the new reward
code, apply the patch:

```bash
git apply <repo>/rl_policy/training/patches/rl_training_code.patch
```

The patch does not touch configs. Keep the original `RL/configs/config_env.yaml`
at batch 4096 on the high-compute machine.

### 2. Confirm the v12 baseline

```bash
export RL_TRAIN_ROOT=<ruma_rl>
export PYTHONPATH=
python <repo>/rl_policy/pipeline/eval_onnx.py \
  --model <ruma_rl>/deployments/rmua_ws/src/basic/src/v12.onnx \
  --name source_v12 --batch 64 --steps 600 --obstacle 2
```

Run the same at 1024 envs if memory permits. Do the same for
`rl_policy/policy/model_11100.onnx` for an apples-to-apples comparison.

### 3. Reproduce/retrain at the original scale

- Use batch 4096 and at least 15000 iterations, multiple seeds.
- Compare the unmodified source reward with the moderate reward preset.
- Use `training/configs/config_rl_moderate.yaml` and `config_rl_strong.yaml` as
  starting points; set `train.resume` and the log root.
- Keep deterministic checkpoint selection; do not deploy the newest checkpoint
  automatically. `pipeline/loop_supervisor.py` implements the global-best gate.

### 4. Distill v12 into the trainable policy

This is the most promising path to exceed 76.6%:

1. Use the training Env as the observation source.
2. Feed depth/state to `v12.onnx`; store teacher actions, observations, and
   resets. A 64-128 env batch is enough; the static-batch v12 requires one ONNX
   call per environment.
3. Behavior-clone our `CustomVisionActorCritic` from `model_11100` toward the
   teacher actions (BC/DAgger).
4. PPO fine-tune with the moderate reward (`p_center=0.8`, `lat_v=0.2`,
   `clearance=1.0`) for 2-3k iterations.
5. Evaluate at 1024 envs, export ONNX, test in the official simulator, then push
   a new commit on this branch only if survival improves.

### 5. Validate in the official simulator

Use `bridge/README.md`. The bridge uses VEL and `debug/pose_gt` for development;
it is not a competition-legal localization path. Test the source v12, the
distilled policy, and `model_11100` with the same path/target speed.

## Running the pipeline

See `pipeline/README.md`. The important environment variables are:

```bash
export RL_TRAIN_ROOT=<ruma_rl>
export PIPELINE_INIT=<repo>/rl_policy/policy/model_11100.pt
# optional: RL_CONDA_SH, RL_CONDA_ENV, RL_PYTHON, RL_POWER_GUARD, RL_BRIDGE_ONNX
```

A single cycle:

```bash
PIPELINE_VARIANT=0 python rl_policy/pipeline/pipeline.py
```

Continuous loop:

```bash
python rl_policy/pipeline/loop_supervisor.py
```

Stop:

```bash
bash rl_policy/pipeline/stop_all.sh
```

## Gotchas

- `RL_TRAIN_ROOT` is required. `pipeline.py` writes
  `$RL_TRAIN_ROOT/RL/configs/config_rl.yaml`; it reads
  `pipeline/config_rl_base.yaml` as the base.
- `PIPELINE_INIT` is required for the first cycle if no
  `global_best_model.pt` exists. An absolute path works; `learn.py` discards the
  project prefix for absolute paths.
- `eval_sweep.py` and `export_onnx.py` read `RL_TRAIN_ROOT`.
- ONNX evaluation can be CPU-only; the Env itself still needs CUDA.
- `stop_all.sh` writes a persistent `STOP`; remove it before starting a new run.
- The supervisor only starts a cycle from a global best that exists; otherwise
  it uses `PIPELINE_INIT`.
- The old strong-only reward path is not worth repeating at laptop batch size:
  it plateaued at 61.8% and lost safety margin.
- The source v12 is public but is not copied into this branch; fetch it from
  `HITSZcjh/rmua_rl`.
