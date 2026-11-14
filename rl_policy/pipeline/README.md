# Training / evaluation pipeline

Portable version of the 24-hour loop. It is designed to run against a separate
`ruma_rl` checkout, not the RMUA simulator workspace.

## Requirements

- A `ruma_rl` checkout at commit `9ac9657` with `Env`/`rsl_rl` installed
- `PyYAML`, `onnxruntime` (CPU is enough for evaluation)
- The source `v12.onnx` is optional; evaluate it from the `ruma_rl` deployment
  directory or a local copy

## Environment

```bash
export RL_TRAIN_ROOT=/path/to/ruma_rl
export PIPELINE_INIT=/path/to/RMUA2026-demo/rl_policy/policy/model_11100.pt
# Optional:
# export RL_CONDA_SH=/path/to/miniconda3/etc/profile.d/conda.sh
# export RL_CONDA_ENV=py311_rmua
# export RL_PYTHON=/path/to/env/bin/python
# export RL_POWER_GUARD=/path/to/power_guard.sh
# export RL_BRIDGE_ONNX=/path/to/rmua_bridge/models/policy_global_best.onnx
# export RL_GLOBAL_HOURS=48
# export RL_PHASE_A_ITERS=3000
# export RL_PHASE_B_ITERS=1800
```

If `RL_CONDA_SH`/`RL_CONDA_ENV` are unset, the pipeline uses the interpreter that
started it and assumes the environment is already active.

## Commands

```bash
# One cycle, variant 0 (moderate reward, obstacle 2)
PIPELINE_VARIANT=0 python pipeline.py

# Continuous A/B/evaluate/select loop (variants 1, 2, 0)
python loop_supervisor.py

# Evaluate torch checkpoints (deterministic, 1024 envs x 600 steps)
python eval_sweep.py \
  --checkpoints <ckpt1> <ckpt2> \
  --speed 3 --obstacle 2 --steps 600 --batch 1024

# Evaluate ONNX policies (supports static batch=1 or dynamic batch)
python eval_onnx.py --model <policy.onnx> --name policy --batch 64 --steps 600

# Export a checkpoint to ONNX
python export_onnx.py --checkpoint <model.pt> --output <policy.onnx>

# Stop the loop
bash stop_all.sh
```

`pipeline.py` writes `$RL_TRAIN_ROOT/RL/configs/config_rl.yaml` for each phase.
Set the hardware batch in `$RL_TRAIN_ROOT/RL/configs/config_env.yaml` before
starting; for a high-compute machine use
`training/configs/config_env_highcompute.yaml` (batch 4096).

## What a cycle is

1. Choose a variant from `variant_params()`.
2. Run Phase A from `PIPELINE_INIT` (or the current global best).
3. Evaluate Phase A checkpoints at obstacle 2.
4. Run Phase B from the Phase A best (unless the variant sets `skip_b`).
5. Compare the cycle best with `global_best.json`; only a better survival result
   replaces `global_best_model.pt` / `global_best_policy.onnx`.
6. Archive `results_cycle_N.csv` and start the next variant.

`loop_supervisor.py` starts each new pipeline from the current global best, so a
bad cycle cannot overwrite a good model.

## Layout

| Path | Meaning |
| --- | --- |
| `pipeline.py` | one training + evaluation cycle |
| `loop_supervisor.py` | continuous cycle selector / global-best gate |
| `eval_sweep.py` | deterministic torch-checkpoint evaluation |
| `eval_onnx.py` | deterministic ONNX evaluation (source v12 or ours) |
| `export_onnx.py` | checkpoint -> ONNX |
| `stop_all.sh` | write `STOP` and kill pipeline/training/guard |
| `config_rl_base.yaml` | source-original RL config used as the base |
