# Training changes and configs

Apply the code patch to a clean `ruma_rl` checkout at commit `9ac9657`:

```bash
cd <ruma_rl>
git apply <repo>/rl_policy/training/patches/rl_training_code.patch
```

The patch contains:

- `RL/rl_env_wrapper.py`: quadratic path-offset penalty (`p_center`), body lateral
  velocity penalty (`lat_v`), clearance-margin penalty (`clearance`), and
  `curriculum.start_speed_level` / `start_obstacle_level`
- `RL/learn.py`: `train.load_optimizer` support
- `DiffPhys/learn.py` and `DiffPhys/configs/config_diff.yaml`: matching center and
  lateral-velocity terms

The patch intentionally does **not** change `RL/configs/config_env.yaml` or
`RL/configs/config_rl.yaml`; use the configs below.

## Configs

| File | Purpose |
| --- | --- |
| `configs/config_env_highcompute.yaml` | original RL env config, batch 4096 |
| `configs/config_rl_moderate.yaml` | `p_center=0.8`, `lat_v=0.2`, `clearance=1.0`, success threshold 0.65, entropy 0.005 |
| `configs/config_rl_strong.yaml` | `p_center=1.5`, `lat_v=0.5`, no clearance, success threshold 0.7, entropy 0.01 |
| `../pipeline/config_rl_base.yaml` | source-original RL config used by the pipeline |

`config_rl_moderate.yaml` and `config_rl_strong.yaml` have `train.resume: null`.
Set it to the checkpoint to continue from, or let `pipeline.py` generate the
same values through its variants.

## What the 24h run showed

- Strong offset/lateral penalties improved path error (`0.43-0.52m` vs `0.699m`)
  but reduced survival (`56-66%` vs `69-70%` for `model_11100`).
- The strong continuation peaked at `model_15550` (61.8% obstacle 2) and did not
  beat `model_11100`.
- The curriculum stayed at obstacle level 2; no fine-tuned checkpoint reached
  the 0.7 success threshold needed to advance.
- The original training used batch 4096; the 24h run used batch 1024 because of
  the 6 GB laptop GPU, i.e. about 4x fewer samples for the same iteration count.
  A high-compute run should restore the larger batch before judging the reward
  design.

Recommended next experiments:

1. Full 4096-batch reproduction from `checkpoint3699` with the source-original
   reward and the source config, multiple seeds.
2. Evaluate the source `v12.onnx` with 1024 envs to confirm its small-batch
   76.6% number.
3. Distill `v12.onnx` into the trainable policy, then PPO fine-tune with the
   moderate reward.
4. Keep best-checkpoint selection by deterministic survival; do not deploy the
   newest checkpoint automatically.
