# Training changes: offset and lateral-velocity penalties

The previous policy under-penalized path offset and lateral oscillation:

- `rew_p_center` was multiplied by zero in `RL/rl_env_wrapper.py`, so small offsets had no
  penalty; `p_bound` only penalized offsets beyond 2.5 m.
- no term penalized body lateral velocity, so the policy could wobble across the path.

The patch in `patches/rl_offset_lateral_reward.patch` changes that:

- `rew_p_center = -p_center * (dp_xy^2 + 0.25 * dp_z^2)`, enabled and quadratic
- `rew_lat_v = -lat_v * v_body_y^2`
- smoother defaults: `d_acc=0.05`, `d_jerk=0.006`
- `DiffPhys/learn.py` gets the same quadratic center term and a lateral-velocity loss
- `RL/learn.py` supports `train.load_optimizer` for clean fine-tuning
- `RL/rl_env_wrapper.py` supports `curriculum.start_speed_level` and
  `curriculum.start_obstacle_level` so fine-tuning can start at speed 3 / obstacle 2

Apply to the source repo:

```bash
cd <ruma_rl>
git apply <repo>/rl_policy/training/patches/rl_offset_lateral_reward.patch
```

`configs/config_rl_reward.yaml` is a snapshot of the reward/fine-tune configuration.
It still contains source-repo run paths (`train.resume`, `eval.checkpoint`); adjust them
to the local checkout.

The recommended fine-tune is from `policy/model_11100.pt` with:

- two phases: moderate offset first, then stronger offset/lateral penalties
- start at speed level 3 / obstacle level 2, curriculum enabled
- deterministic checkpoint selection on survival and `err/pos_xy_err`
- separate evaluation at obstacle 3 for zero-shot robustness

The continuous loop that produced the `global_best` artifacts runs outside this repo and
is not included here; this directory contains the reproducible reward/config changes and
the policy pinned on this branch.
