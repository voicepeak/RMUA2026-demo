import argparse
import json
import os
import sys
from pathlib import Path

import torch

ROOT = Path(os.environ.get('RL_TRAIN_ROOT', '')).expanduser().resolve()
if not (ROOT / 'RL').is_dir():
    raise SystemExit('RL_TRAIN_ROOT must point to the ruma_rl checkout')
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'RL' / 'rsl_rl'))

from Env.utils import load_yaml, make_env, select_device
from RL.rl_env_wrapper import RLQuadEnv
from rsl_rl.runners.on_policy_runner import OnPolicyRunner

BATCH = 1024
DEFAULT_STEPS = 600
METRICS = [
    'err/pos_xy_err', 'err/pos_z_err', 'err/vel_err_l2', 'err/min_clearance',
    'err/future_min_clearance', 'stats/collide_rate', 'stats/out_of_bound_rate',
    'reward/rew_p_center', 'reward/rew_lat_v', 'reward/rew_clearance', 'reward/rew_v', 'reward/rew_collide',
]


def evaluate(checkpoint, speed_level, obstacle_level, steps, batch=BATCH):
    env_cfg = load_yaml(ROOT / 'RL/configs/config_env.yaml')
    env_cfg['env']['batch_size'] = batch
    train_cfg = load_yaml(ROOT / 'RL/configs/config_rl.yaml')
    device = select_device(env_cfg.get('runtime'))

    env_cuda = make_env(env_cfg, device=device)
    env = RLQuadEnv({**train_cfg, 'env': env_cfg['env']}, env_cuda=env_cuda)
    env.set_curriculum_levels(
        speed_level=speed_level,
        obstacle_level=obstacle_level,
        freeze_curriculum=True,
    )
    runner = OnPolicyRunner(env, train_cfg, None, device)
    runner.load(str(checkpoint), load_optimizer=False, map_location=device)
    policy = runner.get_inference_policy(device=device)

    obs = env.get_observations()
    successes = failures = 0
    acc = {k: 0.0 for k in METRICS}
    n = 0
    with torch.no_grad():
        for _ in range(steps):
            action = policy(obs)
            obs, _, dones, extras = env.step(action)
            timeouts = extras['time_outs'].bool()
            early = dones.bool() & ~timeouts
            successes += int(timeouts.sum())
            failures += int(early.sum())
            for k in METRICS:
                if k in extras['log']:
                    acc[k] += float(extras['log'][k].mean())
            n += 1

    episodes = successes + failures
    row = {
        'checkpoint': str(checkpoint),
        'speed_level': speed_level,
        'obstacle_level': obstacle_level,
        'episodes': episodes,
        'survival': successes / max(episodes, 1),
        'early_fail': failures / max(episodes, 1),
    }
    for k in METRICS:
        row[k] = acc[k] / max(n, 1)
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoints', nargs='+', required=True)
    parser.add_argument('--speed', type=int, default=3)
    parser.add_argument('--obstacle', type=int, default=2)
    parser.add_argument('--steps', type=int, default=DEFAULT_STEPS)
    parser.add_argument('--batch', type=int, default=BATCH)
    parser.add_argument('--out', type=str, default='')
    args = parser.parse_args()

    rows = []
    for ckpt in args.checkpoints:
        row = evaluate(Path(ckpt), args.speed, args.obstacle, args.steps, args.batch)
        rows.append(row)
        print(json.dumps(row), flush=True)

    if args.out:
        with open(args.out, 'a') as f:
            for row in rows:
                f.write(json.dumps(row) + '\n')


if __name__ == '__main__':
    main()
