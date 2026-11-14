import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch

ROOT = Path(os.environ.get('RL_TRAIN_ROOT', '')).expanduser().resolve()
if not (ROOT / 'RL').is_dir():
    raise SystemExit('RL_TRAIN_ROOT must point to the ruma_rl checkout')
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'RL' / 'rsl_rl'))

from Env.utils import load_yaml, make_env, select_device
from RL.rl_env_wrapper import RLQuadEnv

METRICS = [
    'err/pos_xy_err', 'err/pos_z_err', 'err/vel_err_l2', 'err/min_clearance',
    'err/future_min_clearance', 'stats/collide_rate', 'stats/out_of_bound_rate',
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--name', default='')
    parser.add_argument('--batch', type=int, default=64)
    parser.add_argument('--steps', type=int, default=600)
    parser.add_argument('--speed', type=int, default=3)
    parser.add_argument('--obstacle', type=int, default=2)
    parser.add_argument('--out', default='')
    args = parser.parse_args()

    env_cfg = load_yaml(ROOT / 'RL/configs/config_env.yaml')
    env_cfg['env']['batch_size'] = args.batch
    train_cfg = load_yaml(ROOT / 'RL/configs/config_rl.yaml')
    device = select_device(env_cfg.get('runtime'))

    env_cuda = make_env(env_cfg, device=device)
    env = RLQuadEnv({**train_cfg, 'env': env_cfg['env']}, env_cuda=env_cuda)
    env.set_curriculum_levels(
        speed_level=args.speed,
        obstacle_level=args.obstacle,
        freeze_curriculum=True,
    )

    session = ort.InferenceSession(args.model, providers=['CPUExecutionProvider'])
    inputs = {i.name: i for i in session.get_inputs()}
    names = list(inputs)
    static_batch_one = isinstance(inputs[names[0]].shape[0], int)

    obs = env.get_observations()
    hx = np.zeros((args.batch, 192), dtype=np.float32)
    successes = failures = 0
    acc = {k: 0.0 for k in METRICS}
    infer_time = 0.0
    batch = args.batch

    with torch.no_grad():
        for _ in range(args.steps):
            depth = obs['depth'].detach().cpu().numpy().astype(np.float32).reshape(batch, 1, 20, 80)
            state = obs['state'].detach().cpu().numpy().astype(np.float32)
            t0 = time.perf_counter()
            if static_batch_one:
                act = np.zeros((batch, 3), dtype=np.float32)
                next_hx = np.zeros_like(hx)
                for i in range(batch):
                    a, nh = session.run(
                        None,
                        {'depth': depth[i:i + 1], 'state': state[i:i + 1], 'hx': hx[i:i + 1]},
                    )
                    act[i] = a[0]
                    next_hx[i] = nh[0]
            else:
                act, next_hx = session.run(None, {'depth': depth, 'state': state, 'hx': hx})
            infer_time += time.perf_counter() - t0
            hx = next_hx.astype(np.float32)

            obs, _, dones, extras = env.step(torch.from_numpy(act).to(device))
            timeouts = extras['time_outs'].bool()
            early = dones.bool() & ~timeouts
            successes += int(timeouts.sum())
            failures += int(early.sum())
            for k in METRICS:
                if k in extras['log']:
                    acc[k] += float(extras['log'][k].mean())

    episodes = successes + failures
    row = {
        'model': args.name or Path(args.model).name,
        'onnx': str(Path(args.model)),
        'batch': batch,
        'steps': args.steps,
        'speed_level': args.speed,
        'obstacle_level': args.obstacle,
        'episodes': episodes,
        'survival': successes / max(episodes, 1),
        'early_fail': failures / max(episodes, 1),
        'infer_ms_per_step': 1000.0 * infer_time / args.steps,
    }
    for k in METRICS:
        row[k] = acc[k] / args.steps
    print(json.dumps(row), flush=True)
    if args.out:
        with open(args.out, 'a') as f:
            f.write(json.dumps(row) + '\n')


if __name__ == '__main__':
    main()
