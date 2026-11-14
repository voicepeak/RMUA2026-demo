import argparse
import os
import sys
from pathlib import Path

ROOT = Path(os.environ.get('RL_TRAIN_ROOT', '')).expanduser().resolve()
if not (ROOT / 'RL').is_dir():
    raise SystemExit('RL_TRAIN_ROOT must point to the ruma_rl checkout')
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'RL' / 'rsl_rl'))

from Env.utils import load_yaml, make_env, select_device
from RL.rl_env_wrapper import RLQuadEnv
from rsl_rl.runners.on_policy_runner import OnPolicyRunner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    env_cfg = load_yaml(ROOT / 'RL/configs/config_env.yaml')
    env_cfg['env']['batch_size'] = 64
    train_cfg = load_yaml(ROOT / 'RL/configs/config_rl.yaml')
    device = select_device(env_cfg.get('runtime'))

    env_cuda = make_env(env_cfg, device=device)
    env = RLQuadEnv({**train_cfg, 'env': env_cfg['env']}, env_cuda=env_cuda)
    runner = OnPolicyRunner(env, train_cfg, None, device)
    runner.load(str(Path(args.checkpoint)), load_optimizer=False, map_location=device)
    runner.alg.policy.export_to_onnx(str(Path(args.output)))
    print(f'exported {args.output}', flush=True)


if __name__ == '__main__':
    main()
