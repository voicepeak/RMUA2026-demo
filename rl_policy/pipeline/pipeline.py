import csv
import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

PIPE = Path(__file__).resolve().parent
LOGS = PIPE / 'logs'
ROOT = Path(os.environ.get('RL_TRAIN_ROOT', '')).expanduser().resolve()
if not (ROOT / 'RL').is_dir():
    raise SystemExit('RL_TRAIN_ROOT must point to the ruma_rl checkout')
CONDA_SH = os.environ.get('RL_CONDA_SH', '')
CONDA_ENV = os.environ.get('RL_CONDA_ENV', '')
GUARD = None
if os.environ.get('RL_POWER_GUARD'):
    GUARD = Path(os.environ['RL_POWER_GUARD']).expanduser().resolve()
START_TIME = time.time()
GLOBAL_DEADLINE = START_TIME + float(os.environ.get('RL_GLOBAL_HOURS', '48')) * 3600
PHASE_A_ITERS = int(os.environ.get('RL_PHASE_A_ITERS', '3200'))
PHASE_B_ITERS = int(os.environ.get('RL_PHASE_B_ITERS', '1800'))
INIT_CHECKPOINT = os.environ.get('PIPELINE_INIT', '')
VARIANT = int(os.environ.get('PIPELINE_VARIANT', '0'))


def variant_params():
    if VARIANT == 1:
        return dict(a_p=0.5, a_lat=0.1, a_obst=3, a_clr=1.2,
                    b_p=0.8, b_lat=0.2, b_clr=1.2,
                    b_dacc=0.05, b_djerk=0.006,
                    a_iters=2500, b_iters=1500,
                    success=0.65, entropy=0.005, skip_b=False)
    if VARIANT == 2:
        return dict(a_p=1.0, a_lat=0.3, a_obst=2, a_clr=0.8,
                    b_p=1.2, b_lat=0.4, b_clr=0.8,
                    b_dacc=0.06, b_djerk=0.008,
                    a_iters=3000, b_iters=1800,
                    success=0.65, entropy=0.005, skip_b=False)
    if VARIANT == 3:
        return dict(a_p=1.5, a_lat=0.5, a_obst=2, a_clr=0.0,
                    b_p=2.5, b_lat=1.0, b_clr=0.0,
                    b_dacc=0.06, b_djerk=0.008,
                    a_iters=2000, b_iters=0,
                    success=0.70, entropy=0.010, skip_b=True)
    return dict(a_p=0.8, a_lat=0.2, a_obst=2, a_clr=1.0,
                b_p=1.0, b_lat=0.3, b_clr=1.0,
                b_dacc=0.05, b_djerk=0.006,
                a_iters=3000, b_iters=1800,
                success=0.65, entropy=0.005, skip_b=False)


def log(msg):
    stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(f'[{stamp}] {msg}', flush=True)


def base_config():
    with open(PIPE / 'config_rl_base.yaml') as f:
        return yaml.safe_load(f)


def write_config(cfg, name):
    config_path = ROOT / 'RL/configs/config_rl.yaml'
    with open(config_path, 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
    shutil.copy2(config_path, LOGS / name)


def shell_cmd(inner):
    parts = []
    if CONDA_SH:
        parts.append(f'source {shlex.quote(CONDA_SH)}')
    if CONDA_ENV:
        parts.append(f'conda activate {shlex.quote(CONDA_ENV)}')
    parts.append('export PYTHONPATH=')
    parts.append(f'export RL_TRAIN_ROOT={shlex.quote(str(ROOT))}')
    parts.append(f'cd {shlex.quote(str(ROOT))}')
    return ['bash', '-lc', ' && '.join(parts) + ' && ' + inner]


def start_training(tag, log_path):
    log_file = open(log_path, 'w')
    cmd = shell_cmd(f'exec {shlex.quote(sys.executable)} -u RL/learn.py')
    proc = subprocess.Popen(
        cmd,
        cwd=ROOT,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    time.sleep(8)
    if proc.poll() is not None:
        log(f'{tag} training exited immediately, see {log_path}')
        return None
    guard = None
    if GUARD and GUARD.exists():
        guard = subprocess.Popen(
            ['bash', str(GUARD)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    guard_pid = guard.pid if guard else 'none'
    log(f'{tag} training started pid={proc.pid}, guard pid={guard_pid}')
    return proc


def wait_training(tag, proc, deadline):
    while proc.poll() is None:
        if (PIPE / 'STOP').exists():
            log(f'{tag} STOP file detected, terminating training')
            proc.send_signal(signal.SIGTERM)
            time.sleep(3)
            if proc.poll() is None:
                proc.kill()
            break
        if time.time() > deadline:
            log(f'{tag} wall deadline reached, terminating training')
            proc.send_signal(signal.SIGTERM)
            for _ in range(30):
                if proc.poll() is not None:
                    break
                time.sleep(1)
            if proc.poll() is None:
                proc.kill()
            break
        time.sleep(30)
    log(f'{tag} training finished rc={proc.poll()}')
    time.sleep(3)
    if GUARD:
        subprocess.run(['pkill', '-f', 'power_guard.sh'], check=False)


def checkpoint_iter(path):
    m = re.search(r'model_(\d+)\.pt$', path.name)
    return int(m.group(1)) if m else -1


def list_checkpoints(log_root):
    dirs = sorted([p for p in log_root.glob('*') if p.is_dir()], key=lambda p: p.stat().st_mtime)
    if not dirs:
        return None, []
    log_dir = dirs[-1]
    ckpts = sorted(log_dir.glob('model_*.pt'), key=checkpoint_iter)
    return log_dir, ckpts


def pick_candidates(ckpts, n):
    if len(ckpts) <= n:
        return ckpts
    idxs = [round(i * (len(ckpts) - 1) / (n - 1)) for i in range(n)]
    return [ckpts[i] for i in idxs]


def run_eval(ckpts, obstacle, out_path, tag):
    if not ckpts:
        return []
    if out_path.exists():
        out_path.unlink()
    log(f'evaluating {tag}: {len(ckpts)} checkpoints at obstacle {obstacle}')
    quoted = ' '.join(str(c) for c in ckpts)
    inner = (
        f'exec {shlex.quote(sys.executable)} {PIPE}/eval_sweep.py --checkpoints {quoted} '
        f'--speed 3 --obstacle {obstacle} --steps 600 --out {out_path}'
    )
    subprocess.run(shell_cmd(inner), cwd=ROOT, check=True)
    rows = []
    with open(out_path) as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def best_row(rows):
    return max(rows, key=lambda r: (r['survival'], -r.get('err/pos_xy_err', 999.0)))


def phase_a_config():
    cfg = base_config()
    cfg['train'] = {
        'resume': INIT_CHECKPOINT,
        'load_optimizer': False,
        'target_speed': 10,
    }
    vp = variant_params()
    cfg['coef'].update({
        'v': 1.0,
        'collide': 15,
        'obj_avoidance': 6.0,
        'd_acc': 0.05,
        'd_jerk': 0.006,
        'p_bound': 2.0,
        'p_center': vp['a_p'],
        'lat_v': vp['a_lat'],
        'clearance': vp['a_clr'],
    })
    cfg['reward']['clearance_target'] = 0.9
    cfg['curriculum'] = {
        'enabled': True,
        'start_from_max': False,
        'start_speed_level': 3,
        'start_obstacle_level': vp['a_obst'],
        'success_threshold': vp['success'],
    }
    cfg['log'] = {'root': 'log_data/rl_pipeline/A', 'run_name': '_A'}
    cfg['max_iterations'] = vp['a_iters']
    cfg['save_interval'] = 50
    cfg['experiment_name'] = 'moderate_offset_A'
    cfg['algorithm']['learning_rate'] = 3.0e-4
    cfg['algorithm']['entropy_coef'] = vp['entropy']
    return cfg


def phase_b_config(resume_ckpt, obstacle_level, iterations):
    cfg = base_config()
    cfg['train'] = {
        'resume': str(resume_ckpt.relative_to(ROOT)),
        'load_optimizer': False,
        'target_speed': 10,
    }
    vp = variant_params()
    cfg['coef'].update({
        'v': 1.0,
        'collide': 15,
        'obj_avoidance': 6.0,
        'd_acc': vp['b_dacc'],
        'd_jerk': vp['b_djerk'],
        'p_bound': 2.0,
        'p_center': vp['b_p'],
        'lat_v': vp['b_lat'],
        'clearance': vp['b_clr'],
    })
    cfg['reward']['clearance_target'] = 0.9
    cfg['curriculum'] = {
        'enabled': True,
        'start_from_max': False,
        'start_speed_level': 3,
        'start_obstacle_level': obstacle_level,
        'success_threshold': vp['success'],
    }
    cfg['log'] = {'root': 'log_data/rl_pipeline/B', 'run_name': '_B'}
    cfg['max_iterations'] = iterations
    cfg['save_interval'] = 50
    cfg['experiment_name'] = 'moderate_offset_B'
    cfg['algorithm']['learning_rate'] = 2.0e-4
    cfg['algorithm']['entropy_coef'] = vp['entropy']
    return cfg


def write_summary(results, best, best_onnx, phase_a_dir, phase_b_dir, a_best, b_best):
    results_csv = PIPE / 'results.csv'
    fields = [
        'phase', 'checkpoint', 'obstacle_level', 'episodes', 'survival', 'early_fail',
        'err/pos_xy_err', 'err/pos_z_err', 'err/vel_err_l2', 'err/min_clearance',
        'stats/collide_rate', 'reward/rew_p_center', 'reward/rew_lat_v', 'reward/rew_clearance',
    ]
    with open(results_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        for row in results:
            writer.writerow(row)

    lines = []
    lines.append('# RMUA RL fine-tune pipeline results')
    lines.append('')
    lines.append(f'- generated: {datetime.now().isoformat(timespec="seconds")}')
    lines.append(f'- best checkpoint: `{best}`')
    lines.append(f'- best ONNX: `{best_onnx}`')
    lines.append(f'- phase A log dir: `{phase_a_dir}`')
    if phase_b_dir:
        lines.append(f'- phase B log dir: `{phase_b_dir}`')
    lines.append(f'- phase A best: `{a_best}`')
    if b_best:
        lines.append(f'- phase B best: `{b_best}`')
    lines.append('')
    lines.append('| phase | checkpoint | obstacle | survival | pos_xy_err | min_clearance | collide_rate |')
    lines.append('| --- | --- | --- | --- | --- | --- | --- |')
    for row in results:
        phase = row.get('phase', '')
        lines.append(
            '| %s | %s | %s | %.3f | %.3f | %.3f | %.4f |' % (
                phase,
                Path(row['checkpoint']).name,
                row['obstacle_level'],
                row['survival'],
                row.get('err/pos_xy_err', 0.0),
                row.get('err/min_clearance', 0.0),
                row.get('stats/collide_rate', 0.0),
            )
        )
    lines.append('')
    lines.append('Notes:')
    lines.append('- survival = reached 150 control steps without collision/out-of-bound')
    lines.append('- all evals are deterministic (mean action), 1024 envs x 600 steps')
    lines.append('- rewards changed: quadratic offset penalty p_center, lateral velocity penalty lat_v')
    lines.append('- Phase B uses stronger offset/lateral penalties and starts from the best Phase A checkpoint')
    (PIPE / 'SUMMARY.md').write_text('\n'.join(lines) + '\n')


def main():
    LOGS.mkdir(parents=True, exist_ok=True)
    if (PIPE / 'STOP').exists():
        log('STOP file exists, skip this cycle')
        return 0
    if not INIT_CHECKPOINT:
        log('PIPELINE_INIT is required')
        return 2
    log(f'starting cycle variant={VARIANT} init={INIT_CHECKPOINT}')
    log('stopping official simulator/bridge to free GPU')
    subprocess.run(
        ['docker', 'exec', 'rmua_noetic', 'bash', '-lc', 'pkill -f rl_policy_bridge_node; true'],
        check=False,
    )
    subprocess.run(['docker', 'restart', 'rmua_noetic'], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-f', 'RL/learn.py'], check=False)
    if GUARD:
        subprocess.run(['pkill', '-f', 'power_guard.sh'], check=False)
    time.sleep(2)

    cfg_a = phase_a_config()
    write_config(cfg_a, 'config_phaseA.yaml')
    a_log = LOGS / 'phaseA_train.log'
    proc_a = start_training('phaseA', a_log)
    if proc_a is None:
        return 1
    a_deadline = min(START_TIME + 24 * 3600, GLOBAL_DEADLINE - 4.5 * 3600)
    wait_training('phaseA', proc_a, a_deadline)

    a_dir, a_ckpts = list_checkpoints(ROOT / 'RL/log_data/rl_pipeline/A')
    if not a_dir or not a_ckpts:
        log('no phase A checkpoints found, abort')
        return 1
    log(f'phase A dir={a_dir} checkpoints={len(a_ckpts)}')
    a_cands = pick_candidates(a_ckpts, 8)
    a_rows = run_eval(a_cands, 2, LOGS / 'A_eval_obst2.jsonl', 'A obst2')
    for row in a_rows:
        row['phase'] = 'A'
    a_best_row = best_row(a_rows)
    a_best = Path(a_best_row['checkpoint'])
    log(f'phase A best={a_best} survival={a_best_row["survival"]:.3f} '
        f'pos_xy={a_best_row.get("err/pos_xy_err", 0):.3f}')

    results = list(a_rows)
    b_rows = []
    b_best = None
    b_dir = None
    b_candidates = []
    if VARIANT == 1:
        b_obstacle = 3
    elif VARIANT == 2:
        b_obstacle = 3 if a_best_row['survival'] >= 0.70 else 2
    else:
        b_obstacle = 3 if a_best_row['survival'] >= 0.70 else 2

    remaining = GLOBAL_DEADLINE - time.time()
    vp = variant_params()
    if vp.get('skip_b'):
        log('variant requests no phase B; keeping phase A best')
    elif remaining > 3.5 * 3600:
        b_iterations = min(vp['b_iters'], max(300, int((remaining - 1.5 * 3600) / 6.5)))
        log(f'starting phase B from {a_best.name}, obstacle={b_obstacle}, iters={b_iterations}')
        cfg_b = phase_b_config(a_best, b_obstacle, b_iterations)
        write_config(cfg_b, 'config_phaseB.yaml')
        b_log = LOGS / 'phaseB_train.log'
        proc_b = start_training('phaseB', b_log)
        if proc_b is not None:
            b_deadline = min(time.time() + 24 * 3600, GLOBAL_DEADLINE - 1.2 * 3600)
            wait_training('phaseB', proc_b, b_deadline)
            b_dir, b_ckpts = list_checkpoints(ROOT / 'RL/log_data/rl_pipeline/B')
            if b_dir and b_ckpts:
                b_candidates = pick_candidates(b_ckpts, 6)
                b_rows = run_eval(b_candidates, 2, LOGS / 'B_eval_obst2.jsonl', 'B obst2')
                for row in b_rows:
                    row['phase'] = 'B'
                results.extend(b_rows)
                b_best_row = best_row(b_rows)
                b_best = Path(b_best_row['checkpoint'])
                log(f'phase B best={b_best} survival={b_best_row["survival"]:.3f}')
            else:
                log('no phase B checkpoints found, skipping B results')
        else:
            log('phase B failed to start')
    else:
        log('not enough remaining time, skipping phase B')

    combined = [r for r in results if r['obstacle_level'] == 2]
    top = sorted(combined, key=lambda r: (r['survival'], -r.get('err/pos_xy_err', 999.0)),
                 reverse=True)[:4]
    top_ckpts = [Path(r['checkpoint']) for r in top]
    obs3_rows = run_eval(top_ckpts, 3, LOGS / 'top_eval_obst3.jsonl', 'top obst3')
    obs3_by_ckpt = {r['checkpoint']: r for r in obs3_rows}
    for row in obs3_rows:
        row['phase'] = next((r['phase'] for r in results if r['checkpoint'] == row['checkpoint']), 'top')
    results.extend(obs3_rows)

    def overall_key(row):
        o3 = obs3_by_ckpt.get(row['checkpoint'], {}).get('survival', 0.0)
        return (row['survival'], o3, -row.get('err/pos_xy_err', 999.0))

    overall = max(combined, key=overall_key)
    best = Path(overall['checkpoint'])
    best_onnx = ROOT / 'RL/log_data/rl_pipeline/best_policy.onnx'
    log(f'overall best={best}')
    subprocess.run(
        shell_cmd(
            f'exec {shlex.quote(sys.executable)} {PIPE}/export_onnx.py --checkpoint {best} --output {best_onnx}'
        ),
        cwd=ROOT,
        check=True,
    )
    shutil.copy2(best, ROOT / 'RL/log_data/rl_pipeline/best_model.pt')
    bridge_model = None
    if os.environ.get('RL_BRIDGE_ONNX'):
        bridge_model = Path(os.environ['RL_BRIDGE_ONNX']).expanduser()
    if bridge_model and bridge_model.parent.exists():
        shutil.copy2(best_onnx, bridge_model)

    o3 = obs3_by_ckpt.get(str(best), {})
    metrics = {
        'checkpoint': str(best),
        'survival_obst2': overall['survival'],
        'survival_obst3': o3.get('survival'),
        'pos_xy_err': overall.get('err/pos_xy_err'),
        'min_clearance': overall.get('err/min_clearance'),
        'onnx': str(best_onnx),
        'variant': VARIANT,
        'timestamp': datetime.now().isoformat(timespec='seconds'),
    }
    (PIPE / 'best_metrics.json').write_text(json.dumps(metrics, indent=2))
    write_summary(results, best, best_onnx, a_dir, b_dir, a_best, b_best)
    log(f'done. best={best} onnx={best_onnx} summary={PIPE / "SUMMARY.md"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
