import csv
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

PIPE = Path(__file__).resolve().parent
LOGS = PIPE / 'logs'
ROOT = Path(os.environ.get('RL_TRAIN_ROOT', '')).expanduser().resolve()
if not (ROOT / 'RL').is_dir():
    raise SystemExit('RL_TRAIN_ROOT must point to the ruma_rl checkout')
PY = os.environ.get('RL_PYTHON', sys.executable)
STOP = PIPE / 'STOP'
GLOBAL_JSON = PIPE / 'global_best.json'
GLOBAL_MODEL = ROOT / 'RL/log_data/rl_pipeline/global_best_model.pt'
GLOBAL_ONNX = PIPE / 'global_best_policy.onnx'
BRIDGE_ONNX = None
if os.environ.get('RL_BRIDGE_ONNX'):
    BRIDGE_ONNX = Path(os.environ['RL_BRIDGE_ONNX']).expanduser()
VARIANTS = [1, 2, 0]


def log(msg):
    stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(f'[{stamp}] {msg}', flush=True)


def pipeline_running():
    out = subprocess.run(
        ['pgrep', '-f', r'python .*pipeline\.py'], capture_output=True, text=True
    )
    pids = [line for line in out.stdout.splitlines() if line.strip()]
    me = str(os.getpid())
    return [pid for pid in pids if pid != me]


def stop_all():
    for pat in ('RL/learn.py', 'power_guard.sh', 'python pipeline.py'):
        subprocess.run(['pkill', '-f', pat], check=False)
    subprocess.run(['pkill', '-f', 'loop_supervisor.py'], check=False)


def start_pipeline(cycle, variant, init_path, log_path):
    env = os.environ.copy()
    env['PIPELINE_INIT'] = init_path
    env['PIPELINE_VARIANT'] = str(variant)
    env['PIPELINE_CYCLE'] = str(cycle)
    env['PYTHONPATH'] = ''
    log_file = open(log_path, 'w')
    proc = subprocess.Popen(
        [PY, 'pipeline.py'],
        cwd=PIPE,
        env=env,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return proc


def read_cycle_best():
    metrics_path = PIPE / 'best_metrics.json'
    if metrics_path.exists():
        try:
            data = json.loads(metrics_path.read_text())
            return {
                'checkpoint': data.get('checkpoint', ''),
                'survival': float(data.get('survival_obst2', 0.0)),
                'survival3': data.get('survival_obst3'),
                'pos_xy': data.get('pos_xy_err'),
                'onnx': data.get('onnx', ''),
            }
        except Exception as exc:
            log(f'failed to read best_metrics.json: {exc}')
    results = PIPE / 'results.csv'
    if not results.exists():
        return None
    rows = []
    with open(results) as f:
        for row in csv.DictReader(f):
            if int(row.get('obstacle_level', 0)) == 2:
                rows.append(row)
    if not rows:
        return None
    best = max(rows, key=lambda r: (float(r['survival']), -float(r.get('err/pos_xy_err') or 999)))
    return {
        'checkpoint': best['checkpoint'],
        'survival': float(best['survival']),
        'survival3': None,
        'pos_xy': float(best.get('err/pos_xy_err') or 999),
        'onnx': str(ROOT / 'RL/log_data/rl_pipeline/best_policy.onnx'),
    }


def global_best():
    if GLOBAL_JSON.exists():
        try:
            return json.loads(GLOBAL_JSON.read_text())
        except Exception:
            return None
    return None


def score(d):
    survival = d.get('survival', d.get('survival_obst2', 0.0))
    pos_xy = d.get('pos_xy', d.get('pos_xy_err'))
    return (float(survival), -(float(pos_xy) if pos_xy is not None else 999.0))


def is_better(new, old):
    if old is None:
        return True
    return score(new) > score(old)


def update_global_best(cycle, variant, best):
    if not best or not Path(best['checkpoint']).exists():
        return
    old = global_best()
    if not is_better(best, old):
        log(f'cycle {cycle} did not beat global best '
            f'(new={best["survival"]:.3f}, old={score(old)[0]:.3f})')
        shutil.copy2(PIPE / 'results.csv', LOGS / f'results_cycle_{cycle}.csv')
        return
    GLOBAL_MODEL.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best['checkpoint'], GLOBAL_MODEL)
    onnx = Path(best.get('onnx', ''))
    if onnx.exists():
        shutil.copy2(onnx, GLOBAL_ONNX)
        if BRIDGE_ONNX and BRIDGE_ONNX.parent.exists():
            shutil.copy2(onnx, BRIDGE_ONNX)
    payload = {
        'checkpoint': str(GLOBAL_MODEL),
        'source_checkpoint': best['checkpoint'],
        'survival_obst2': best['survival'],
        'survival_obst3': best.get('survival3'),
        'pos_xy_err': best.get('pos_xy'),
        'cycle': cycle,
        'variant': variant,
        'onnx': str(GLOBAL_ONNX),
        'updated': datetime.now().isoformat(timespec='seconds'),
    }
    GLOBAL_JSON.write_text(json.dumps(payload, indent=2))
    (PIPE / 'GLOBAL_BEST.md').write_text(
        '# Global best\n\n' + '\n'.join(f'- {k}: `{v}`' for k, v in payload.items()) + '\n'
    )
    log(f'global best updated: survival={best["survival"]:.3f} ckpt={GLOBAL_MODEL}')
    shutil.copy2(PIPE / 'results.csv', LOGS / f'results_cycle_{cycle}.csv')


def prune_checkpoints(keep_paths):
    keep = {str(Path(p)) for p in keep_paths if p}
    for sub in ('A', 'B'):
        root = ROOT / 'RL/log_data/rl_pipeline' / sub
        if not root.exists():
            continue
        for log_dir in [p for p in root.glob('*') if p.is_dir()]:
            ckpts = sorted(log_dir.glob('model_*.pt'),
                           key=lambda p: int(p.stem.split('_')[-1]) if p.stem.split('_')[-1].isdigit() else -1)
            recent = {str(c) for c in ckpts[-12:]}
            for ckpt in ckpts:
                if str(ckpt) in keep or str(ckpt) in recent:
                    continue
                try:
                    ckpt.unlink()
                except OSError:
                    pass


def main():
    LOGS.mkdir(parents=True, exist_ok=True)
    log('supervisor started')
    cycle = 0
    while True:
        if STOP.exists():
            log('STOP file detected, exiting supervisor')
            stop_all()
            return 0
        running = pipeline_running()
        if running:
            time.sleep(30)
            continue

        variant = VARIANTS[cycle % len(VARIANTS)]
        init = (
            'RL/log_data/rl_pipeline/global_best_model.pt'
            if GLOBAL_MODEL.exists()
            else os.environ.get('PIPELINE_INIT', '')
        )
        if not init:
            log('no global best and PIPELINE_INIT is unset; exiting')
            return 2
        cycle_log = LOGS / f'pipeline_cycle_{cycle}.log'
        log(f'starting cycle {cycle} variant={variant} init={init}')
        proc = start_pipeline(cycle, variant, init, cycle_log)
        while proc.poll() is None:
            if STOP.exists():
                log('STOP detected while pipeline running, stopping all')
                proc.send_signal(signal.SIGTERM)
                stop_all()
                return 0
            time.sleep(30)
        log(f'cycle {cycle} finished rc={proc.poll()}')
        best = read_cycle_best()
        update_global_best(cycle, variant, best)
        keep = []
        if best:
            keep.append(best.get('checkpoint'))
        if GLOBAL_JSON.exists():
            try:
                keep.append(json.loads(GLOBAL_JSON.read_text()).get('source_checkpoint'))
            except Exception:
                pass
        prune_checkpoints(keep)
        for name in ('SUMMARY.md', 'best_metrics.json'):
            src = PIPE / name
            if src.exists():
                shutil.copy2(src, LOGS / f'{name.replace(".", "_")}_cycle_{cycle}{src.suffix}')
        cycle += 1
        time.sleep(20)


if __name__ == '__main__':
    raise SystemExit(main())
