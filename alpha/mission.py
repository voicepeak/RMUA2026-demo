"""P0 host orchestration and bounded PWM placeholder controller."""
import argparse
import hashlib
import math
from datetime import datetime
import json
import os
import signal
from pathlib import Path
import subprocess
import sys
import threading
import time

HERE = Path(__file__).resolve().parent


def controller(run):
    # Use the existing guarded PWM chain. This process never imports truth topics.
    sys.path.remove(str(HERE))
    sys.path.insert(0, str(HERE.parent))
    import numpy as np
    from adapters.ros_noetic import pwm_experiment as pwm
    live = {'elapsed': None, 'last': 0.0, 'status': None}
    observer_stop = threading.Event()
    original_read = pwm.FlightSensors.read
    def read_with_hud(sensors):
        state, raw = original_read(sensors)
        now = time.monotonic()
        if now-live['last'] >= .2:
            live['last'] = now
            elapsed = live['elapsed']
            phase = ('initializing' if elapsed is None else 'takeoff' if elapsed < 3
                     else 'hover' if elapsed < 8 else 'descent' if elapsed < 11 else 'ground reference')
            status = {'phase': phase, 'position_ned': list(state.position_ned),
                      'velocity_ned': list(state.velocity_ned), 'health': state.health.value,
                      'elapsed': elapsed}
            live['status'] = status
        return state, raw
    pwm.FlightSensors.read = read_with_hud
    def write_hud():
        target = Path(run)/'hud_state.json'
        while not observer_stop.wait(.2):
            status = live['status']
            if status is not None:
                try:
                    tmp = target.with_suffix('.tmp')
                    tmp.write_text(json.dumps(status))
                    tmp.replace(target)
                except OSError:
                    pass
    observer = threading.Thread(target=write_hud, daemon=True)
    observer.start()
    def reference(origin, elapsed, *args):
        live['elapsed'] = elapsed
        # 3 s ascent, 5 s hold, 3 s descent; then retain ground reference.
        if elapsed < 3:
            u, sign = min(1., elapsed / 3), 1
        elif elapsed < 8:
            return tuple(np.asarray(origin) + (0, 0, -1)), (0., 0., 0.), (0., 0., 0.)
        elif elapsed < 11:
            u, sign = (elapsed - 8) / 3, -1
        else:
            return tuple(origin), (0., 0., 0.), (0., 0., 0.)
        q = 10*u**3 - 15*u**4 + 6*u**5
        dq = (30*u**2 - 60*u**3 + 30*u**4)/3
        ddq = (60*u - 180*u**2 + 120*u**3)/9
        height = q if sign == 1 else 1-q
        return tuple(np.asarray(origin)+(0, 0, -height)), (0., 0., -sign*dq), (0., 0., -sign*ddq)
    pwm.benchmark_reference = reference
    try:
        return pwm.run_experiment(Path(run), (), .3, hover_seconds=14, stationary_bootstrap=True)
    finally:
        observer_stop.set()
        observer.join(timeout=1)



def evaluate_p0(rows):
    if not rows:
        raise ValueError('missing telemetry')
    origin = float(rows[0]['reference'][2])
    samples = [(float(r['sim_elapsed']), float(r['state']['position_ned'][2])) for r in rows]
    if any(not math.isfinite(t) or not math.isfinite(z) for t, z in samples):
        raise ValueError('invalid telemetry')
    hover = [(t, z) for t, z in samples if 3 <= t <= 8]
    if (not hover or hover[0][0] > 3.15 or hover[-1][0] < 7.85
            or any(b[0]-a[0] > .3 for a, b in zip(hover, hover[1:]))):
        raise ValueError('hover telemetry does not cover the five-second reference')
    # The planned hover target is one metre above the starting reference.
    hover_min_rise = min(origin-z for _, z in hover)
    final_t, final_z = samples[-1]
    error = abs(final_z-origin)
    if hover_min_rise < .5:
        raise ValueError('takeoff/hover rise less than 0.5m')
    if final_t < 11 or error > .3:
        raise ValueError('return to starting height not verified')
    return {'start_z_ned': origin, 'end_z_ned': final_z, 'return_error_m': error,
            'hover_observed_s': hover[-1][0]-hover[0][0], 'hover_reference_s': 5,
            'hover_min_rise_m': hover_min_rise, 'last_sim_elapsed_s': final_t,
            'interpretation': 'estimated height, not physical contact'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('seed', type=int)
    args = parser.parse_args()
    if args.seed < 0:
        parser.error('seed must be nonnegative')
    timestamp = datetime.now().astimezone()
    run = HERE/'runs'/(timestamp.strftime('%Y%m%d_%H%M%S_%f')+'_'+str(args.seed))
    run.mkdir(parents=True)
    commit = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], cwd=HERE, text=True).strip()
    dirty = bool(subprocess.check_output(['git', 'status', '--porcelain', '--', 'alpha'], cwd=HERE.parent, text=True).strip())
    cont = os.environ.get('RMUA_CONTAINER', 'rmua_next_gpu')
    # Existing workspace mount is checked by simulator startup and controller launch.
    relative = run.relative_to(HERE.parent.parent)
    container_run = '/workspace/'+str(relative)
    meta = {'seed': args.seed, 'commit': commit, 'dirty_alpha': dirty, 'simulator': 'simulator_12.0.0.5', 'launcher': 'run_simulator_offscreen.sh', 'container': cont, 'control': 'PWM', 'sensors': 'official IMU/GPS and front-left camera', 'official_score': 'unavailable'}
    meta['alpha_sha256'] = {name: hashlib.sha256((HERE/name).read_bytes()).hexdigest()
                            for name in ('mission.py', 'view.py', 'run.sh', 'sim_up.sh', 'sim_down.sh')}
    (run/'meta.json').write_text(json.dumps(meta, indent=2)+'\n')
    result = {'ts': timestamp.isoformat(timespec='seconds'), 'seed': args.seed, 'stage': 0, 'score': 0, 'time_s': 0., 'fail': 'env', 'commit': commit, 'note': 'P0 placeholder; official score unavailable; dirty_alpha='+str(dirty).lower()}
    start = None
    hud = None
    def interrupted(signum, _frame):
        raise RuntimeError('host interrupted by signal '+str(signum))
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    try:
        subprocess.run([str(HERE/'sim_up.sh'), str(args.seed)], check=True, env={**os.environ, 'ALPHA_SIM_LOG': container_run+'/simulator.log'})
        subprocess.run(['docker', 'exec', cont, 'bash', '-c', 'test -x /workspace/alpha/alpha/.tools/ffmpeg/usr/bin/ffmpeg || command -v ffmpeg'], check=True)
        command = 'source /opt/ros/noetic/setup.bash; source /workspace/rmua_ws/devel/setup.bash; test ! -x /workspace/alpha/alpha/.tools/ffmpeg/usr/bin/ffmpeg || export ALPHA_FFMPEG=/workspace/alpha/alpha/.tools/ffmpeg/usr/bin/ffmpeg; exec python3 /workspace/alpha/alpha/view.py "$1/hud.mp4"'
        hud = subprocess.Popen(['docker', 'exec', '-u', '1000:1000', cont, 'bash', '-c', command, 'alpha-hud', container_run], stdout=(run/'hud.log').open('w'), stderr=subprocess.STDOUT)
        time.sleep(2)
        if hud.poll() is not None:
            raise RuntimeError('HUD recorder exited before flight')
        start = time.monotonic()
        command = 'source /opt/ros/noetic/setup.bash; source /workspace/rmua_ws/devel/setup.bash; source /workspace/main/.runtime_ws/devel/setup.bash; cd /workspace/alpha; exec python3 alpha/mission.py --controller "$1"'
        with (run/'controller.log').open('w') as log:
            proc = subprocess.run(['docker', 'exec', '-u', '1000:1000', cont, 'bash', '-c', command, 'alpha-controller', container_run], stdout=log, stderr=subprocess.STDOUT, timeout=35)
        result['time_s'] = round(time.monotonic()-start, 2)
        telemetry = run/'pwm_telemetry.jsonl'
        if telemetry.exists():
            with telemetry.open() as source:
                rows = [json.loads(line) for line in source]
            if rows:
                result['time_s'] = round(rows[-1]['wall_elapsed'], 2)
        if proc.returncode != 0:
            summary = run/'pwm_summary.json'
            fault = json.loads(summary.read_text()).get('fault') if summary.exists() else None
            raise RuntimeError('PWM controller failed: '+str(fault or 'unknown'))
        evaluation = evaluate_p0(rows if telemetry.exists() else [])
        (run/'p0_evaluation.json').write_text(json.dumps(evaluation, indent=2)+'\n')
        result['fail'] = None
        result['note'] += '; P0 estimated-height acceptance; no contact claim; return_error_m=%.3f' % evaluation['return_error_m']
        subprocess.run(['docker', 'exec', cont, 'pkill', '-INT', '-f', '[p]ython3 /workspace/alpha/alpha/view.py'], check=False)
        try:
            hud.wait(timeout=20)
        except subprocess.TimeoutExpired:
            hud.terminate()
        if hud.returncode != 0 or not (run/'hud.mp4').is_file() or (run/'hud.mp4').stat().st_size == 0:
            raise RuntimeError('HUD recording failed')
        video_check = 'if test -x /workspace/alpha/alpha/.tools/ffmpeg/usr/bin/ffprobe; then probe=/workspace/alpha/alpha/.tools/ffmpeg/usr/bin/ffprobe; else probe=ffprobe; fi; "$probe" -v error -select_streams v:0 -show_entries stream=codec_name -of csv=p=0 "$1/hud.mp4" | grep -qx h264'
        subprocess.run(['docker', 'exec', cont, 'bash', '-c', video_check, 'alpha-video', container_run], check=True)
    except (subprocess.SubprocessError, OSError, RuntimeError, ValueError) as exc:
        result['fail'] = 'env'
        result['note'] += '; '+str(exc)
        fault_file = run/'fault_observation.json'
        if fault_file.exists():
            fault_data = json.loads(fault_file.read_text())
            elapsed = fault_data.get('wall_elapsed_at_last_read')
            if elapsed is not None:
                result['time_s'] = round(elapsed, 2)
        elif start is not None and not (run/'pwm_telemetry.jsonl').exists():
            result['time_s'] = round(time.monotonic()-start, 2)
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        if hud and hud.poll() is None:
            subprocess.run(['docker', 'exec', cont, 'pkill', '-INT', '-f', '[p]ython3 /workspace/alpha/alpha/view.py'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
            try:
                hud.wait(timeout=15)
            except subprocess.TimeoutExpired:
                hud.terminate()
        if (HERE/'.sim-owner').exists():
            if subprocess.run([str(HERE/'sim_down.sh')]).returncode:
                result['fail'] = 'env'
                result['note'] += '; cleanup failed'
        with (HERE/'scoreboard.jsonl').open('a') as out:
            out.write(json.dumps(result, ensure_ascii=False)+'\n')
        print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result['fail'] is None else 1


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--controller':
        raise SystemExit(controller(sys.argv[2]))
    raise SystemExit(main())
