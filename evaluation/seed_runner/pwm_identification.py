"""Own a fresh local scene, run bounded PWM experiment, always stop the scene."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from evaluation.seed_runner.manifest import prepare_run, source_hashes


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--levels', default='.10,.15,.20,.25,.30,.35,.40')
    parser.add_argument('--pulse-seconds', type=float, default=.3)
    parser.add_argument('--motor-pulses', action='store_true')
    parser.add_argument('--hover-seconds', type=float, default=0.)
    args = parser.parse_args()
    container = os.environ.get('RMUA_CONTAINER', 'rmua_noetic')
    digest = subprocess.check_output(['docker', 'inspect', container, '--format', '{{.Image}}'], text=True).strip()
    config = json.loads((root/'configs/official_baseline.yaml').read_text())
    config['experiment'] = {'levels': args.levels, 'pulse_seconds': args.pulse_seconds,
                            'motor_pulses': args.motor_pulses, 'hover_seconds': args.hover_seconds}
    run, manifest = prepare_run(root, root/'runs', args.seed, digest, config)
    manifest['stage'] = 'P0_PWM_IDENTIFICATION'
    print(run, flush=True)
    code = 1
    with (run/'simulator.log').open('x') as output:
        simulator = subprocess.Popen([str(root/'tools/run_sim.sh'), str(args.seed), 'offscreen'],
                                     stdout=output, stderr=subprocess.STDOUT)
        try:
            # Wait for live official inputs, never a fixed blind startup delay.
            deadline = time.monotonic()+90
            while time.monotonic() < deadline:
                if simulator.poll() is not None:
                    raise RuntimeError('fresh simulator failed; see simulator.log')
                try:
                    probe = subprocess.run([str(root/'tools/enter_env.sh'), 'python3', '-c',
                        'import rospy; from sensor_msgs.msg import Imu; rospy.init_node("pwm_ready", anonymous=True); rospy.wait_for_message("/airsim_node/drone_1/imu/imu", Imu, timeout=2)'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=8)
                except subprocess.TimeoutExpired:
                    continue
                if probe.returncode == 0:
                    break
            else:
                raise RuntimeError('simulator sensor readiness timeout')
            command = [str(root/'tools/enter_env.sh'), 'env', 'OPENBLAS_NUM_THREADS=1',
                    'python3', '-m', 'adapters.ros_noetic.pwm_experiment', '--run-dir',
                    '/workspace/main/'+str(run.relative_to(root)), '--levels', args.levels,
                    '--pulse-seconds', str(args.pulse_seconds)]
            if args.motor_pulses:
                command.append('--motor-pulses')
            if args.hover_seconds:
                command.extend(['--hover-seconds', str(args.hover_seconds)])
            code = subprocess.call(command, timeout=40)
        except Exception as error:
            manifest['error'] = str(error)
            print(str(error), flush=True)
        finally:
            # Only stop a process group launched by this invocation.
            if simulator.poll() is None:
                subprocess.run([str(root/'tools/stop_sim.sh')], timeout=15)
                try:
                    simulator.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    simulator.terminate()
            manifest['source_sha256_after'] = source_hashes(root)
            manifest['source_unchanged'] = manifest['source_sha256_before'] == manifest['source_sha256_after']
            manifest['observer_exit_code'] = code
            manifest['telemetry'] = ['pwm_telemetry.jsonl', 'pwm_summary.json', 'simulator.log']
            (run/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
