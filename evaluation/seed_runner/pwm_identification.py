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
    parser.add_argument('--evaluate-truth', action='store_true')
    parser.add_argument('--step', action='store_true')
    parser.add_argument('--mode', choices=('PWM', 'VEL'), default='PWM')
    parser.add_argument('--sensor-fault', choices=('GPS_ZERO','IMU_DROP','IMU_ROLLBACK'))
    parser.add_argument('--fault-seconds', type=float, default=.15)
    parser.add_argument('--gps-bridge',action='store_true')
    args = parser.parse_args()
    if args.mode == 'VEL' and (args.motor_pulses or not 3 <= args.hover_seconds <= 20):
        parser.error('VEL requires hover-seconds [3,20] and no motor pulses')
    if args.sensor_fault and args.mode != 'PWM':
        parser.error('sensor injection currently available for PWM experiments only')
    if args.gps_bridge and (args.mode != 'PWM' or args.hover_seconds < 3):
        parser.error('GPS bridge is only available in bounded PWM hover')
    container = os.environ.get('RMUA_CONTAINER', 'rmua_noetic')
    digest = subprocess.check_output(['docker', 'inspect', container, '--format', '{{.Image}}'], text=True).strip()
    config = json.loads((root/'configs/official_baseline.yaml').read_text())
    config['control_mode'] = args.mode
    config['experiment'] = {'levels': args.levels, 'pulse_seconds': args.pulse_seconds,
                            'motor_pulses': args.motor_pulses, 'hover_seconds': args.hover_seconds,
                            'evaluate_truth': args.evaluate_truth, 'step': args.step,
                            'sensor_fault':args.sensor_fault, 'fault_seconds':args.fault_seconds,
                            'gps_bridge':args.gps_bridge}
    run, manifest = prepare_run(root, root/'runs', args.seed, digest, config)
    manifest['stage'] = 'P0_PWM_IDENTIFICATION' if args.mode == 'PWM' else 'P1_VEL_BENCHMARK'
    print(run, flush=True)
    code = 1
    evaluator = None
    evaluation_log = None
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
            if args.evaluate_truth:
                evaluation_log = (run/'truth_evaluator.log').open('x')
                evaluator = subprocess.Popen([str(root/'tools/enter_env.sh'), 'python3', '-m',
                    'evaluation.metrics.truth_recorder', '--run-dir',
                    '/workspace/main/'+str(run.relative_to(root))], stdout=evaluation_log, stderr=subprocess.STDOUT)
                ready_deadline = time.monotonic()+5
                while not (run/'truth_ready').exists() and time.monotonic() < ready_deadline:
                    if evaluator.poll() is not None:
                        break
                    time.sleep(.05)
                if not (run/'truth_ready').exists():
                    raise RuntimeError('hidden truth evaluator failed to receive truth')
            command = [str(root/'tools/enter_env.sh'), 'env', 'OPENBLAS_NUM_THREADS=1',
                    'python3', '-m', 'adapters.ros_noetic.pwm_experiment' if args.mode == 'PWM' else 'adapters.ros_noetic.vel_experiment',
                    '--run-dir', '/workspace/main/'+str(run.relative_to(root))]
            if args.mode == 'PWM':
                command.extend(['--levels', args.levels, '--pulse-seconds', str(args.pulse_seconds)])
            if args.motor_pulses:
                command.append('--motor-pulses')
            if args.hover_seconds:
                command.extend(['--hover-seconds', str(args.hover_seconds)])
            if args.step:
                command.append('--step')
            if args.sensor_fault:
                command.extend(['--sensor-fault',args.sensor_fault,'--fault-seconds',str(args.fault_seconds)])
            if args.gps_bridge:
                command.append('--gps-bridge')
            code = subprocess.call(command, timeout=40)
        except Exception as error:
            manifest['error'] = str(error)
            print(str(error), flush=True)
        finally:
            if evaluator:
                (run/'truth_stop').touch()
                try:
                    manifest['truth_evaluator_exit_code'] = evaluator.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    evaluator.terminate()
                    manifest['truth_evaluator_exit_code'] = 'TIMEOUT'
                evaluation_log.close()
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
            prefix = 'pwm' if args.mode == 'PWM' else 'vel'
            manifest['telemetry'] = [prefix+'_telemetry.jsonl', prefix+'_summary.json', 'simulator.log']
            if args.evaluate_truth:
                manifest['evaluation_only_inputs'] = ['/airsim_node/drone_1/debug/pose_gt']
                manifest['telemetry'].append('evaluation_truth.jsonl')
            (run/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    if args.evaluate_truth and (run/(prefix+'_telemetry.jsonl')).exists() and (run/'evaluation_truth.jsonl').exists():
        metric_code = subprocess.call(['python3', '-m', 'evaluation.metrics.closed_loop', str(run)], cwd=root)
        if code == 0:
            code = metric_code
    return code


if __name__ == '__main__':
    raise SystemExit(main())
