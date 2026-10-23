"""Own a fresh uninstrumented scene for read-only LiDAR or stereo observation."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

from evaluation.seed_runner.controller_cleanup import stop_bound_controller
from evaluation.seed_runner.manifest import prepare_run, source_hashes
from evaluation.seed_runner.simulator_preflight import verify_local_simulator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--kind', choices=('lidar', 'stereo', 'pose-history'), default='lidar')
    parser.add_argument('--image-age-budget-ms', type=int, choices=(150, 200), default=150)
    args = parser.parse_args()
    if args.seed < 0 or not math.isfinite(args.duration) or not 0 < args.duration <= 30:
        parser.error('nonnegative seed and duration in (0,30] required')
    if args.kind != 'stereo' and args.image_age_budget_ms != 150:
        parser.error('image age experiment requires --kind stereo')
    root = Path(__file__).resolve().parents[2]
    container = os.environ.get('RMUA_CONTAINER', 'rmua_noetic')
    digest = subprocess.check_output(['docker', 'inspect', container, '--format', '{{.Image}}'],
                                     text=True).strip()
    before = verify_local_simulator(root)
    config = json.loads((root / 'configs/official_baseline.yaml').read_text())
    config['experiment'] = {'kind': args.kind + '_read_only', 'duration_s': args.duration}
    if args.kind == 'stereo':
        config['experiment']['image_age_budget_ms'] = args.image_age_budget_ms
    run, manifest = prepare_run(root, root / 'runs', args.seed, digest, config)
    stages = {'lidar': 'P2_READ_ONLY_LIDAR_ENDPOINTS', 'stereo': 'P2_READ_ONLY_STEREO_PAIRING',
              'pose-history': 'P2_READ_ONLY_POSE_HISTORY'}
    manifest.update(stage=stages[args.kind], acceptance_eligible=False,
                    simulator_preflight=before, run_state='PREPARED')
    (run / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    container_run = '/workspace/main/' + str(run.relative_to(root))
    module = 'adapters.ros_noetic.' + ('stereo_observer' if args.kind == 'stereo' else 'lidar_observer')
    audit_module = 'evaluation.metrics.' + ('stereo_pairing' if args.kind == 'stereo' else 'lidar_observation')
    if args.kind == 'pose-history':
        module, audit_module = 'adapters.ros_noetic.pose_history_observer', 'evaluation.metrics.pose_history'
    sensor_type = 'PointCloud2' if args.kind == 'lidar' else 'Imu'
    sensor_topic = '/airsim_node/drone_1/lidar' if args.kind == 'lidar' else '/airsim_node/drone_1/imu/imu'
    marker = 'rmua_' + args.kind.replace('-', '_') + '_sensor_readiness'
    code = 1
    observer_code = None
    print(run, flush=True)
    with (run / 'simulator.log').open('x') as log:
        simulator = subprocess.Popen([str(root / 'tools/run_sim.sh'), str(args.seed), 'offscreen'],
                                     stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline:
                if simulator.poll() is not None:
                    raise RuntimeError('fresh simulator failed; see simulator.log')
                try:
                    ready = subprocess.run([str(root / 'tools/enter_env.sh'), 'python3', '-c',
                        'import socket,sys; socket.setdefaulttimeout(2.); import rosgraph,rospy; '
                        'from sensor_msgs.msg import ' + sensor_type + '; '
                        'sys.exit(2) if not rosgraph.is_master_online() else None; '
                        'rospy.init_node("sensor_ready",anonymous=True); '
                        'rospy.wait_for_message(' + repr(sensor_topic) + ',' + sensor_type + ',timeout=2)',
                        marker, '--run-dir', container_run], stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL, timeout=8)
                except subprocess.TimeoutExpired:
                    stop_bound_controller(container, marker, container_run)
                    continue
                if ready.returncode == 0:
                    break
            else:
                raise RuntimeError(args.kind + ' readiness timeout')
            manifest['run_state'] = 'OBSERVING'
            (run / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
            with (run / 'observer.log').open('x') as observer_log:
                command = [str(root / 'tools/enter_env.sh'), 'env',
                        'OPENBLAS_NUM_THREADS=1', 'python3', '-m', module, '--run-dir', container_run,
                        '--duration', str(args.duration)]
                if args.kind == 'stereo':
                    command.extend(['--image-age-budget-ms', str(args.image_age_budget_ms)])
                code = subprocess.call(command, stdout=observer_log,
                        stderr=subprocess.STDOUT, timeout=args.duration + 15)
                observer_code = code
            manifest['run_state'] = 'OBSERVED' if code == 0 else 'OBSERVER_FAILED'
        except Exception as error:
            code = 1
            manifest.update(error=str(error), run_state='FAILED')
        finally:
            cleanup_errors = []
            for key, owned_module in (('observer', module), ('readiness', marker)):
                try:
                    manifest['terminated_owned_' + key + '_pids'] = stop_bound_controller(
                        container, owned_module, container_run)
                except Exception as error:
                    cleanup_errors.append(key + ': ' + str(error))
            try:
                if simulator.poll() is None:
                    subprocess.run([str(root / 'tools/stop_sim.sh')], check=True, timeout=25,
                                   stdout=subprocess.DEVNULL)
                simulator.wait(timeout=15)
            except Exception as error:
                cleanup_errors.append('simulator: ' + str(error))
            manifest['cleanup_errors'] = cleanup_errors
            if cleanup_errors:
                code = 1
                manifest['run_state'] = 'CLEANUP_FAILED'
            try:
                if code == 0 or (args.kind == 'stereo' and (run / 'stereo_summary.json').exists()):
                    with (run / 'audit.log').open('x') as audit_log:
                        audit_code = subprocess.call([str(root / 'tools/enter_env.sh'), 'env',
                            'OPENBLAS_NUM_THREADS=1', 'python3', '-m', audit_module,
                            container_run], stdout=audit_log, stderr=subprocess.STDOUT, timeout=15)
                    manifest['audit_exit_code'] = audit_code
                    code = code or audit_code
                    if audit_code and manifest['run_state'] == 'OBSERVED':
                        manifest['run_state'] = 'AUDIT_FAILED'
                after = verify_local_simulator(root)
                manifest['original_simulator_files_unchanged'] = before == after
            except Exception as error:
                code = 1
                manifest.update(run_state='AUDIT_FAILED', audit_error=str(error),
                                original_simulator_files_unchanged=False)
            manifest['source_sha256_after'] = source_hashes(root)
            manifest['source_unchanged'] = manifest['source_sha256_before'] == manifest['source_sha256_after']
            manifest['observer_exit_code'] = observer_code
            manifest['result_exit_code'] = code
            manifest['telemetry'] = ['lidar_frames.jsonl', 'lidar_summary.json',
                                     'lidar_first_message.json', 'lidar_first_message.bin',
                                     'lidar_endpoint_snapshot.json', 'lidar_audit.json']
            if args.kind == 'stereo':
                manifest['telemetry'] = ['stereo_pairs.jsonl', 'stereo_summary.json',
                    'stereo_first_left.json', 'stereo_first_left.bin', 'stereo_first_right.json',
                    'stereo_first_right.bin', 'stereo_first_pair.npz', 'stereo_pairing_audit.json',
                    'stereo_image_timing.jsonl']
            if args.kind == 'pose-history':
                manifest['telemetry'] = ['history_states.jsonl', 'camera_history.jsonl',
                                        'pose_history_summary.json', 'pose_history_audit.json']
            (run / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return code if manifest['source_unchanged'] and manifest['original_simulator_files_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
