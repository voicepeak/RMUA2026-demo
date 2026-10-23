"""Own a fresh uninstrumented scene for read-only online LiDAR observation."""
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
    args = parser.parse_args()
    if args.seed < 0 or not math.isfinite(args.duration) or not 0 < args.duration <= 30:
        parser.error('nonnegative seed and duration in (0,30] required')
    root = Path(__file__).resolve().parents[2]
    container = os.environ.get('RMUA_CONTAINER', 'rmua_noetic')
    digest = subprocess.check_output(['docker', 'inspect', container, '--format', '{{.Image}}'],
                                     text=True).strip()
    before = verify_local_simulator(root)
    config = json.loads((root / 'configs/official_baseline.yaml').read_text())
    config['experiment'] = {'kind': 'lidar_endpoints_read_only', 'duration_s': args.duration}
    run, manifest = prepare_run(root, root / 'runs', args.seed, digest, config)
    manifest.update(stage='P2_READ_ONLY_LIDAR_ENDPOINTS', acceptance_eligible=False,
                    simulator_preflight=before, run_state='PREPARED')
    container_run = '/workspace/main/' + str(run.relative_to(root))
    module = 'adapters.ros_noetic.lidar_observer'
    marker = 'rmua_lidar_sensor_readiness'
    code = 1
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
                        'from sensor_msgs.msg import PointCloud2; '
                        'sys.exit(2) if not rosgraph.is_master_online() else None; '
                        'rospy.init_node("lidar_ready",anonymous=True); '
                        'rospy.wait_for_message("/airsim_node/drone_1/lidar",PointCloud2,timeout=2)',
                        marker, '--run-dir', container_run], stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL, timeout=8)
                except subprocess.TimeoutExpired:
                    stop_bound_controller(container, marker, container_run)
                    continue
                if ready.returncode == 0:
                    break
            else:
                raise RuntimeError('lidar readiness timeout')
            manifest['run_state'] = 'OBSERVING'
            (run / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
            with (run / 'observer.log').open('x') as observer_log:
                code = subprocess.call([str(root / 'tools/enter_env.sh'), 'env',
                        'OPENBLAS_NUM_THREADS=1', 'python3', '-m', module, '--run-dir', container_run,
                        '--duration', str(args.duration)], stdout=observer_log,
                        stderr=subprocess.STDOUT, timeout=args.duration + 15)
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
                if code == 0:
                    with (run / 'audit.log').open('x') as audit_log:
                        code = subprocess.call([str(root / 'tools/enter_env.sh'), 'env',
                            'OPENBLAS_NUM_THREADS=1', 'python3', '-m', 'evaluation.metrics.lidar_observation',
                            container_run], stdout=audit_log, stderr=subprocess.STDOUT, timeout=15)
                    if code:
                        manifest['run_state'] = 'AUDIT_FAILED'
                after = verify_local_simulator(root)
                manifest['original_simulator_files_unchanged'] = before == after
            except Exception as error:
                code = 1
                manifest.update(run_state='AUDIT_FAILED', audit_error=str(error),
                                original_simulator_files_unchanged=False)
            manifest['source_sha256_after'] = source_hashes(root)
            manifest['source_unchanged'] = manifest['source_sha256_before'] == manifest['source_sha256_after']
            manifest['observer_exit_code'] = code
            manifest['telemetry'] = ['lidar_frames.jsonl', 'lidar_summary.json',
                                     'lidar_first_message.json', 'lidar_first_message.bin',
                                     'lidar_endpoint_snapshot.json', 'lidar_audit.json']
            (run / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return code if manifest['source_unchanged'] and manifest['original_simulator_files_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
