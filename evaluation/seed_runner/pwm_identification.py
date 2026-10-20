"""Own a fresh local scene, run bounded PWM experiment, always stop the scene."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from evaluation.seed_runner.manifest import prepare_run, source_hashes
from evaluation.seed_runner.simulator_preflight import verify_local_simulator
from evaluation.seed_runner.controller_cleanup import stop_bound_controller


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
    parser.add_argument('--yaw-motion',action='store_true')
    parser.add_argument('--braking',action='store_true')
    parser.add_argument('--snapshot',action='store_true')
    parser.add_argument('--controller-image',help='locally built immutable standalone image; no beta bind mount')
    parser.add_argument('--contract-audit-only',action='store_true',help='read-only service handshake; no controller')
    args = parser.parse_args()
    if args.mode == 'VEL' and (args.motor_pulses or not 3 <= args.hover_seconds <= 20):
        parser.error('VEL requires hover-seconds [3,20] and no motor pulses')
    if args.sensor_fault and args.mode != 'PWM':
        parser.error('sensor injection currently available for PWM experiments only')
    if args.gps_bridge and (args.mode != 'PWM' or args.hover_seconds < 3):
        parser.error('GPS bridge is only available in bounded PWM hover')
    if (args.braking or args.yaw_motion or args.step) and args.hover_seconds < 10:
        parser.error('response identification requires hover-seconds >=10')
    if args.braking and args.step:
        parser.error('braking and step are separate experiments')
    if args.contract_audit_only and (args.controller_image or args.hover_seconds or args.motor_pulses
            or args.step or args.sensor_fault or args.gps_bridge or args.yaw_motion or args.braking
            or args.evaluate_truth or args.snapshot):
        parser.error('contract-audit-only cannot be combined with flight/recording modes')
    container = os.environ.get('RMUA_CONTAINER', 'rmua_noetic')
    simulator_digest = subprocess.check_output(['docker', 'inspect', container, '--format', '{{.Image}}'], text=True).strip()
    image_info=None
    if args.controller_image:
        image_info=json.loads(subprocess.check_output(['docker','image','inspect',args.controller_image],text=True))[0]
    digest=image_info['Id'] if image_info else simulator_digest
    simulator_evidence=verify_local_simulator(root)
    config = json.loads((root/'configs/official_baseline.yaml').read_text())
    config['control_mode'] = args.mode
    config['experiment'] = {'levels': args.levels, 'pulse_seconds': args.pulse_seconds,
                            'motor_pulses': args.motor_pulses, 'hover_seconds': args.hover_seconds,
                            'evaluate_truth': args.evaluate_truth, 'step': args.step,
                            'sensor_fault':args.sensor_fault, 'fault_seconds':args.fault_seconds,
                            'gps_bridge':args.gps_bridge,'yaw_motion':args.yaw_motion,'braking':args.braking,
                            'snapshot':args.snapshot,'controller_image':args.controller_image,
                            'contract_audit_only':args.contract_audit_only}
    run, manifest = prepare_run(root, root/'runs', args.seed, digest, config)
    manifest['stage'] = ('P1_PWM_CLOSED_LOOP' if args.hover_seconds else 'P0_PWM_IDENTIFICATION') if args.mode == 'PWM' else 'P1_VEL_BENCHMARK'
    if args.contract_audit_only:manifest['stage']='P0_READ_ONLY_SERVICE_CONTRACT'
    manifest['simulator_preflight']=simulator_evidence
    manifest['simulator_container_image_digest']=simulator_digest
    manifest['controller_source_kind']='immutable_docker_image' if image_info else 'workspace_bind_mount'
    if args.contract_audit_only:manifest['controller_source_kind']='none_read_only_observer'
    if image_info:
        manifest['controller_git_revision']=(image_info['Config'].get('Labels') or {}).get('org.opencontainers.image.revision','unknown')
    manifest['run_state']='PREPARED'
    (run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(run, flush=True)
    code = 1
    evaluator = None
    evaluation_log = None
    controller_name='rmua-next-flight-'+run.name
    module='adapters.ros_noetic.pwm_experiment' if args.mode == 'PWM' else 'adapters.ros_noetic.vel_experiment'
    if args.contract_audit_only:module='adapters.ros_noetic.service_contract_audit'
    readiness_marker='rmua_next_sensor_readiness'
    container_run='/workspace/main/'+str(run.relative_to(root))
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
                        'import socket,sys; socket.setdefaulttimeout(2.); import rosgraph,rospy; from sensor_msgs.msg import Imu; '
                        'sys.exit(2) if not rosgraph.is_master_online() else None; '
                        'rospy.init_node("pwm_ready", anonymous=True); rospy.wait_for_message("/airsim_node/drone_1/imu/imu", Imu, timeout=2)',
                        readiness_marker,'--run-dir',container_run],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=8)
                except subprocess.TimeoutExpired:
                    stop_bound_controller(container,readiness_marker,container_run)
                    continue
                if probe.returncode == 0:
                    break
            else:
                raise RuntimeError('simulator sensor readiness timeout')
            if args.snapshot:
                subprocess.run([str(root/'tools/enter_env.sh'),'env','OPENBLAS_NUM_THREADS=1',
                    'python3','-m','adapters.ros_noetic.scene_snapshot','--run-dir',
                    '/workspace/main/'+str(run.relative_to(root))],check=True,timeout=25)
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
            if image_info:
                (run/'ros_home').mkdir()
                command=['docker','run','--rm','--name',controller_name,'--network','host',
                         '--user',str(os.getuid())+':'+str(os.getgid()),'-e','ROS_HOME=/rmua-run/ros_home',
                         '--mount','type=bind,source='+str(run)+',target=/rmua-run',digest,
                         'python3','-m',module,'--run-dir','/rmua-run']
            else:
                command = [str(root/'tools/enter_env.sh'), 'env', 'OPENBLAS_NUM_THREADS=1',
                        'python3','-m',module,'--run-dir','/workspace/main/'+str(run.relative_to(root))]
            if args.mode == 'PWM' and not args.contract_audit_only:
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
            if args.yaw_motion:
                command.append('--yaw-motion')
            if args.braking:
                command.append('--braking')
            manifest['controller_command']=command
            manifest['run_state']='AUDIT_RUNNING' if args.contract_audit_only else 'CONTROL_RUNNING'
            (run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
            code = subprocess.call(command, timeout=40)
        except Exception as error:
            manifest['error'] = str(error)
            print(str(error), flush=True)
        finally:
            try:
                manifest['terminated_owned_readiness_pids']=stop_bound_controller(container,readiness_marker,container_run)
            except Exception as error:
                manifest['readiness_cleanup_error']=str(error)
                code=1
            if image_info:
                # The UUID name belongs exclusively to this host invocation.
                subprocess.run(['docker','rm','-f',controller_name],stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL,timeout=10)
            else:
                try:
                    manifest['terminated_owned_controller_pids']=stop_bound_controller(container,module,
                                        container_run)
                except Exception as error:
                    manifest['controller_cleanup_error']=str(error)
                    code=1
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
            manifest['run_state']=('AUDIT_COMPLETE' if args.contract_audit_only else 'CONTROL_COMPLETE') if code==0 else 'FAILED'
            prefix = 'pwm' if args.mode == 'PWM' else 'vel'
            manifest['telemetry'] = [prefix+'_telemetry.jsonl', prefix+'_summary.json', 'simulator.log']
            if args.contract_audit_only:manifest['telemetry']=['service_contract_audit.json','simulator.log']
            if args.evaluate_truth:
                manifest['evaluation_only_inputs'] = ['/airsim_node/drone_1/debug/pose_gt',
                                                     '/airsim_node/drone_1/debug/rotor_pwm']
                manifest['telemetry'].extend(['evaluation_truth.jsonl','evaluation_actuation.jsonl'])
            manifest['artifacts']=sorted(p.name for p in run.iterdir() if p.is_file() and p.name!='manifest.json')
            (run/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    if args.evaluate_truth and (run/(prefix+'_telemetry.jsonl')).exists() and (run/'evaluation_truth.jsonl').exists():
        metric_code = subprocess.call(['python3', '-m', 'evaluation.metrics.closed_loop', str(run)], cwd=root)
        manifest['evaluation_exit_code']=metric_code
        if code == 0:
            code = metric_code
        manifest['run_state']='EVALUATED' if code==0 else 'FAILED'
        manifest['artifacts']=sorted(p.name for p in run.iterdir() if p.is_file() and p.name!='manifest.json')
        (run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
