"""Own a fresh local scene, run bounded PWM experiment, always stop the scene."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from evaluation.seed_runner.manifest import prepare_run, source_hashes,file_hash
from evaluation.seed_runner.simulator_preflight import verify_local_simulator
from evaluation.seed_runner.controller_cleanup import stop_bound_controller


def cleanup_simulator(simulator, stop_script):
    """Report cleanup failures; terminating the local wrapper is not scene cleanup."""
    result = {'stop_and_launcher_completed': False}
    if simulator.poll() is not None:
        result['error'] = 'SIMULATOR_EXITED_BEFORE_CLEANUP'
        result['launcher_exit_code'] = simulator.returncode
        return result
    try:
        stopped = subprocess.run([str(stop_script)], timeout=15)
        result['stop_script_exit_code'] = stopped.returncode
        if stopped.returncode != 0:
            result['error'] = 'STOP_SCRIPT_FAILED'
        else:
            result['launcher_exit_code'] = simulator.wait(timeout=10)
            result['stop_and_launcher_completed'] = True
            return result
    except (OSError, subprocess.TimeoutExpired) as error:
        result['error'] = 'SIMULATOR_CLEANUP_ERROR: ' + str(error)
    # This terminates only our local docker-exec wrapper, not necessarily the
    # container process group. Never report that as verified scene cleanup.
    if simulator.poll() is None:
        try:
            simulator.terminate()
            result['launcher_exit_code'] = simulator.wait(timeout=5)
        except ProcessLookupError:
            result['launcher_exit_code'] = simulator.poll()
        except subprocess.TimeoutExpired:
            result['wrapper_wait_timeout'] = True
    return result


def evaluation_input_error(run, prefix, manifest):
    if manifest.get('truth_evaluator_exit_code') != 0:
        return 'TRUTH_RECORDER_INCOMPLETE'
    if not (run/(prefix+'_telemetry.jsonl')).is_file() or not (run/'evaluation_truth.jsonl').is_file():
        return 'EVALUATION_INPUT_MISSING'
    return None


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--levels', default='.10,.15,.20,.25,.30,.35,.40')
    parser.add_argument('--pulse-seconds', type=float, default=.3)
    parser.add_argument('--motor-pulses', action='store_true')
    parser.add_argument('--hover-seconds', type=float, default=0.)
    parser.add_argument('--control-hz', type=int, choices=(50, 80), default=80,
                        help='experimental PWM control loop rate; default remains 80 Hz')
    parser.add_argument('--evaluate-truth', action='store_true')
    parser.add_argument('--step', action='store_true')
    parser.add_argument('--mode', choices=('PWM', 'VEL'), default='PWM')
    parser.add_argument('--sensor-fault', choices=('GPS_ZERO','IMU_DROP','IMU_ROLLBACK','IMU_RATE_SPIKE'))
    parser.add_argument('--fault-seconds', type=float, default=.15)
    parser.add_argument('--gps-bridge',action='store_true')
    parser.add_argument('--yaw-motion',action='store_true')
    parser.add_argument('--braking',action='store_true')
    parser.add_argument('--snapshot',action='store_true')
    parser.add_argument('--scene-trace',action='store_true',help='isolated, bounded camera/LiDAR recording')
    parser.add_argument('--stationary-bootstrap',action='store_true',help='experimental resting initialization before PWM')
    parser.add_argument('--angular-recovery',action='store_true')
    parser.add_argument('--angular-recovery-profile',choices=('standard','fast'),default='standard')
    parser.add_argument('--angular-pulse',action='store_true')
    parser.add_argument('--angular-pulse-moment',type=float)
    parser.add_argument('--isolated-reference',action='store_true')
    parser.add_argument('--reference-stall-seconds',type=float,default=0.)
    parser.add_argument('--collective-pulse-pwm',type=float)
    parser.add_argument('--record-pwm-commands',action='store_true',help='opt in for a new observer-compatible standalone image')
    parser.add_argument('--second-wire-observer',action='store_true',help='additional independent diagnostic PWM subscriber')
    parser.add_argument('--bridge-probe',choices=('observe','serialize','lifetime','native-input','native-serialize'),help='instrumented simulator, never acceptance eligible')
    parser.add_argument('--controller-image',help='locally built immutable standalone image; no beta bind mount')
    parser.add_argument('--contract-audit-only',action='store_true',help='read-only service handshake; no controller')
    args = parser.parse_args()
    if args.second_wire_observer and (args.mode != 'PWM' or not args.evaluate_truth
                                      or args.controller_image or args.contract_audit_only):
        parser.error('second observer requires evaluated workspace PWM')
    if args.control_hz != 80 and (args.mode != 'PWM' or args.controller_image or args.contract_audit_only):
        parser.error('nondefault control rate requires the PWM workspace controller')
    if args.mode == 'VEL' and (args.motor_pulses or not 3 <= args.hover_seconds <= 20):
        parser.error('VEL requires hover-seconds [3,20] and no motor pulses')
    if args.sensor_fault and args.mode != 'PWM':
        parser.error('sensor injection currently available for PWM experiments only')
    if args.stationary_bootstrap and (args.mode!='PWM' or not args.hover_seconds):
        parser.error('stationary bootstrap requires bounded PWM hover')
    if args.angular_recovery and (args.mode!='PWM' or args.hover_seconds<10 or args.gps_bridge or args.sensor_fault):
        parser.error('angular recovery requires >=10s healthy-state PWM hover without GPS/fault modes')
    if args.angular_recovery_profile!='standard' and not args.angular_recovery:
        parser.error('fast recovery profile requires angular-recovery')
    if args.angular_pulse and (not args.angular_recovery or args.step or args.braking or args.yaw_motion):
        parser.error('angular pulse requires isolated angular-recovery hover')
    if args.angular_pulse_moment is not None and (not args.angular_pulse or not 0<args.angular_pulse_moment<=.8):
        parser.error('angular-pulse-moment requires angular-pulse, moment (0,.8] Nm')
    if args.isolated_reference and (args.mode!='PWM' or args.hover_seconds<10 or args.angular_recovery
                                   or args.gps_bridge or args.sensor_fault):
        parser.error('isolated reference requires >=10s healthy-state PWM hover without other fault modes')
    if args.reference_stall_seconds and (not args.isolated_reference or not .1<=args.reference_stall_seconds<=1.):
        parser.error('reference stall requires isolated reference and duration [.1,1]')
    if args.collective_pulse_pwm is not None and (args.mode!='PWM' or not 0<=args.collective_pulse_pwm<=1
            or args.hover_seconds<10 or args.step or args.braking or args.yaw_motion or args.sensor_fault
            or args.gps_bridge or args.angular_recovery or args.isolated_reference):
        parser.error('collective pulse requires isolated >=10s healthy-state PWM hover, input in [0,1]')
    if args.record_pwm_commands and (not args.evaluate_truth or args.mode!='PWM'):
        parser.error('record-pwm-commands requires evaluated PWM experiment')
    if args.bridge_probe and (args.mode!='PWM' or not args.evaluate_truth or args.hover_seconds<10
            or args.controller_image or args.sensor_fault or args.gps_bridge or args.angular_recovery
            or args.angular_pulse or args.isolated_reference or args.collective_pulse_pwm is not None):
        parser.error('bridge probe requires evaluated healthy-state workspace PWM hover without other fault modes')
    if args.gps_bridge and (args.mode != 'PWM' or args.hover_seconds < 3):
        parser.error('GPS bridge is only available in bounded PWM hover')
    if (args.braking or args.yaw_motion or args.step) and args.hover_seconds < 10:
        parser.error('response identification requires hover-seconds >=10')
    if args.braking and args.step:
        parser.error('braking and step are separate experiments')
    if args.contract_audit_only and (args.controller_image or args.hover_seconds or args.motor_pulses
            or args.step or args.sensor_fault or args.gps_bridge or args.yaw_motion or args.braking
            or args.evaluate_truth or args.snapshot or args.scene_trace or args.stationary_bootstrap
            or args.angular_recovery or args.angular_pulse or args.isolated_reference or args.reference_stall_seconds
            or args.collective_pulse_pwm is not None or args.record_pwm_commands or args.bridge_probe):
        parser.error('contract-audit-only cannot be combined with flight/recording modes')
    container = os.environ.get('RMUA_CONTAINER', 'rmua_noetic')
    simulator_digest = subprocess.check_output(['docker', 'inspect', container, '--format', '{{.Image}}'], text=True).strip()
    image_info=None
    if args.controller_image:
        image_info=json.loads(subprocess.check_output(['docker','image','inspect',args.controller_image],text=True))[0]
    digest=image_info['Id'] if image_info else simulator_digest
    record_commands=args.mode=='PWM' and (not image_info or args.record_pwm_commands)
    simulator_evidence=verify_local_simulator(root)
    probe_path=None
    if args.bridge_probe:
        probe_variant=('native-input' if args.bridge_probe=='native-serialize' else
                       args.bridge_probe if args.bridge_probe in ('lifetime','native-input') else 'concurrency')
        probe_path=Path(subprocess.check_output(['bash',str(root/'tools/build_bridge_probe.sh'),probe_variant],cwd=root,text=True).strip()).resolve()
        if not probe_path.is_file() or root/'runs/probe_builds' not in probe_path.parents:
            raise ValueError('invalid owned probe build path')
    config = json.loads((root/'configs/official_baseline.yaml').read_text())
    config['control_mode'] = args.mode
    config['experiment'] = {'levels': args.levels, 'pulse_seconds': args.pulse_seconds,
                            'control_hz': args.control_hz,
                            'motor_pulses': args.motor_pulses, 'hover_seconds': args.hover_seconds,
                            'evaluate_truth': args.evaluate_truth, 'step': args.step,
                            'sensor_fault':args.sensor_fault, 'fault_seconds':args.fault_seconds,
                            'gps_bridge':args.gps_bridge,'yaw_motion':args.yaw_motion,'braking':args.braking,
                            'snapshot':args.snapshot,'scene_trace':args.scene_trace,'controller_image':args.controller_image,
                            'contract_audit_only':args.contract_audit_only,
                            'stationary_bootstrap':args.stationary_bootstrap,
                            'angular_recovery':args.angular_recovery,'angular_pulse':args.angular_pulse,
                            'angular_recovery_profile':args.angular_recovery_profile,
                            'angular_pulse_moment':args.angular_pulse_moment if args.angular_pulse_moment is not None else (.5 if args.angular_pulse else None),
                            'isolated_reference':args.isolated_reference,
                            'reference_stall_seconds':args.reference_stall_seconds,
                            'collective_pulse_pwm':args.collective_pulse_pwm,
                            'record_pwm_commands':record_commands if args.evaluate_truth else False,
                            'second_wire_observer':args.second_wire_observer,
                            'bridge_probe':args.bridge_probe}
    run, manifest = prepare_run(root, root/'runs', args.seed, digest, config)
    manifest['stage'] = ('P1_PWM_CLOSED_LOOP' if args.hover_seconds else 'P0_PWM_IDENTIFICATION') if args.mode == 'PWM' else 'P1_VEL_BENCHMARK'
    if args.contract_audit_only:manifest['stage']='P0_READ_ONLY_SERVICE_CONTRACT'
    if probe_path:
        manifest['stage']='P0_INSTRUMENTED_BRIDGE_DIAGNOSIS'
        manifest['acceptance_eligible']=False
        manifest['simulator_runtime_instrumentation']={'mode':args.bridge_probe,
            'build_variant':probe_variant,
            'source_sha256':file_hash(root/'tools/diagnostics/bridge_probe.cpp'),
            'binary_sha256':file_hash(probe_path),'original_simulator_files_modified':False}
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
    second_wire = None
    second_wire_log = None
    scene_recorder=None
    scene_log=None
    controller_name='rmua-next-flight-'+run.name
    module='adapters.ros_noetic.pwm_experiment' if args.mode == 'PWM' else 'adapters.ros_noetic.vel_experiment'
    if args.contract_audit_only:module='adapters.ros_noetic.service_contract_audit'
    readiness_marker='rmua_next_sensor_readiness'
    container_run='/workspace/main/'+str(run.relative_to(root))
    simulator_command=[str(root/'tools/run_sim.sh'),str(args.seed),'offscreen']
    simulator_environment=os.environ.copy()
    if probe_path:
        simulator_command.append('diagnostic-bridge-probe')
        simulator_environment.update({'RMUA_BRIDGE_PROBE_SO':'/workspace/main/'+str(probe_path.relative_to(root)),
            'RMUA_BRIDGE_PROBE_MODE':args.bridge_probe,'RMUA_BRIDGE_PROBE_OUTPUT':container_run+'/bridge_probe.jsonl'})
    with (run/'simulator.log').open('x') as output:
        simulator = subprocess.Popen(simulator_command,env=simulator_environment,
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
                evaluation_command=[str(root/'tools/enter_env.sh'), 'python3', '-m',
                    'evaluation.metrics.truth_recorder', '--run-dir',
                    '/workspace/main/'+str(run.relative_to(root))]
                if record_commands:evaluation_command.append('--record-pwm-commands')
                evaluator = subprocess.Popen(evaluation_command, stdout=evaluation_log, stderr=subprocess.STDOUT)
                ready_deadline = time.monotonic()+5
                while not (run/'truth_ready').exists() and time.monotonic() < ready_deadline:
                    if evaluator.poll() is not None:
                        break
                    time.sleep(.05)
                if not (run/'truth_ready').exists():
                    raise RuntimeError('hidden truth evaluator failed to receive truth')
            if args.second_wire_observer:
                second_wire_log = (run/'second_wire.log').open('x')
                second_wire = subprocess.Popen([str(root/'tools/enter_env.sh'), 'python3', '-m',
                    'evaluation.metrics.second_wire_recorder', '--run-dir', container_run],
                    stdout=second_wire_log, stderr=subprocess.STDOUT)
                deadline = time.monotonic()+5
                while not (run/'second_wire_ready').exists() and time.monotonic() < deadline:
                    if second_wire.poll() is not None: break
                    time.sleep(.05)
                if not (run/'second_wire_ready').exists():
                    raise RuntimeError('second wire observer did not start')
            if args.scene_trace:
                scene_log=(run/'scene_recorder.log').open('x')
                scene_recorder=subprocess.Popen([str(root/'tools/enter_env.sh'),'env','OPENBLAS_NUM_THREADS=1',
                    'python3','-m','adapters.ros_noetic.scene_trace','--run-dir',container_run],
                    stdout=scene_log,stderr=subprocess.STDOUT)
                ready_deadline=time.monotonic()+8
                while not (run/'scene_ready').exists() and time.monotonic()<ready_deadline:
                    if scene_recorder.poll() is not None:break
                    time.sleep(.05)
                if not (run/'scene_ready').exists():raise RuntimeError('scene recorder not ready')
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
                if args.control_hz != 80:
                    command.extend(['--control-hz', str(args.control_hz)])
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
            if args.stationary_bootstrap:command.append('--stationary-bootstrap')
            if args.angular_recovery:command.append('--angular-recovery')
            if args.angular_recovery_profile!='standard':
                command.extend(['--angular-recovery-profile',args.angular_recovery_profile])
            if args.angular_pulse:command.append('--angular-pulse')
            if args.angular_pulse_moment is not None:command.extend(['--angular-pulse-moment',str(args.angular_pulse_moment)])
            if args.isolated_reference:command.append('--isolated-reference')
            if args.reference_stall_seconds:command.extend(['--reference-stall-seconds',str(args.reference_stall_seconds)])
            if args.collective_pulse_pwm is not None:command.extend(['--collective-pulse-pwm',str(args.collective_pulse_pwm)])
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
            if scene_recorder:
                (run/'scene_stop').touch()
                try:
                    manifest['scene_recorder_exit_code']=scene_recorder.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    manifest['scene_recorder_exit_code']='TIMEOUT'
                    stop_bound_controller(container,'adapters.ros_noetic.scene_trace',container_run)
                    scene_recorder.terminate()
                scene_log.close()
            if evaluator:
                (run/'truth_stop').touch()
                try:
                    manifest['truth_evaluator_exit_code'] = evaluator.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    evaluator.terminate()
                    manifest['truth_evaluator_exit_code'] = 'TIMEOUT'
                evaluation_log.close()
            if second_wire:
                (run/'second_wire_stop').touch()
                try:
                    manifest['second_wire_exit_code'] = second_wire.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    manifest['second_wire_exit_code'] = 'TIMEOUT'
                    try:
                        manifest['terminated_owned_second_wire_pids'] = stop_bound_controller(
                            container, 'evaluation.metrics.second_wire_recorder', container_run)
                    except Exception as error:
                        manifest['second_wire_cleanup_error'] = str(error)
                    if second_wire.poll() is None:
                        try:
                            second_wire.terminate()
                        except ProcessLookupError:
                            pass
                if manifest['second_wire_exit_code'] != 0:
                    code = 1
                second_wire_log.close()
            # Only stop a process group launched by this invocation.
            manifest['simulator_cleanup'] = cleanup_simulator(simulator, root/'tools/stop_sim.sh')
            if not manifest['simulator_cleanup']['stop_and_launcher_completed']:
                code = 1
            manifest['source_sha256_after'] = source_hashes(root)
            if probe_path:
                try:
                    after=verify_local_simulator(root)
                    manifest['original_simulator_files_unchanged']=after==simulator_evidence
                    probe_file=run/'bridge_probe.jsonl'
                    observations=[json.loads(line) for line in probe_file.read_text().splitlines()]
                    manifest['bridge_probe_summary']={key:max(r[key] for r in observations)
                        for key in ('pwm_calls','bridge_calls','overlap_entries','write_errors')}
                    manifest['bridge_probe_summary']['complete_exit_record']=any(r['event']=='exit' for r in observations)
                    if args.bridge_probe in ('lifetime','native-input','native-serialize'):
                        required=('parameter_borrow','command_constructed','command_handoff',
                                  'parameter_free_begin','parameter_free_end','bridge_begin','bridge_end')
                        counts={event:sum(r['event']==event for r in observations) for event in required}
                        manifest['bridge_lifetime_event_counts']=counts
                        if min(counts.values())==0:raise RuntimeError('lifetime hook coverage missing')
                        from evaluation.metrics.bridge_lifetime import evaluate as evaluate_lifetime
                        lifetime=evaluate_lifetime(observations)
                        (run/'bridge_lifetime_audit.json').write_text(json.dumps(lifetime,indent=2)+'\n')
                        if not lifetime['hook_coverage_valid']:raise RuntimeError('lifetime handoff coverage missing')
                    if args.bridge_probe in ('native-input','native-serialize'):
                        from evaluation.metrics.native_input import evaluate as evaluate_native_input
                        sender=[json.loads(x) for x in (run/'pwm_telemetry.jsonl').read_text().splitlines()]
                        controller_summary=json.loads((run/'pwm_summary.json').read_text())
                        native=evaluate_native_input(observations,sender,controller_summary.get('control_end_wall_monotonic'))
                        (run/'native_input_audit.json').write_text(json.dumps(native,indent=2)+'\n')
                        if not native['snapshot_coverage_valid']:
                            raise RuntimeError('native input snapshot coverage incomplete')
                    if (not manifest['original_simulator_files_unchanged'] or
                            min(manifest['bridge_probe_summary'][key] for key in ('pwm_calls','bridge_calls'))==0
                            or manifest['bridge_probe_summary']['write_errors']
                            or any(r['mode']!=args.bridge_probe for r in observations)):
                        raise RuntimeError('probe inactive or original simulator files changed')
                except Exception as error:
                    manifest['bridge_probe_error']=str(error);code=2
            manifest['source_unchanged'] = manifest['source_sha256_before'] == manifest['source_sha256_after']
            manifest['observer_exit_code'] = code
            manifest['run_state']=('AUDIT_COMPLETE' if args.contract_audit_only else 'CONTROL_COMPLETE') if code==0 else 'FAILED'
            prefix = 'pwm' if args.mode == 'PWM' else 'vel'
            manifest['telemetry'] = [prefix+'_telemetry.jsonl', prefix+'_summary.json', 'simulator.log']
            if args.contract_audit_only:manifest['telemetry']=['service_contract_audit.json','simulator.log']
            if args.evaluate_truth:
                manifest['evaluation_only_inputs'] = ['/airsim_node/drone_1/debug/pose_gt',
                                                     '/airsim_node/drone_1/debug/rotor_pwm',
                                                     '/airsim_node/drone_1/debug/wind',
                                                     '/airsim_node/drone_1/imu/imu',
                                                     '/airsim_node/drone_1/gps','/rosout_agg']
                manifest['telemetry'].extend(['evaluation_truth.jsonl','evaluation_actuation.jsonl',
                                             'evaluation_wind.jsonl','evaluation_imu.jsonl',
                                             'evaluation_gps.jsonl','evaluation_rosout.jsonl'])
                if record_commands:
                    manifest['evaluation_only_inputs'].append('/airsim_node/drone_1/rotor_pwm_cmd')
                    manifest['telemetry'].append('evaluation_pwm_commands.jsonl')
                if args.second_wire_observer:
                    manifest['telemetry'].append('second_wire_commands.jsonl')
            manifest['artifacts']=sorted(p.name for p in run.iterdir() if p.is_file() and p.name!='manifest.json')
            (run/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    if args.evaluate_truth:
        input_error = evaluation_input_error(run, prefix, manifest)
        if input_error:
            manifest['evaluation_input_error'] = input_error
            if code == 0:
                code = 1
            manifest['run_state'] = 'FAILED'
            (run/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    if args.evaluate_truth and (run/(prefix+'_telemetry.jsonl')).exists() and (run/'evaluation_truth.jsonl').exists():
        metric_code = subprocess.call(['python3', '-m', 'evaluation.metrics.closed_loop', str(run)], cwd=root)
        manifest['evaluation_exit_code']=metric_code
        if code == 0:
            code = metric_code
        if args.mode=='PWM':
            manifest['physics_diagnostic_exit_code']=subprocess.call(
                ['python3','-m','evaluation.metrics.physics_residual',str(run)],cwd=root,
                stdout=subprocess.DEVNULL)
            if record_commands:
                manifest['command_trace_audit_exit_code']=subprocess.call(
                    ['python3','-m','evaluation.metrics.command_trace',str(run)],cwd=root,
                    stdout=subprocess.DEVNULL)
                if code==0 and manifest['command_trace_audit_exit_code']!=0:
                    code=manifest['command_trace_audit_exit_code']
            if args.second_wire_observer and (run/'second_wire_commands.jsonl').exists():
                from evaluation.metrics.command_trace import evaluate as evaluate_trace, compare_observers
                def rows(name):
                    return [json.loads(line) for line in (run/name).read_text().splitlines()]
                second = evaluate_trace(rows('pwm_telemetry.jsonl'), rows('second_wire_commands.jsonl'), [])
                (run/'second_wire_audit.json').write_text(json.dumps(second, indent=2)+'\n')
                manifest['second_wire_audit_valid'] = second['wire_trace_valid']
                primary_path = run/'command_trace_audit.json'
                if primary_path.exists():
                    primary = json.loads(primary_path.read_text())
                    comparison = compare_observers(primary, second)
                    (run/'wire_observer_comparison.json').write_text(json.dumps(comparison, indent=2)+'\n')
                else:
                    manifest['second_wire_comparison_error'] = 'PRIMARY_AUDIT_MISSING'
                    code = 1
                if code == 0 and not second['wire_trace_valid']:
                    code = 2
            elif args.second_wire_observer:
                manifest['second_wire_comparison_error'] = 'SECOND_WIRE_RECORD_MISSING'
                code = 1
        manifest['run_state']='EVALUATED' if code==0 else 'FAILED'
        manifest['artifacts']=sorted(p.name for p in run.iterdir() if p.is_file() and p.name!='manifest.json')
        (run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
