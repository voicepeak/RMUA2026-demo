"""Bounded local simulator identification/hover. Host MUST stop scene on return.

No zero-PWM hover fallback. No VEL, debug input, task service or competition mode.
These are development experiments, not an accepted competition controller.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
import numpy as np
from control.control_authority.lease import ControlLease, check_graph
from control.pwm_publisher.guard import CommandGuard, PwmCommand
from estimation.geometry import rotation
from interfaces.contracts import Health
from adapters.ros_noetic.flight_sensors import FlightSensors
from control.motor_allocator.allocator import MotorAllocator,Allocation
from control.se3_controller.controller import Se3Controller
from control.reference import benchmark_reference,benchmark_yaw
from adapters.ros_noetic.sensor_fault import LocalSensorFault
from control.control_authority.graph_monitor import GraphMonitor
from control.safety_supervisor.hover_recovery import HoverRecovery
from control.safety_supervisor.angular_recovery import AngularRecovery
from control.reference_worker import BenchmarkSpec,ReferenceWorker
from control.safety_supervisor.reference_hold import ReferenceHold
from control.collective_identification import CollectivePulse
from adapters.ros_noetic.control_connections import expected_control_connections


def motor_pattern():
    # Lift gently, then probe one motor at a time with very small positive pulses.
    pattern = [(.20,)*4]*4 + [(.176,)*4]*2
    for motor in range(4):
        values = [.176]*4
        values[motor] += .003
        pattern.append(tuple(values))
        pattern.append((.176,)*4)
    return tuple(pattern)


def validate_angular_pulse_outcome(sent, phase):
    if not sent:
        raise RuntimeError('ANGULAR_EXCITATION_NOT_SENT')
    if phase == 'NORMAL':
        raise RuntimeError('ANGULAR_RECOVERY_NOT_TRIGGERED')
    if phase != 'RECOVERY_HOLD':
        raise RuntimeError('ANGULAR_RECOVERY_NOT_SETTLED')


def run_experiment(run, levels, pulse_seconds, motor_pulses=False, hover_seconds=0, step=False,
                   sensor_fault=None, fault_seconds=.15, gps_bridge=False, yaw_motion=False,braking=False,
                   stationary_bootstrap=False,angular_recovery=False,angular_pulse=False,
                   isolated_reference=False,reference_stall_seconds=0.,collective_pulse_pwm=None,
                   angular_pulse_moment=.5,angular_recovery_profile='standard',control_hz=80,
                   control_stall_seconds=0.,control_stall_stage='before-check'):
    if type(control_hz) is not int or control_hz not in (50, 80):
        raise ValueError('experimental control rate must be 50 or 80 Hz')
    if control_stall_seconds not in (0., .15):
        raise ValueError('experimental control stall must be 0 or 0.15 s')
    if control_stall_stage not in ('before-check', 'before-publish') or (control_stall_stage != 'before-check' and not control_stall_seconds):
        raise ValueError('invalid control stall stage')
    if control_stall_seconds and (hover_seconds < 10 or motor_pulses or sensor_fault or gps_bridge
            or step or yaw_motion or braking or angular_recovery or angular_pulse
            or isolated_reference or collective_pulse_pwm is not None):
        raise ValueError('control stall requires isolated bounded hover')
    import rospy
    import rosgraph
    from airsim_ros.msg import RotorPWM
    rospy.init_node('rmua_next_pwm_identification', anonymous=True, disable_signals=True)
    master = rosgraph.Master(rospy.get_name())
    scene_id = rospy.get_param('/run_id')
    rows, fault = [], None
    latest_state,latest_raw,latest_elapsed=None,None,None
    fresh_read=False
    fault_observation=None
    last_published_wall=None
    control_stall = None
    guard_observation = None
    def graph(require_self):
        pubs, _, _ = master.getSystemState()
        check_graph(dict(pubs), rospy.get_name(), require_self=require_self)

    with ControlLease(scene_id, 'PWM'):
        graph(False)
        sensors = FlightSensors(track_startup_velocity=True, allow_stationary_bootstrap=stationary_bootstrap)
        publisher = None
        monitor = None
        recovery = HoverRecovery()
        angular_supervisor=AngularRecovery(angular_recovery_profile)
        pulse_sent=False
        reference_worker=None
        reference_hold=ReferenceHold()
        collective_pulse=CollectivePulse(collective_pulse_pwm) if collective_pulse_pwm is not None else None
        try:
            deadline = time.monotonic()+5
            while time.monotonic() < deadline:
                try:
                    state, raw = sensors.read()
                    bootstrap=None
                    if state.health == Health.OK and stationary_bootstrap:
                        bootstrap=sensors.bootstrap_at_rest_before_control()
                    if state.health == Health.OK and (not stationary_bootstrap or bootstrap is not None):
                        if bootstrap is not None:
                            (run/'stationary_bootstrap.json').write_text(json.dumps(bootstrap,indent=2)+'\n')
                            state,raw=sensors.read()
                        break
                except (ValueError, RuntimeError):
                    pass
                time.sleep(.01)
            else:
                raise RuntimeError('NO_HEALTHY_STATE')
            origin = np.array(state.position_ned)
            sensors.seal_precontrol()
            publisher = rospy.Publisher('/airsim_node/drone_1/rotor_pwm_cmd', RotorPWM, queue_size=1)
            expected_connections,backend_nodes=expected_control_connections(master.getSystemState(),
                                               '/airsim_node/drone_1/rotor_pwm_cmd')
            deadline = time.monotonic()+2
            while publisher.get_num_connections() < expected_connections and time.monotonic() < deadline:
                time.sleep(.01)
            if publisher.get_num_connections() < expected_connections:
                raise RuntimeError('PWM_SUBSCRIBER_COUNT')
            graph(True)
            _, subscribers, _ = master.getSystemState()
            runtime_inputs = sorted(topic for topic, nodes in subscribers if rospy.get_name() in nodes)
            official_config = json.loads((Path(__file__).resolve().parents[2]/'configs/official_baseline.yaml').read_text())
            if any(topic not in official_config['sensors'] for topic in runtime_inputs):
                raise RuntimeError('INPUT_WHITELIST_POLICY')
            (run/'runtime_input_audit.json').write_text(json.dumps({'node': rospy.get_name(),
                'topics': runtime_inputs, 'debug_inputs': False,'expected_control_connections':expected_connections,
                'control_connections':publisher.get_num_connections(),'backend_subscriber_nodes':backend_nodes}, indent=2)+'\n')
            monitor = GraphMonitor(lambda: graph(True))
            monitor.start()
            guard = CommandGuard(sensors.clock.epoch, minimum_period=.0105)
            pattern = motor_pattern() if motor_pulses else tuple((v,)*4 for v in levels)
            config = json.loads((Path(__file__).resolve().parents[2]/'configs/vehicle_identification.json').read_text())
            (run/'vehicle_model.json').write_text(json.dumps(config,indent=2)+'\n')
            allocator, controller = MotorAllocator(config), Se3Controller(config)
            previous_saturated = False
            yaw_reference = math.atan2(rotation(state.orientation_wxyz)[1, 0],
                                       rotation(state.orientation_wxyz)[0, 0])
            state, raw = sensors.read()
            if isolated_reference:
                reference_worker=ReferenceWorker(BenchmarkSpec(tuple(origin),yaw_reference,step,braking,yaw_motion),
                                                  stall_seconds=reference_stall_seconds)
                reference_worker.start()
                deadline=time.monotonic()+3.
                while time.monotonic()<deadline:
                    state,raw=sensors.read()
                    reference_worker.submit(state.metadata.stamp,state.metadata.epoch,0.)
                    packet,reason=reference_worker.snapshot(state.metadata.stamp,state.metadata.epoch)
                    if packet is not None:break
                    time.sleep(.005)
                else:raise RuntimeError('REFERENCE_WORKER_STARTUP')
            start = time.monotonic()
            state, raw = sensors.read()
            sensors.arm_rate_guard(recovery=angular_recovery)
            sensors.seal_startup_velocity()
            start_stamp = state.metadata.stamp
            if sensor_fault:
                sensors.local_injection = LocalSensorFault(sensor_fault, start_stamp+6., fault_seconds)
            next_tick = start
            previous_control_stamp = start_stamp-.01
            sequence = 0
            while True:
                now = time.monotonic()
                index = int((now-start)/pulse_seconds)
                if (hover_seconds and now-start >= hover_seconds) or (not hover_seconds and index >= len(pattern)):
                    break
                fresh_read=False
                state, raw = sensors.read()
                latest_state,latest_raw,latest_elapsed=state,raw,now-start
                fresh_read=True
                if (control_stall_seconds and control_stall_stage == 'before-check' and control_stall is None
                        and state.metadata.stamp-start_stamp >= 6.):
                    stall_begin = time.monotonic()
                    time.sleep(control_stall_seconds)
                    control_stall = {'begin_wall_monotonic': stall_begin,
                                     'end_wall_monotonic': time.monotonic(),
                                     'source_stamp': state.metadata.stamp, 'stage': control_stall_stage}
                R = rotation(state.orientation_wxyz)
                if state.health != Health.OK and not (hover_seconds and gps_bridge):
                    raise RuntimeError('STATE_HEALTH_'+state.health.value)
                angular_decision=angular_supervisor.decide(state) if angular_recovery else None
                if not angular_recovery and R[2, 2] < math.cos(.25):
                    raise RuntimeError('TILT_LIMIT')
                if np.linalg.norm(state.velocity_ned) > 4:
                    raise RuntimeError('SPEED_LIMIT')
                if not angular_recovery and np.linalg.norm(state.angular_velocity_body) > 1.:
                    raise RuntimeError('ANGULAR_RATE_LIMIT')
                if np.linalg.norm(np.array(state.position_ned)-origin) > (1.8 if hover_seconds else 1.2):
                    raise RuntimeError('POSITION_LIMIT')
                monitor.check()
                details = {}
                if hover_seconds:
                    sim_elapsed = state.metadata.stamp-start_stamp
                    if reference_worker is None:
                        reference, velocity, acceleration = benchmark_reference(origin, sim_elapsed, step,braking)
                        yaw_command,yaw_rate = benchmark_yaw(yaw_reference,sim_elapsed,yaw_motion)
                    allow_integral = not previous_saturated
                    phase = 'NORMAL'
                    rate_gain_scale=1.
                    attitude_gain_scale=1.
                    if reference_worker is not None:
                        reference_worker.submit(state.metadata.stamp,state.metadata.epoch,sim_elapsed)
                        packet,reason=reference_worker.snapshot(state.metadata.stamp,state.metadata.epoch)
                        selected=reference_hold.select(state,packet,reason)
                        reference,velocity,acceleration=selected.position,selected.velocity,selected.acceleration
                        yaw_command,yaw_rate=selected.yaw,selected.yaw_rate
                        phase=selected.phase
                        allow_integral=allow_integral and selected.allow_integral
                    if gps_bridge:
                        gps_stamp = raw['last_gps_stamp']
                        gps_age = math.inf if gps_stamp is None else state.metadata.stamp-gps_stamp
                        decision = recovery.decide(state,gps_age,(reference,velocity,acceleration))
                        reference,velocity,acceleration = decision.position,decision.velocity,decision.acceleration
                        allow_integral = allow_integral and decision.allow_integral
                        phase = decision.phase
                        if phase != 'NORMAL':
                            yaw_command,yaw_rate = yaw_reference,0.
                    if angular_decision is not None and angular_decision.phase!='NORMAL':
                        reference=angular_decision.hold_position
                        velocity=acceleration=(0.,0.,0.)
                        yaw_command,yaw_rate=angular_decision.hold_yaw,0.
                        rate_gain_scale=angular_decision.rate_gain_scale
                        attitude_gain_scale=angular_decision.attitude_gain_scale
                        allow_integral=False
                        phase=angular_decision.phase
                    dt = state.metadata.stamp-previous_control_stamp
                    if dt <= 0:
                        time.sleep(.002)
                        continue
                    if dt > .05:
                        raise RuntimeError('CONTROL_STATE_GAP')
                    wrench = controller.step(state, reference, velocity, acceleration,
                                             yaw_command, dt, allow_integral,yaw_rate=yaw_rate,
                                             rate_gain_scale=rate_gain_scale,attitude_gain_scale=attitude_gain_scale)
                    moment=np.array(wrench.moment)
                    pulse_this_tick=(angular_pulse and not pulse_sent and sim_elapsed>=5.
                                     and phase=='NORMAL')
                    if pulse_this_tick:moment[1]+=angular_pulse_moment
                    allocation = allocator.allocate(wrench.thrust, moment)
                    collective_values=collective_pulse.request(state,sim_elapsed) if collective_pulse else None
                    if collective_values is not None:
                        achieved=allocator.B@(allocator.max_thrust*np.array(collective_values))
                        allocation=Allocation(collective_values,tuple(achieved),True,0.,
                                              min(collective_values[0],1-collective_values[0]))
                    previous_saturated = allocation.saturated
                    values = allocation.pwm
                    details = {'reference': tuple(reference), 'wrench': asdict(wrench),
                               'allocation': asdict(allocation), 'sim_elapsed': sim_elapsed,'supervisor_phase':phase,
                               'yaw_reference':yaw_command,'yaw_rate_reference':yaw_rate}
                    details['angular_pulse_applied']=pulse_this_tick
                    details['rate_gain_scale']=rate_gain_scale
                    details['attitude_gain_scale']=attitude_gain_scale
                    details['moment_requested_including_experiment_pulse']=moment.tolist()
                    details['collective_pulse_applied']=collective_values is not None
                else:
                    values = pattern[index]
                command = PwmCommand(state.metadata.stamp, state.metadata.epoch, sequence, values)
                with sensors.lock:
                    if sensors.fault:
                        raise RuntimeError(sensors.fault)
                    accepted_wall = time.monotonic()
                    guard_observation = {'wall_now': accepted_wall,
                        'source_arrival': raw['imu_callback_arrival_wall_monotonic'],
                        'source_stamp': command.stamp, 'latest_stamp': sensors.clock.stamp,
                        'latest_arrival': sensors.clock.arrival}
                    allowed = guard.accept(command, sensors.clock, accepted_wall,
                                           raw['imu_callback_arrival_wall_monotonic'])
                if allowed:
                    msg = RotorPWM()
                    msg.header.stamp = rospy.Time.from_sec(command.stamp)
                    msg.header.seq = sequence
                    msg.rotorPWM0, msg.rotorPWM1, msg.rotorPWM2, msg.rotorPWM3 = command.values
                    if (control_stall_seconds and control_stall_stage == 'before-publish'
                            and control_stall is None and state.metadata.stamp-start_stamp >= 6.):
                        stall_begin = time.monotonic()
                        time.sleep(control_stall_seconds)
                        control_stall = {'begin_wall_monotonic': stall_begin,
                                         'end_wall_monotonic': time.monotonic(),
                                         'source_stamp': state.metadata.stamp, 'stage': control_stall_stage}
                    with sensors.lock:
                        if sensors.fault:
                            raise RuntimeError(sensors.fault)
                        recheck_wall = time.monotonic()
                        guard_observation = {'stage': 'before_publish', 'wall_now': recheck_wall,
                            'source_arrival': raw['imu_callback_arrival_wall_monotonic'],
                            'source_stamp': command.stamp, 'latest_stamp': sensors.clock.stamp,
                            'latest_arrival': sensors.clock.arrival}
                        guard.before_publish(command, sensors.clock, recheck_wall,
                                             raw['imu_callback_arrival_wall_monotonic'])
                    publish_begin_wall=time.monotonic()
                    publisher.publish(msg)
                    published_wall = time.monotonic()
                    last_published_wall=published_wall
                    guard.published(published_wall)
                    rows.append({'wall_elapsed': now-start, 'command': asdict(command),
                                 'guard_accept_wall_elapsed': accepted_wall-start,
                                 'publish_wall_elapsed': published_wall-start,
                                 'publish_begin_wall_monotonic':publish_begin_wall,
                                 'publish_end_wall_monotonic':published_wall,
                                 'raw': raw, 'state': {k:v for k,v in asdict(state).items() if k != 'metadata'},
                                 **details})
                    sequence += 1
                    if details.get('angular_pulse_applied'):pulse_sent=True
                    if details.get('collective_pulse_applied'):collective_pulse.published(command.stamp)
                previous_control_stamp = state.metadata.stamp
                next_tick += 1/control_hz
                delay = next_tick-time.monotonic()
                if delay < -.1:
                    raise RuntimeError('CONTROL_DEADLINE')
                time.sleep(max(0., delay))
            if collective_pulse is not None and not collective_pulse.sent:
                raise RuntimeError('COLLECTIVE_EXCITATION_NOT_SENT')
            if control_stall_seconds and control_stall is None:
                raise RuntimeError('CONTROL_STALL_NOT_INJECTED')
            if angular_pulse:
                validate_angular_pulse_outcome(pulse_sent, angular_supervisor.phase)
        except Exception as error:
            fault = str(error)
            failed_wall=time.monotonic()
            fault_observation={'reason':fault,'fresh_read_in_fault_iteration':fresh_read,
                'failure_wall_monotonic':failed_wall,
                'wall_elapsed_at_last_read':latest_elapsed,
                'wall_since_last_publication_s':None if last_published_wall is None else failed_wall-last_published_wall,
                'state':asdict(latest_state) if latest_state else None,'raw':latest_raw,
                'guard_observation':guard_observation}
            with sensors.lock:
                fault_observation['rate_guard_event'] = sensors.rate_guard_event
        finally:
            control_end_wall=time.monotonic()
            if monitor:
                monitor.close()
            if publisher:
                publisher.unregister()
            if reference_worker:reference_worker.close()
            sensors.close()
    (run/'pwm_telemetry.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    if fault_observation is not None:
        (run/'fault_observation.json').write_text(json.dumps(fault_observation,indent=2)+'\n')
    result = {'commands': len(rows), 'fault': fault, 'startup_diagnostics': sensors.startup_snapshot(),
              'startup_velocity': sensors.startup_velocity_snapshot(), 'levels': levels,
              'control_end_wall_monotonic':control_end_wall,
              'motor_pulses': motor_pulses,
              'hover_seconds': hover_seconds, 'control_hz': control_hz,
              'control_stall': control_stall,
              'step': step,
              'reference_clock': 'IMU',
              'gps_rejections': sensors.estimator.rejections,
              'injection': asdict(sensors.local_injection) if sensors.local_injection else None,
              'graph_monitor':monitor.summary() if monitor else None,
              'gps_bridge_enabled':gps_bridge,'recovery_transitions':recovery.transitions,
              'yaw_motion':yaw_motion,
              'braking':braking,
              'angular_recovery_enabled':angular_recovery,
              'angular_recovery_phase':angular_supervisor.phase,
              'angular_recovery_profile':angular_recovery_profile,
              'angular_recovery_transitions':angular_supervisor.transitions,
              'angular_pulse_sent':pulse_sent,
              'angular_pulse_moment_Nm':angular_pulse_moment if angular_pulse else None,
              'reference_worker':reference_worker.summary() if reference_worker else None,
              'reference_hold_transitions':reference_hold.transitions,
              'collective_excitation':None if collective_pulse is None else vars(collective_pulse),
              'pulse_seconds': pulse_seconds,
              'stage': 'LOCAL_PWM_CLOSED_LOOP_HOVER' if hover_seconds else 'LOCAL_OPEN_LOOP_IDENTIFICATION',
              'official_result': 'UNKNOWN', 'host_scene_stop_required': True}
    (run/'pwm_summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    return 0 if rows and fault is None else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--levels', default='.10,.15,.20,.25,.30,.35,.40')
    parser.add_argument('--pulse-seconds', type=float, default=.3)
    parser.add_argument('--motor-pulses', action='store_true')
    parser.add_argument('--hover-seconds', type=float, default=0.)
    parser.add_argument('--control-hz', type=int, choices=(50, 80), default=80)
    parser.add_argument('--control-stall-seconds', type=float, choices=(0., .15), default=0.)
    parser.add_argument('--control-stall-stage', choices=('before-check','before-publish'), default='before-check')
    parser.add_argument('--step', action='store_true')
    parser.add_argument('--sensor-fault', choices=('GPS_ZERO','IMU_DROP','IMU_ROLLBACK','IMU_RATE_SPIKE'))
    parser.add_argument('--fault-seconds',type=float,default=.15)
    parser.add_argument('--gps-bridge',action='store_true')
    parser.add_argument('--yaw-motion',action='store_true')
    parser.add_argument('--braking',action='store_true')
    parser.add_argument('--stationary-bootstrap',action='store_true')
    parser.add_argument('--angular-recovery',action='store_true')
    parser.add_argument('--angular-recovery-profile',choices=('standard','fast'),default='standard')
    parser.add_argument('--angular-pulse',action='store_true',help='single bounded pitch moment pulse at 5s')
    parser.add_argument('--angular-pulse-moment',type=float)
    parser.add_argument('--isolated-reference',action='store_true')
    parser.add_argument('--reference-stall-seconds',type=float,default=0.)
    parser.add_argument('--collective-pulse-pwm',type=float)
    args = parser.parse_args()
    if args.control_stall_stage != 'before-check' and not args.control_stall_seconds:
        parser.error('control stall stage requires a control stall')
    if args.control_stall_seconds and (args.hover_seconds < 10 or args.motor_pulses or args.sensor_fault or args.gps_bridge
            or args.step or args.yaw_motion or args.braking or args.angular_recovery or args.angular_pulse
            or args.isolated_reference or args.collective_pulse_pwm is not None):
        parser.error('control stall is restricted to isolated healthy bounded hover')
    levels = tuple(float(value) for value in args.levels.split(','))
    if (not levels or len(levels) > 20 or not all(0 < v <= .6 for v in levels)
            or not .05 <= args.pulse_seconds <= 1 or not 0 <= args.hover_seconds <= 20
            or (args.hover_seconds and args.motor_pulses)):
        parser.error('bounded identification requires <=20 levels in (0,.6], pulses [.05,1]s')
    if args.step and args.hover_seconds < 10:
        parser.error('step profile requires hover-seconds >=10')
    if (args.braking or args.yaw_motion) and args.hover_seconds < 10:
        parser.error('braking/yaw identification requires >=10s hover experiment')
    if args.braking and args.step:
        parser.error('braking and position step are separate experiments')
    if args.sensor_fault and (args.hover_seconds < 8 or not 0 < args.fault_seconds <= 1):
        parser.error('local sensor injection requires >=8s hover and duration (0,1]')
    if args.angular_recovery and (args.hover_seconds<10 or args.gps_bridge or args.sensor_fault):
        parser.error('angular recovery requires >=10s healthy-state hover without GPS/fault modes')
    if args.angular_recovery_profile!='standard' and not args.angular_recovery:
        parser.error('fast recovery profile requires angular-recovery')
    if args.angular_pulse and (not args.angular_recovery or args.step or args.braking or args.yaw_motion):
        parser.error('angular pulse requires isolated angular-recovery hover')
    if args.angular_pulse_moment is not None and (not args.angular_pulse or not 0<args.angular_pulse_moment<=.8):
        parser.error('angular-pulse-moment requires angular-pulse, moment (0,.8] Nm')
    if args.isolated_reference and (args.hover_seconds<10 or args.angular_recovery or args.gps_bridge or args.sensor_fault):
        parser.error('isolated reference requires >=10s healthy-state hover without other fault modes')
    if args.reference_stall_seconds and (not args.isolated_reference or not .1<=args.reference_stall_seconds<=1.):
        parser.error('reference stall requires isolated reference and duration [.1,1]')
    if args.collective_pulse_pwm is not None and (not 0<=args.collective_pulse_pwm<=1
            or args.hover_seconds<10 or args.step or args.braking or args.yaw_motion or args.sensor_fault
            or args.gps_bridge or args.angular_recovery or args.isolated_reference):
        parser.error('collective pulse requires isolated >=10s healthy-state hover, PWM in [0,1]')
    return run_experiment(args.run_dir, levels, args.pulse_seconds, args.motor_pulses, args.hover_seconds, args.step,
                          args.sensor_fault,args.fault_seconds,args.gps_bridge,args.yaw_motion,args.braking,
                          args.stationary_bootstrap,args.angular_recovery,args.angular_pulse,
                          args.isolated_reference,args.reference_stall_seconds,args.collective_pulse_pwm,
                          .5 if args.angular_pulse_moment is None else args.angular_pulse_moment,
                          args.angular_recovery_profile,args.control_hz,args.control_stall_seconds,
                          args.control_stall_stage)


if __name__ == '__main__':
    raise SystemExit(main())
