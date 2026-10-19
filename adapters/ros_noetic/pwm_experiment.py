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
from control.motor_allocator.allocator import MotorAllocator
from control.se3_controller.controller import Se3Controller


def motor_pattern():
    # Lift gently, then probe one motor at a time with very small positive pulses.
    pattern = [(.20,)*4]*4 + [(.176,)*4]*2
    for motor in range(4):
        values = [.176]*4
        values[motor] += .003
        pattern.append(tuple(values))
        pattern.append((.176,)*4)
    return tuple(pattern)


def run_experiment(run, levels, pulse_seconds, motor_pulses=False, hover_seconds=0, step=False):
    import rospy
    import rosgraph
    from airsim_ros.msg import RotorPWM
    rospy.init_node('rmua_next_pwm_identification', anonymous=True, disable_signals=True)
    master = rosgraph.Master(rospy.get_name())
    scene_id = rospy.get_param('/run_id')
    rows, fault = [], None
    def graph(require_self):
        pubs, _, _ = master.getSystemState()
        check_graph(dict(pubs), rospy.get_name(), require_self=require_self)

    with ControlLease(scene_id, 'PWM'):
        graph(False)
        sensors = FlightSensors()
        publisher = None
        try:
            deadline = time.monotonic()+5
            while time.monotonic() < deadline:
                try:
                    state, raw = sensors.read()
                    if state.health == Health.OK:
                        break
                except (ValueError, RuntimeError):
                    pass
                time.sleep(.01)
            else:
                raise RuntimeError('NO_HEALTHY_STATE')
            origin = np.array(state.position_ned)
            publisher = rospy.Publisher('/airsim_node/drone_1/rotor_pwm_cmd', RotorPWM, queue_size=1)
            deadline = time.monotonic()+2
            while publisher.get_num_connections() != 1 and time.monotonic() < deadline:
                time.sleep(.01)
            if publisher.get_num_connections() != 1:
                raise RuntimeError('PWM_SUBSCRIBER_COUNT')
            graph(True)
            _, subscribers, _ = master.getSystemState()
            runtime_inputs = sorted(topic for topic, nodes in subscribers if rospy.get_name() in nodes)
            official_config = json.loads((Path(__file__).resolve().parents[2]/'configs/official_baseline.yaml').read_text())
            if any(topic not in official_config['sensors'] for topic in runtime_inputs):
                raise RuntimeError('INPUT_WHITELIST_POLICY')
            (run/'runtime_input_audit.json').write_text(json.dumps({'node': rospy.get_name(),
                'topics': runtime_inputs, 'debug_inputs': False}, indent=2)+'\n')
            guard = CommandGuard(sensors.clock.epoch, minimum_period=.0105)
            pattern = motor_pattern() if motor_pulses else tuple((v,)*4 for v in levels)
            config = json.loads((Path(__file__).resolve().parents[2]/'configs/vehicle_identification.json').read_text())
            allocator, controller = MotorAllocator(config), Se3Controller(config)
            previous_saturated = False
            yaw_reference = math.atan2(rotation(state.orientation_wxyz)[1, 0],
                                       rotation(state.orientation_wxyz)[0, 0])
            start = time.monotonic()
            next_tick, next_graph = start, start
            previous_control_wall = start-1/80
            sequence = 0
            while True:
                now = time.monotonic()
                index = int((now-start)/pulse_seconds)
                if (hover_seconds and now-start >= hover_seconds) or (not hover_seconds and index >= len(pattern)):
                    break
                state, raw = sensors.read()
                R = rotation(state.orientation_wxyz)
                if state.health != Health.OK:
                    raise RuntimeError('STATE_HEALTH_'+state.health.value)
                if R[2, 2] < math.cos(.25):
                    raise RuntimeError('TILT_LIMIT')
                if np.linalg.norm(state.velocity_ned) > 4:
                    raise RuntimeError('SPEED_LIMIT')
                if np.linalg.norm(np.array(state.position_ned)-origin) > (1.8 if hover_seconds else 1.2):
                    raise RuntimeError('POSITION_LIMIT')
                if now >= next_graph:
                    graph(True)
                    next_graph = now+.1
                details = {}
                if hover_seconds:
                    s = min(1., (now-start)/3.)
                    h = 10*s**3-15*s**4+6*s**5
                    dh = (30*s**2-60*s**3+30*s**4)/3
                    ddh = (60*s-180*s**2+120*s**3)/9 if s < 1 else 0.
                    reference = origin + (0., 0., -h)
                    if step and now-start >= 4:
                        reference[0] += .4 if now-start < 8 else -.4
                    dt = now-previous_control_wall
                    wrench = controller.step(state, tuple(reference), (0., 0., -dh), (0., 0., -ddh),
                                             yaw_reference, dt, not previous_saturated)
                    allocation = allocator.allocate(wrench.thrust, wrench.moment)
                    previous_saturated = allocation.saturated
                    values = allocation.pwm
                    details = {'reference': tuple(reference), 'wrench': asdict(wrench),
                               'allocation': asdict(allocation)}
                else:
                    values = pattern[index]
                command = PwmCommand(state.metadata.stamp, state.metadata.epoch, sequence, values)
                with sensors.lock:
                    accepted_wall = time.monotonic()
                    allowed = guard.accept(command, sensors.clock, accepted_wall)
                if allowed:
                    msg = RotorPWM()
                    msg.header.stamp = rospy.Time.from_sec(command.stamp)
                    msg.header.seq = sequence
                    msg.rotorPWM0, msg.rotorPWM1, msg.rotorPWM2, msg.rotorPWM3 = command.values
                    publisher.publish(msg)
                    published_wall = time.monotonic()
                    rows.append({'wall_elapsed': now-start, 'command': asdict(command),
                                 'guard_accept_wall_elapsed': accepted_wall-start,
                                 'publish_wall_elapsed': published_wall-start,
                                 'raw': raw, 'state': {k:v for k,v in asdict(state).items() if k != 'metadata'},
                                 **details})
                    sequence += 1
                previous_control_wall = now
                next_tick += 1/80.
                delay = next_tick-time.monotonic()
                if delay < -.1:
                    raise RuntimeError('CONTROL_DEADLINE')
                time.sleep(max(0., delay))
        except Exception as error:
            fault = str(error)
        finally:
            if publisher:
                publisher.unregister()
            sensors.close()
    (run/'pwm_telemetry.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    result = {'commands': len(rows), 'fault': fault, 'levels': levels,
              'motor_pulses': motor_pulses,
              'hover_seconds': hover_seconds,
              'step': step,
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
    parser.add_argument('--step', action='store_true')
    args = parser.parse_args()
    levels = tuple(float(value) for value in args.levels.split(','))
    if (not levels or len(levels) > 20 or not all(0 < v <= .6 for v in levels)
            or not .05 <= args.pulse_seconds <= 1 or not 0 <= args.hover_seconds <= 20
            or (args.hover_seconds and args.motor_pulses)):
        parser.error('bounded identification requires <=20 levels in (0,.6], pulses [.05,1]s')
    if args.step and args.hover_seconds < 10:
        parser.error('step profile requires hover-seconds >=10')
    return run_experiment(args.run_dir, levels, args.pulse_seconds, args.motor_pulses, args.hover_seconds, args.step)


if __name__ == '__main__':
    raise SystemExit(main())
