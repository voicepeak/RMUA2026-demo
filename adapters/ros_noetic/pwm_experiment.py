"""Bounded local simulator identification. Host harness MUST stop scene on return.

No zero-PWM hover fallback. No VEL, debug input, task service or competition mode.
This is an open-loop experiment, not an accepted flight controller.
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


def run_experiment(run, levels, pulse_seconds):
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
            guard = CommandGuard(sensors.clock.epoch)
            start = time.monotonic()
            next_tick, next_graph = start, start
            sequence = 0
            while True:
                now = time.monotonic()
                index = int((now-start)/pulse_seconds)
                if index >= len(levels):
                    break
                state, raw = sensors.read()
                R = rotation(state.orientation_wxyz)
                if state.health != Health.OK:
                    raise RuntimeError('STATE_HEALTH_'+state.health.value)
                if R[2, 2] < math.cos(.25):
                    raise RuntimeError('TILT_LIMIT')
                if np.linalg.norm(state.velocity_ned) > 4:
                    raise RuntimeError('SPEED_LIMIT')
                if np.linalg.norm(np.array(state.position_ned)-origin) > 1.2:
                    raise RuntimeError('POSITION_LIMIT')
                if now >= next_graph:
                    graph(True)
                    next_graph = now+.1
                command = PwmCommand(state.metadata.stamp, state.metadata.epoch, sequence, (levels[index],)*4)
                with sensors.lock:
                    allowed = guard.accept(command, sensors.clock, time.monotonic())
                if allowed:
                    msg = RotorPWM()
                    msg.header.stamp = rospy.Time.from_sec(command.stamp)
                    msg.header.seq = sequence
                    msg.rotorPWM0, msg.rotorPWM1, msg.rotorPWM2, msg.rotorPWM3 = command.values
                    publisher.publish(msg)
                    rows.append({'wall_elapsed': now-start, 'command': asdict(command),
                                 'raw': raw, 'state': {k:v for k,v in asdict(state).items() if k != 'metadata'}})
                    sequence += 1
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
              'pulse_seconds': pulse_seconds, 'stage': 'LOCAL_OPEN_LOOP_IDENTIFICATION',
              'official_result': 'UNKNOWN', 'host_scene_stop_required': True}
    (run/'pwm_summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    return 0 if rows and fault is None else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--levels', default='.10,.15,.20,.25,.30,.35,.40')
    parser.add_argument('--pulse-seconds', type=float, default=.3)
    args = parser.parse_args()
    levels = tuple(float(value) for value in args.levels.split(','))
    if (not levels or len(levels) > 20 or not all(0 < v <= .6 for v in levels)
            or not .05 <= args.pulse_seconds <= 1):
        parser.error('bounded identification requires <=20 levels in (0,.6], pulses [.05,1]s')
    return run_experiment(args.run_dir, levels, args.pulse_seconds)


if __name__ == '__main__':
    raise SystemExit(main())
