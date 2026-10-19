"""Separate-scene bounded VEL baseline. Never imports or publishes RotorPWM."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
import numpy as np
from adapters.ros_noetic.flight_sensors import FlightSensors
from control.control_authority.lease import ControlLease, check_graph
from control.pwm_publisher.guard import CommandGuard
from control.reference import benchmark_reference
from control.vel_benchmark.controller import VelocityCommand, body_velocity
from estimation.geometry import rotation
from interfaces.contracts import Health


def run_experiment(run, duration, step):
    import rosgraph
    import rospy
    from airsim_ros.msg import VelCmd
    rospy.init_node('rmua_next_vel_benchmark', anonymous=True, disable_signals=True)
    master = rosgraph.Master(rospy.get_name())
    rows, fault = [], None
    connections = 0
    def graph(require_self):
        pubs, _, _ = master.getSystemState()
        check_graph(dict(pubs), rospy.get_name(), 'VEL', require_self)
    with ControlLease(rospy.get_param('/run_id'), 'VEL'):
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
            publisher = rospy.Publisher('/airsim_node/drone_1/vel_body_cmd', VelCmd, queue_size=1)
            deadline = time.monotonic()+2
            while publisher.get_num_connections() != 1 and time.monotonic() < deadline:
                time.sleep(.01)
            connections = publisher.get_num_connections()
            if connections != 1:
                raise RuntimeError('VEL_SUBSCRIBER_MD5_OR_COUNT')
            graph(True)
            _, subscribers, _ = master.getSystemState()
            inputs = sorted(topic for topic, nodes in subscribers if rospy.get_name() in nodes)
            config = json.loads((Path(__file__).resolve().parents[2]/'configs/official_baseline.yaml').read_text())
            if any(topic not in config['sensors'] for topic in inputs):
                raise RuntimeError('INPUT_WHITELIST_POLICY')
            (run/'runtime_input_audit.json').write_text(json.dumps({'node':rospy.get_name(),
                'topics':inputs,'debug_inputs':False},indent=2)+'\n')
            state, raw = sensors.read()
            origin = np.array(state.position_ned)
            R = rotation(state.orientation_wxyz)
            yaw = math.atan2(R[1, 0], R[0, 0])
            guard = CommandGuard(sensors.clock.epoch, minimum_period=.0105)
            start = time.monotonic()
            next_tick, next_graph = start, start
            sequence = 0
            while time.monotonic()-start < duration:
                now = time.monotonic()
                state, raw = sensors.read()
                if state.health != Health.OK:
                    raise RuntimeError('STATE_HEALTH_'+state.health.value)
                if rotation(state.orientation_wxyz)[2,2] < math.cos(.35):
                    raise RuntimeError('TILT_LIMIT')
                if np.linalg.norm(np.array(state.position_ned)-origin) > 1.8:
                    raise RuntimeError('POSITION_LIMIT')
                if np.linalg.norm(state.velocity_ned) > 4:
                    raise RuntimeError('SPEED_LIMIT')
                if now >= next_graph:
                    graph(True)
                    next_graph = now+.1
                reference, velocity, _ = benchmark_reference(origin, now-start, step)
                values = body_velocity(state, reference, velocity, yaw)
                command = VelocityCommand(state.metadata.stamp, state.metadata.epoch, sequence, values)
                with sensors.lock:
                    accepted_wall = time.monotonic()
                    allowed = guard.accept(command, sensors.clock, accepted_wall)
                if allowed:
                    msg = VelCmd()
                    msg.header.stamp = rospy.Time.from_sec(command.stamp)
                    msg.header.seq = sequence
                    msg.vx, msg.vy, msg.vz, msg.yawRate = values
                    msg.va, msg.stop = 4, 0
                    publisher.publish(msg)
                    rows.append({'wall_elapsed':now-start,'guard_accept_wall_elapsed':accepted_wall-start,
                        'publish_wall_elapsed':time.monotonic()-start, 'command':asdict(command),
                        'raw':raw,'reference':reference,
                        'state':{k:v for k,v in asdict(state).items() if k != 'metadata'}})
                    sequence += 1
                next_tick += .02
                delay = next_tick-time.monotonic()
                if delay < -.1:
                    raise RuntimeError('CONTROL_DEADLINE')
                time.sleep(max(0.,delay))
        except Exception as error:
            fault = str(error)
        finally:
            if publisher:
                publisher.unregister()
            sensors.close()
    (run/'vel_telemetry.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    result = {'commands':len(rows),'fault':fault,'control_mode':'VEL',
              'connected_subscribers':connections,'message_md5':VelCmd._md5sum,
              'duration_s':duration,'step':step,'official_result':'UNKNOWN',
              'host_scene_stop_required':True}
    (run/'vel_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    return 0 if rows and fault is None else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--hover-seconds',type=float,required=True)
    parser.add_argument('--step',action='store_true')
    args=parser.parse_args()
    if not 3 <= args.hover_seconds <= 20 or (args.step and args.hover_seconds < 10):
        parser.error('bounded VEL duration must be [3,20], step >=10')
    return run_experiment(args.run_dir,args.hover_seconds,args.step)


if __name__ == '__main__':
    raise SystemExit(main())
