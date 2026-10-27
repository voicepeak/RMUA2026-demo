"""Read-only official current end-goal and IMU-clock observer."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import threading
import time

from estimation.clock import SensorClock
from planning.mission.goal_tracker import GoalTracker


def observe(run, duration):
    import rospy
    import rosgraph
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Imu
    rospy.init_node('rmua_goal_observer', anonymous=True, disable_signals=True)
    lock = threading.RLock(); clock = SensorClock(); tracker = GoalTracker()
    rows = []; events = Counter(); faults = []
    def imu(message):
        with lock:
            try:
                if clock.observe(message.header.stamp.to_sec(), time.monotonic()):
                    tracker.reset(clock.epoch); events['IMU_CLOCK_RESET'] += 1
            except ValueError as error: faults.append('IMU:' + str(error))
    def goal(message):
        arrival = time.monotonic()
        with lock:
            events['goal_received'] += 1
            if not clock.fresh(arrival):
                events['no_fresh_imu'] += 1; return
            try:
                p = message.pose.position
                position = (p.x, p.y, p.z)
                observed = tracker.observe(position, message.header.stamp.to_sec(), message.header.frame_id,
                                           arrival, clock.stamp, clock.epoch)
                rows.append({'position': position, 'source_stamp': message.header.stamp.to_sec(),
                             'source_frame': message.header.frame_id, 'arrival_wall': arrival,
                             'imu_stamp': clock.stamp, 'epoch': clock.epoch, 'snapshot': asdict(observed)})
            except (ValueError, ArithmeticError) as error:
                events['invalid:' + str(error)] += 1
    listeners = [rospy.Subscriber('/airsim_node/end_goal', PoseStamped, goal, queue_size=1),
                 rospy.Subscriber('/airsim_node/drone_1/imu/imu', Imu, imu, queue_size=1)]
    graph = None
    try:
        end = time.monotonic() + duration
        while time.monotonic() < end and not rospy.is_shutdown():
            time.sleep(.02)
        with lock:
            latest = tracker.snapshot(time.monotonic(), clock.epoch, clock.fresh(time.monotonic()))
        pubs, subs, _ = rosgraph.Master(rospy.get_name()).getSystemState(); node = rospy.get_name()
        graph = {'subscriptions': sorted(t for t, nodes in subs if node in nodes),
                 'publishers': sorted(t for t, nodes in pubs if node in nodes)}
        if graph['subscriptions'] != sorted(['/airsim_node/end_goal','/airsim_node/drone_1/imu/imu']) or set(graph['publishers']) - {'/rosout'}:
            faults.append('GRAPH_POLICY')
    except Exception as error:
        faults.append(str(error)); latest = None
    finally:
        for listener in listeners: listener.unregister()
    with (run / 'goal_observations.jsonl').open('x') as output:
        for row in rows: output.write(json.dumps(row, allow_nan=False) + '\n')
    summary = {'events': dict(events), 'rows': len(rows), 'faults': faults, 'node_graph': graph,
               'latest': asdict(latest) if latest is not None else None,
               'control_published': False, 'debug_inputs_used': False, 'stage_completion_inferred': False,
               'frame_calibration_verified': False, 'acceptance_eligible': False}
    (run / 'goal_summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    return 0 if rows and latest is not None and latest.status == 'OBSERVED' and not faults else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=6.)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 30: parser.error('invalid duration')
    return observe(args.run_dir, args.duration)


if __name__ == '__main__': raise SystemExit(main())
