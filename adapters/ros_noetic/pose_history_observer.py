"""Read-only camera-header association with historical IMU/GPS ESKF poses."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import threading
import time

from adapters.ros_noetic.flight_sensors import FlightSensors
from estimation.history import HistoryUnavailable, PoseHistory


CAMERA = '/airsim_node/drone_1/front_left/Scene'
INPUTS = sorted((CAMERA, '/airsim_node/initial_pose', '/airsim_node/drone_1/gps',
                 '/airsim_node/drone_1/imu/imu'))


def observe(run, duration):
    import rosgraph
    import rospy
    from sensor_msgs.msg import Image

    rospy.init_node('rmua_pose_history_observer', anonymous=True, disable_signals=True)
    history = PoseHistory()
    sensors = FlightSensors(pose_history=history)
    lock = threading.Lock()
    pending = None

    def receive(message):
        nonlocal pending
        with lock:
            pending = (message.header.stamp.to_sec(), time.monotonic(), message.header.frame_id)

    listener = rospy.Subscriber(CAMERA, Image, receive, queue_size=1, buff_size=4000000)
    started = time.monotonic()
    dumped = None
    count = matched = 0
    reasons = Counter()
    fault = None
    graph = None
    source_count = 0
    last_image_arrival = None
    with (run / 'history_states.jsonl').open('x') as sources, (run / 'camera_history.jsonl').open('x') as output:
        def dump_sources():
            nonlocal dumped, source_count
            for state, arrival in history.snapshot_sources():
                key = (state.metadata.epoch, state.metadata.stamp)
                if dumped is None or key > dumped:
                    sources.write(json.dumps({'state': asdict(state), 'arrival_wall_monotonic': arrival},
                                             allow_nan=False) + '\n')
                    dumped = key
                    source_count += 1

        try:
            while time.monotonic() - started < duration and not rospy.is_shutdown():
                with sensors.lock:
                    if sensors.fault:
                        raise RuntimeError(sensors.fault)
                    ready = sensors.estimator.initialized and sensors.raw is not None
                    if ready and not sensors.clock.fresh(time.monotonic()):
                        raise RuntimeError('STALE_IMU')
                dump_sources()
                with lock:
                    item, pending = pending, None
                if not ready or item is None:
                    if time.monotonic() - started > 5 and (not ready or last_image_arrival is None):
                        raise RuntimeError('NO_STATE_OR_CAMERA')
                    if last_image_arrival is not None and time.monotonic() - last_image_arrival > .5:
                        raise RuntimeError('STALE_CAMERA')
                    time.sleep(.003)
                    continue
                stamp, arrival, header_frame = item
                last_image_arrival = arrival
                begin = time.monotonic()
                pose = None
                reason = None
                with sensors.lock:
                    if sensors.fault:
                        raise RuntimeError(sensors.fault)
                    now, epoch = sensors.clock.stamp, sensors.clock.epoch
                    query_wall = time.monotonic()
                    try:
                        pose = history.lookup(stamp, now, query_wall, epoch)
                    except HistoryUnavailable as error:
                        reason = str(error)
                elapsed = time.monotonic() - begin
                output.write(json.dumps({'image_stamp': stamp, 'image_header_frame_id': header_frame,
                    'image_arrival_wall_monotonic': arrival, 'query_wall_monotonic': query_wall,
                    'query_now': now, 'epoch': epoch, 'lookup_elapsed_wall_s': elapsed,
                    'reason': reason, 'historical_pose': asdict(pose) if pose else None,
                    'world_projection_applied': False, 'current_control_eligible': False}, allow_nan=False) + '\n')
                count += 1
                if pose is not None:
                    matched += 1
                else:
                    reasons[reason] += 1
                time.sleep(.003)
        except Exception as error:
            fault = str(error)
        finally:
            try:
                pubs, subs, _ = rosgraph.Master(rospy.get_name()).getSystemState()
                node = rospy.get_name()
                graph = {'subscriptions': sorted(t for t, nodes in subs if node in nodes),
                         'publishers': sorted(t for t, nodes in pubs if node in nodes)}
                if graph['subscriptions'] != INPUTS or set(graph['publishers']) - {'/rosout'}:
                    fault = fault or 'HISTORY_OBSERVER_GRAPH_POLICY'
            except Exception as error:
                fault = fault or 'GRAPH_AUDIT_FAILED:' + str(error)
            listener.unregister()
            sensors.close()
            dump_sources()
    summary = {'images': count, 'historical_matches': matched, 'unavailable_reasons': dict(reasons),
               'source_states': source_count, 'fault': fault, 'node_graph': graph,
               'horizon_s': history.horizon_s, 'max_bracket_s': history.max_bracket_s,
               'control_published': False, 'debug_inputs_used': False,
               'image_exposure_time_calibrated': False, 'world_projection_applied': False,
               'historical_uncertainty_calibrated': False, 'acceptance_eligible': False}
    (run / 'pose_history_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return 0 if matched and fault is None else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=10.)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 30:
        parser.error('duration must be in (0,30] seconds')
    return observe(args.run_dir, args.duration)


if __name__ == '__main__':
    raise SystemExit(main())
