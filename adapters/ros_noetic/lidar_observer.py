"""Bounded read-only online LiDAR endpoint observer, isolated from flight control."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import threading
import time

import numpy as np

from adapters.ros_noetic.lidar_cloud import decode_xyz
from world_model.occupancy.lidar import endpoint_snapshot


TOPIC = '/airsim_node/drone_1/lidar'


def observe(run, duration):
    import rosgraph
    import rospy
    from sensor_msgs.msg import PointCloud2

    rospy.init_node('rmua_lidar_endpoints', anonymous=True, disable_signals=True)
    lock = threading.Lock()
    pending = None
    received = 0

    def receive(message):
        nonlocal pending, received
        with lock:
            pending = (message, time.monotonic())
            received += 1

    listener = rospy.Subscriber(TOPIC, PointCloud2, receive, queue_size=1, buff_size=4000000)
    events = Counter()
    epoch = 0
    previous = None
    latest = None
    schemas = []
    count = 0
    started = time.monotonic()
    fault = None
    graph = None
    last_processed_arrival = None
    try:
        with (run / 'lidar_frames.jsonl').open('x') as output:
            while time.monotonic() - started < duration and not rospy.is_shutdown():
                now = time.monotonic()
                if last_processed_arrival is not None and now - last_processed_arrival > .5:
                    raise RuntimeError('STALE_LIDAR_STREAM')
                if last_processed_arrival is None and now - started > 5:
                    raise RuntimeError('NO_VALID_LIDAR_STREAM')
                with lock:
                    item, pending = pending, None
                if item is None:
                    time.sleep(.002)
                    continue
                message, arrival = item
                begin = time.monotonic()
                if begin - arrival > .2:
                    events['STALE_PENDING'] += 1
                    continue
                try:
                    cloud = decode_xyz(message, epoch)
                    stamp = cloud.metadata.stamp
                    if previous is not None and stamp <= previous:
                        if stamp == previous:
                            events['DUPLICATE'] += 1
                            continue
                        epoch += 1
                        latest = None
                        events['CLOCK_RESET'] += 1
                        cloud = decode_xyz(message, epoch)
                    previous = stamp
                    snapshot = endpoint_snapshot(cloud, stamp, epoch)
                except (ValueError, ArithmeticError) as error:
                    events['REJECTED:' + str(error)] += 1
                    continue
                schema = {'frame_id': message.header.frame_id,
                          'point_step': message.point_step, 'is_bigendian': message.is_bigendian,
                          'fields': [{'name': f.name, 'offset': f.offset, 'datatype': f.datatype,
                                      'count': f.count} for f in message.fields]}
                if schema not in schemas:
                    schemas.append(schema)
                if count == 0:
                    with (run / 'lidar_first_message.bin').open('xb') as raw:
                        raw.write(bytes(message.data))
                    first = dict(schema, stamp=stamp, width=message.width, height=message.height,
                                 row_step=message.row_step, is_dense=message.is_dense)
                    (run / 'lidar_first_message.json').write_text(json.dumps(first, indent=2) + '\n')
                ranges = np.hypot(np.hypot(cloud.points[:, 0], cloud.points[:, 1]), cloud.points[:, 2])
                row = {'stamp': stamp, 'epoch': epoch, 'frame_id': cloud.metadata.frame_id,
                       'arrival_wall_monotonic': arrival, 'processing_begin_wall_monotonic': begin,
                       'processing_end_wall_monotonic': time.monotonic(),
                       'input_points': cloud.input_points, 'finite_points': len(cloud.points),
                       'nonfinite_points': cloud.nonfinite_points,
                       'unconfirmed_zero_or_max_range_points': int(((ranges <= 0) | (ranges >= 30)).sum()),
                       'occupied_voxels': len(snapshot.cells), 'free_voxels': 0,
                       'motion_compensated': False, 'world_transform_applied': False}
                output.write(json.dumps(row, allow_nan=False) + '\n')
                latest = snapshot
                last_processed_arrival = arrival
                count += 1
            if last_processed_arrival is None or time.monotonic() - last_processed_arrival > .5:
                raise RuntimeError('STALE_LIDAR_STREAM')
            # Only the observer's registrations are audited; the simulator's
            # debug publishers are not algorithm subscriptions.
            pubs, subs, _ = rosgraph.Master(rospy.get_name()).getSystemState()
            node = rospy.get_name()
            graph = {'subscriptions': sorted(t for t, nodes in subs if node in nodes),
                     'publishers': sorted(t for t, nodes in pubs if node in nodes)}
            if graph['subscriptions'] != [TOPIC] or set(graph['publishers']) - {'/rosout'}:
                raise RuntimeError('LIDAR_OBSERVER_GRAPH_POLICY')
    except Exception as error:
        fault = str(error)
    finally:
        listener.unregister()
        if latest is not None:
            (run / 'lidar_endpoint_snapshot.json').write_text(json.dumps(asdict(latest), indent=2) + '\n')
        summary = {'processed_frames': count, 'received_frames': received, 'events': dict(events),
                   'fault': fault, 'schemas': schemas, 'node_graph': graph,
                   'control_published': False, 'debug_inputs_used': False,
                   'frame_semantics': 'frozen_sensor_frame_at_scan_stamp',
                   'motion_compensated': False, 'free_space_certified': False,
                   'flight_clearance_eligible': False, 'official_result': 'UNKNOWN'}
        (run / 'lidar_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return 0 if count and fault is None else 2


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
