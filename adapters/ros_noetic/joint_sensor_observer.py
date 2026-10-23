"""Bounded same-scene stereo/LiDAR/raw IMU-GPS diagnostic recorder."""
import argparse
from collections import Counter, OrderedDict, deque
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import threading
import time

from adapters.ros_noetic.capture_io import save_message
from adapters.ros_noetic.flight_sensors import FlightSensors
from adapters.ros_noetic.lidar_cloud import decode_xyz
from adapters.ros_noetic.stereo_images import decode_image
from estimation.history import HistoryUnavailable, PoseHistory
from perception.sensor_sync import LidarAssociationBuffer
from perception.stereo_depth.pairing import StereoPairBuffer


TOPICS = {'left': '/airsim_node/drone_1/front_left/Scene',
          'right': '/airsim_node/drone_1/front_right/Scene', 'lidar': '/airsim_node/drone_1/lidar'}
INPUTS = sorted(tuple(TOPICS.values()) + ('/airsim_node/initial_pose',
                '/airsim_node/drone_1/gps', '/airsim_node/drone_1/imu/imu'))


def raw_record(tag, message, arrival):
    def vector(v):return (v.x, v.y, v.z)
    def quaternion(q):return (q.w, q.x, q.y, q.z)
    record = {'sensor': tag, 'stamp': message.header.stamp.to_sec(),
              'stamp_secs': message.header.stamp.secs, 'stamp_nsecs': message.header.stamp.nsecs,
              'sequence': message.header.seq, 'frame_id': message.header.frame_id,
              'arrival_wall_monotonic': arrival}
    if tag == 'imu':
        record.update(acceleration_body=vector(message.linear_acceleration),
                      omega_body=vector(message.angular_velocity), orientation_wxyz=quaternion(message.orientation),
                      acceleration_covariance=tuple(message.linear_acceleration_covariance),
                      omega_covariance=tuple(message.angular_velocity_covariance),
                      orientation_covariance=tuple(message.orientation_covariance))
    else:
        record.update(position=vector(message.pose.position), orientation_wxyz=quaternion(message.pose.orientation))
    finite = True
    def sanitize(value):
        nonlocal finite
        if isinstance(value, (tuple, list)):return [sanitize(v) for v in value]
        if isinstance(value, float) and not math.isfinite(value):
            finite = False
            return None
        return value
    record = {key: sanitize(value) for key, value in record.items()}
    record['raw_fields_finite'] = finite
    return record


def observe(run, duration, image_ttl_s):
    import rosgraph
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image, Imu, PointCloud2

    rospy.init_node('rmua_joint_sensors', anonymous=True, disable_signals=True)
    history = PoseHistory()
    sensors = FlightSensors(pose_history=history)
    pairs = StereoPairBuffer()
    clouds = LidarAssociationBuffer()
    lock = threading.Lock()
    pending, last_received = {}, {}
    raw_pending = deque()
    received = Counter()
    raw_drops = 0

    def receive(tag, message):
        with lock:
            arrival = time.monotonic()
            pending[tag] = (message, arrival)
            last_received[tag] = arrival
            received[tag] += 1

    def receive_raw(tag, message):
        nonlocal raw_drops
        with lock:
            if len(raw_pending) == 512:
                raw_pending.popleft()
                raw_drops += 1
            raw_pending.append((tag, message, time.monotonic()))
            received[tag] += 1

    listeners = [rospy.Subscriber(topic, PointCloud2 if tag == 'lidar' else Image,
                 lambda msg, t=tag: receive(t, msg), queue_size=1, buff_size=4000000)
                 for tag, topic in TOPICS.items()]
    listeners += [rospy.Subscriber('/airsim_node/drone_1/imu/imu', Imu,
                                  lambda msg: receive_raw('imu', msg), queue_size=1),
                  rospy.Subscriber('/airsim_node/drone_1/gps', PoseStamped,
                                   lambda msg: receive_raw('gps', msg), queue_size=1)]
    original = {tag: OrderedDict() for tag in TOPICS}
    packet_root = run / 'joint'
    packet_root.mkdir(exist_ok=False)
    captures, reasons = [], Counter()
    source_count = raw_count = stereo_count = 0
    dumped = None
    last_capture = None
    fault = graph = None
    started = time.monotonic()
    with (run / 'joint_states.jsonl').open('x') as states, (run / 'joint_raw_sensors.jsonl').open('x') as raw:
        def dump_sources():
            nonlocal source_count, raw_count, dumped
            for state, arrival in history.snapshot_sources():
                key = (state.metadata.epoch, state.metadata.stamp)
                if dumped is None or key > dumped:
                    states.write(json.dumps({'state': asdict(state), 'arrival_wall_monotonic': arrival}, allow_nan=False) + '\n')
                    dumped = key
                    source_count += 1
            with lock:
                batch = list(raw_pending)
                raw_pending.clear()
            for tag, message, arrival in batch:
                raw.write(json.dumps(raw_record(tag, message, arrival), allow_nan=False) + '\n')
                raw_count += 1

        try:
            while time.monotonic() - started < duration and not rospy.is_shutdown():
                dump_sources()
                with sensors.lock:
                    if sensors.fault:raise RuntimeError(sensors.fault)
                    ready = sensors.estimator.initialized and sensors.raw is not None
                    if ready and not sensors.clock.fresh(time.monotonic()):raise RuntimeError('STALE_IMU')
                    now, epoch = sensors.clock.stamp, sensors.clock.epoch
                with lock:
                    batch = dict(pending);pending.clear()
                    latest = dict(last_received)
                if time.monotonic() - started > 5:
                    if not ready or set(latest) != set(TOPICS):raise RuntimeError('JOINT_STREAM_NOT_READY')
                    if any(time.monotonic() - t > .5 for t in latest.values()):raise RuntimeError('STALE_JOINT_STREAM')
                    if not captures:raise RuntimeError('NO_JOINT_CAPTURE_WITHIN_LIMITS')
                if not ready:
                    time.sleep(.003)
                    continue
                for tag, (message, arrival) in batch.items():
                    if tag == 'lidar':
                        decoded = decode_xyz(message, epoch)
                        accepted = clouds.add(decoded, arrival)
                    else:
                        decoded = decode_image(message, tag, epoch, ttl_s=image_ttl_s)
                        accepted = pairs.add(tag, decoded, arrival)
                    if accepted:
                        original[tag][decoded.metadata.stamp] = message
                        while len(original[tag]) > 8:original[tag].popitem(last=False)
                with sensors.lock:
                    if sensors.fault:raise RuntimeError(sensors.fault)
                    now, epoch = sensors.clock.stamp, sensors.clock.epoch
                pair = pairs.pop(now, time.monotonic())
                if pair is None:
                    time.sleep(.003)
                    continue
                stereo_count += 1
                if len(captures) >= 6 or (last_capture is not None and time.monotonic() - last_capture < 1.5):
                    continue
                selection_wall = time.monotonic()
                association = clouds.select(pair, now, selection_wall, epoch)
                if association is None:
                    reasons['NO_LIDAR_ASSOCIATION'] += 1
                    continue
                stamps = {'left': pair.left.metadata.stamp, 'right': pair.right.metadata.stamp,
                          'lidar': association.cloud.metadata.stamp}
                try:
                    with sensors.lock:
                        if sensors.fault:raise RuntimeError(sensors.fault)
                        query_now, query_wall = sensors.clock.stamp, time.monotonic()
                        poses = {tag: history.lookup(stamp, query_now, query_wall, epoch) for tag, stamp in stamps.items()}
                except HistoryUnavailable as error:
                    reasons['HISTORY:' + str(error)] += 1
                    continue
                messages = {tag: original[tag].get(stamp) for tag, stamp in stamps.items()}
                if any(message is None for message in messages.values()):
                    reasons['RAW_MESSAGE_EVICTED'] += 1
                    continue
                directory = packet_root / str(len(captures)).zfill(4)
                directory.mkdir(exist_ok=False)
                for tag, message in messages.items():
                    save_message(directory, tag, message, 'lidar' if tag == 'lidar' else 'image')
                packet = {'stamps': stamps, 'epoch': epoch, 'query_now': query_now, 'query_wall': query_wall,
                          'selection_now': now, 'selection_wall': selection_wall,
                          'poses': {tag: asdict(pose) for tag, pose in poses.items()},
                          'stereo_header_skew_s': pair.skew_s,
                          'left_lidar_header_skew_s': association.left_skew_s,
                          'right_lidar_header_skew_s': association.right_skew_s,
                          'left_source_metadata': asdict(pair.left.metadata),
                          'right_source_metadata': asdict(pair.right.metadata),
                          'lidar_source_metadata': asdict(association.cloud.metadata),
                          'lidar_current_at_selection': association.lidar_current_at_selection,
                          'image_age_budget_s': image_ttl_s, 'capture_wall': time.monotonic(),
                          'current_control_eligible': False, 'per_point_motion_compensation_applied': False,
                          'exposure_sync_verified': False, 'world_projection_applied': False}
                packet_path = directory / 'packet.json'
                packet_path.write_text(json.dumps(packet, indent=2, allow_nan=False) + '\n')
                captures.append({'directory': str(directory.relative_to(run)),
                    'packet_sha256': hashlib.sha256(packet_path.read_bytes()).hexdigest()})
                last_capture = time.monotonic()
            if not captures:raise RuntimeError('NO_JOINT_CAPTURES')
        except Exception as error:
            fault = str(error)
        finally:
            try:
                pubs, subs, _ = rosgraph.Master(rospy.get_name()).getSystemState();node = rospy.get_name()
                graph = {'subscriptions': sorted(t for t, nodes in subs if node in nodes),
                         'publishers': sorted(t for t, nodes in pubs if node in nodes)}
                if graph['subscriptions'] != INPUTS or set(graph['publishers']) - {'/rosout'}:
                    fault = fault or 'JOINT_OBSERVER_GRAPH_POLICY'
            except Exception as error:
                fault = fault or 'GRAPH_AUDIT_FAILED:' + str(error)
            for listener in listeners:listener.unregister()
            sensors.close()
            dump_sources()
    summary = {'captures': captures, 'stereo_pairs': stereo_count, 'received': dict(received),
               'source_states': source_count, 'raw_sensor_records': raw_count, 'raw_queue_drops': raw_drops,
               'rejections': dict(reasons), 'stereo_events': dict(pairs.events), 'lidar_events': dict(clouds.events),
               'fault': fault, 'node_graph': graph, 'image_age_budget_s': image_ttl_s,
               'max_stereo_skew_s': .005, 'max_lidar_skew_s': clouds.max_skew_s,
               'raw_recording_is_lossy': True, 'payloads_retained_exactly_for_selected_captures': True,
               'control_published': False, 'debug_inputs_used': False,
               'per_point_motion_compensation_applied': False, 'acceptance_eligible': False}
    (run / 'joint_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return 0 if captures and fault is None and not raw_drops else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--image-age-budget-ms', type=int, choices=(150, 200), default=150)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 30:
        parser.error('duration must be in (0,30] seconds')
    return observe(args.run_dir, args.duration, args.image_age_budget_ms / 1000.)


if __name__ == '__main__':
    raise SystemExit(main())
