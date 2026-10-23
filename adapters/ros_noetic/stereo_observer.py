"""Read-only front stereo header pairing; no depth or exposure-sync claim."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import threading
import time

import numpy as np

from adapters.ros_noetic.stereo_images import decode_image
from estimation.clock import SensorClock
from perception.stereo_depth.pairing import StereoPairBuffer


TOPICS = {'left': '/airsim_node/drone_1/front_left/Scene',
          'right': '/airsim_node/drone_1/front_right/Scene',
          'imu': '/airsim_node/drone_1/imu/imu'}


def observe(run, duration, image_ttl_s=.15):
    import rosgraph
    import rospy
    from sensor_msgs.msg import Image, Imu

    rospy.init_node('rmua_stereo_pairing', anonymous=True, disable_signals=True)
    pending = {}
    lock = threading.Lock()
    received = Counter()
    latest_received_imu_stamp = None

    def receive(side, message):
        nonlocal latest_received_imu_stamp
        with lock:
            if side == 'imu':
                latest_received_imu_stamp = message.header.stamp.to_sec()
            pending[side] = (message, time.monotonic(), latest_received_imu_stamp)
            received[side] += 1

    listeners = [rospy.Subscriber(topic, Imu if side == 'imu' else Image,
                 lambda msg, s=side: receive(s, msg), queue_size=1, buff_size=4000000)
                 for side, topic in TOPICS.items()]
    clock = SensorClock()
    buffer = StereoPairBuffer()
    started = time.monotonic()
    last_pair = None
    count = 0
    decoded = Counter()
    headers = {}
    fault = None
    graph = None
    try:
        with (run / 'stereo_pairs.jsonl').open('x') as output, (run / 'stereo_image_timing.jsonl').open('x') as timing:
            while time.monotonic() - started < duration and not rospy.is_shutdown():
                begin = time.monotonic()
                with lock:
                    batch = dict(pending)
                    pending.clear()
                if 'imu' in batch:
                    message, arrival, _ = batch['imu']
                    if clock.observe(message.header.stamp.to_sec(), arrival):
                        raise RuntimeError('IMU_CLOCK_RESET')
                if clock.stamp is None:
                    if begin - started > 5:
                        raise RuntimeError('NO_IMU_CLOCK')
                    time.sleep(.002)
                    continue
                if not clock.fresh(time.monotonic()):
                    raise RuntimeError('STALE_IMU_CLOCK')
                for side in ('left', 'right'):
                    if side not in batch:
                        continue
                    message, arrival, arrival_imu_stamp = batch[side]
                    decode_begin = time.monotonic()
                    frame = decode_image(message, side, clock.epoch, ttl_s=image_ttl_s)
                    accepted = buffer.add(side, frame, arrival)
                    timing.write(json.dumps({'side': side, 'stamp': frame.metadata.stamp,
                        'imu_stamp_at_callback': arrival_imu_stamp, 'imu_stamp_at_processing': clock.stamp,
                        'arrival_wall_monotonic': arrival, 'decode_begin_wall_monotonic': decode_begin,
                        'decode_end_wall_monotonic': time.monotonic(), 'epoch': clock.epoch,
                        'buffer_accepted': accepted}, allow_nan=False) + '\n')
                    decoded[side] += 1
                    if side not in headers:
                        header = {'stamp': message.header.stamp.to_sec(),
                                  'frame_id': message.header.frame_id, 'encoding': message.encoding,
                                  'width': message.width, 'height': message.height,
                                  'step': message.step, 'is_bigendian': message.is_bigendian}
                        headers[side] = header
                        (run / ('stereo_first_' + side + '.json')).write_text(json.dumps(header, indent=2) + '\n')
                        with (run / ('stereo_first_' + side + '.bin')).open('xb') as raw:
                            raw.write(bytes(message.data))
                pair = buffer.pop(clock.stamp, time.monotonic())
                if pair is not None:
                    if count == 0:
                        np.savez(run / 'stereo_first_pair.npz', left=pair.left.pixels, right=pair.right.pixels)
                    row = {'left_stamp': pair.left.metadata.stamp, 'right_stamp': pair.right.metadata.stamp,
                           'imu_clock_stamp': clock.stamp, 'pair_stamp': pair.metadata.stamp,
                           'valid_until': pair.metadata.valid_until, 'epoch': pair.metadata.epoch,
                           'header_skew_s': pair.skew_s, 'encoding': pair.left.encoding,
                           'image_shape': list(pair.left.pixels.shape),
                           'processing_begin_wall_monotonic': begin,
                           'emitted_wall_monotonic': time.monotonic(),
                           'exposure_sync_verified': False, 'calibrated_rectification_verified': False}
                    output.write(json.dumps(row, allow_nan=False) + '\n')
                    count += 1
                    last_pair = time.monotonic()
                if last_pair is None and time.monotonic() - started > 5:
                    raise RuntimeError('NO_STEREO_PAIR_WITHIN_5MS')
                if last_pair is not None and time.monotonic() - last_pair > .5:
                    raise RuntimeError('STALE_STEREO_PAIRS')
                time.sleep(.002)
    except Exception as error:
        fault = str(error)
    finally:
        try:
            pubs, subs, _ = rosgraph.Master(rospy.get_name()).getSystemState()
            node = rospy.get_name()
            graph = {'subscriptions': sorted(t for t, nodes in subs if node in nodes),
                     'publishers': sorted(t for t, nodes in pubs if node in nodes)}
            if graph['subscriptions'] != sorted(TOPICS.values()) or set(graph['publishers']) - {'/rosout'}:
                fault = fault or 'STEREO_OBSERVER_GRAPH_POLICY'
        except Exception as error:
            fault = fault or 'GRAPH_AUDIT_FAILED:' + str(error)
        for listener in listeners:
            listener.unregister()
        summary = {'pairs': count, 'decoded_frames': dict(decoded), 'received_frames': dict(received),
                   'events': dict(buffer.events), 'fault': fault, 'node_graph': graph,
                   'source_headers': headers, 'max_skew_s': buffer.max_skew_s,
                   'queue_capacity_per_side': buffer.capacity, 'clock_source': TOPICS['imu'],
                   'image_age_budget_s': image_ttl_s, 'experimental_age_budget': image_ttl_s != .15,
                   'control_published': False, 'debug_inputs_used': False,
                   'exposure_sync_verified': False, 'calibrated_rectification_verified': False,
                   'metric_depth_verified': False, 'acceptance_eligible': False}
        (run / 'stereo_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return 0 if count and fault is None else 2


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
