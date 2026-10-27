"""Online read-only visual motion probe, with bounded latest-only processing."""
import argparse
from collections import Counter, OrderedDict
import json
import math
from pathlib import Path
import threading
import time

from adapters.ros_noetic.stereo_images import decode_image
from adapters.ros_noetic.stereo_observer import TOPICS
from estimation.clock import SensorClock
from perception.stereo_depth.geometry import load_nominal_rig
from perception.stereo_depth.pairing import StereoPairBuffer
from perception.stereo_depth.transport import LatestSlot
from perception.visual_odometry.online import MotionWindow


def observe(run, duration, image_ttl, settings):
    import cv2
    import rosgraph
    import rospy
    from sensor_msgs.msg import Image, Imu
    cv2.setNumThreads(1)
    rospy.init_node('rmua_visual_motion_observer', anonymous=True, disable_signals=True)
    lock = threading.Lock(); clock = SensorClock(); pending = {}; received = Counter(); overwritten = Counter()
    faults = []; slot = LatestSlot(); stop = threading.Event(); rows = []
    rig = load_nominal_rig(settings); window = MotionWindow(rig)
    def receive(side, message):
        arrival = time.monotonic()
        with lock:
            received[side] += 1
            if side == 'imu':
                try:
                    if clock.observe(message.header.stamp.to_sec(), arrival): faults.append('IMU_CLOCK_RESET')
                except ValueError as error: faults.append(str(error))
            else:
                if side in pending: overwritten[side] += 1
                pending[side] = (message, arrival)
    def snapshot():
        with lock:
            return clock.stamp, clock.epoch, clock.fresh(time.monotonic()) and not faults
    listeners = [rospy.Subscriber(topic, Imu if side == 'imu' else Image,
                 lambda msg, s=side: receive(s, msg), queue_size=1, buff_size=4000000)
                 for side, topic in TOPICS.items()]
    def worker():
        try:
            with (run / 'visual_motion_results.jsonl').open('x') as output:
                while not stop.is_set():
                    item = slot.take()
                    if item is None:
                        time.sleep(.002); continue
                    pair, arrival = item
                    started = time.monotonic()
                    row = window.process(pair, arrival, snapshot, time.monotonic)
                    row.update(processing_s=time.monotonic()-started, source_pair_stamp=pair.metadata.stamp,
                               source_epoch=pair.metadata.epoch, source_skew_s=pair.skew_s)
                    output.write(json.dumps(row, allow_nan=False) + '\n'); output.flush(); rows.append(row)
        except Exception as error:
            with lock: faults.append('WORKER:' + str(error))
    thread = threading.Thread(target=worker, daemon=True); thread.start()
    buffer = StereoPairBuffer(); arrivals = {side: OrderedDict() for side in ('left', 'right')}
    started = time.monotonic(); pairs = 0; graph = None
    try:
        while time.monotonic() - started < duration and not rospy.is_shutdown():
            with lock:
                batch = dict(pending); pending.clear(); errors = list(faults)
            if errors: raise RuntimeError(errors[0])
            now, epoch, fresh = snapshot()
            if now is None:
                if time.monotonic() - started > 5: raise RuntimeError('NO_IMU_CLOCK')
                time.sleep(.002); continue
            if not fresh: raise RuntimeError('STALE_IMU_CLOCK')
            for side, (message, arrival) in batch.items():
                frame = decode_image(message, side, epoch, image_ttl)
                if buffer.add(side, frame, arrival): arrivals[side][frame.metadata.stamp] = arrival
                while len(arrivals[side]) > 8: arrivals[side].popitem(last=False)
            pair = buffer.pop(now, time.monotonic())
            if pair is not None:
                source_arrival = min(arrivals[side][getattr(pair, side).metadata.stamp] for side in arrivals)
                slot.put((pair, source_arrival)); pairs += 1
            time.sleep(.002)
    except Exception as error:
        with lock: faults.append(str(error))
    finally:
        stop.set(); slot.close(); thread.join(10.)
        if thread.is_alive(): faults.append('WORKER_DID_NOT_STOP')
        try:
            pubs, subs, _ = rosgraph.Master(rospy.get_name()).getSystemState(); node = rospy.get_name()
            graph = {'subscriptions': sorted(t for t, nodes in subs if node in nodes),
                     'publishers': sorted(t for t, nodes in pubs if node in nodes)}
            if graph['subscriptions'] != sorted(TOPICS.values()) or set(graph['publishers']) - {'/rosout'}:
                faults.append('GRAPH_POLICY')
        except Exception as error: faults.append('GRAPH_AUDIT:' + str(error))
        for listener in listeners: listener.unregister()
        summary = {'faults': faults, 'pairs': pairs, 'received': dict(received), 'overwritten': dict(overwritten),
                   'pairing_events': dict(buffer.events), 'slot': slot.stats(), 'results': len(rows),
                   'statuses': dict(Counter(row['status'] for row in rows)),
                   'timing_current_results': sum(row.get('timing_current', False) for row in rows),
                   'node_graph': graph, 'image_age_budget_s': image_ttl,
                   'source_settings_sha256': rig.settings_sha256, 'control_published': False,
                   'debug_inputs_used': False, 'control_eligible': False, 'acceptance_eligible': False}
        (run / 'visual_motion_summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    return 0 if pairs and rows and not faults else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--image-age-budget-ms', type=int, choices=(150, 200), default=150)
    parser.add_argument('--settings', default='/workspace/main/configs/simulator_settings.json')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 30: parser.error('duration must be in (0,30]')
    return observe(args.run_dir, args.duration, args.image_age_budget_ms/1000., args.settings)


if __name__ == '__main__':
    raise SystemExit(main())
