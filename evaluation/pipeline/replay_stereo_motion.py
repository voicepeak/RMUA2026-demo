"""Replay retained stereo captures through nominal stereo visual odometry."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

from adapters.ros_noetic.capture_io import read_payload
from estimation.geometry import rotation
from interfaces.contracts import Metadata
from perception.stereo_depth.geometry import load_nominal_rig
from perception.stereo_depth.pairing import ImageFrame, StereoPair
from perception.visual_odometry.stereo import estimate_stereo_motion


def read_pair(directory, epoch):
    frames = []
    for side in ('left', 'right'):
        info, payload = read_payload(directory / (side + '.json'))
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(info['encoding'])
        if channels is None or info['step'] < info['width'] * channels or len(payload) != info['step'] * info['height']:
            raise ValueError('unsupported recorded image')
        pixels = np.ndarray((info['height'], info['width'], channels), dtype=np.uint8,
                            buffer=payload, strides=(info['step'], channels, 1))
        stamp = info['stamp']
        frames.append(ImageFrame(Metadata(stamp, stamp + .2, epoch, 'retained_' + side,
                                          'front_' + side + '_image'), info['encoding'], pixels))
    left, right = frames
    stamp = max(left.metadata.stamp, right.metadata.stamp)
    meta = Metadata(stamp, min(left.metadata.valid_until, right.metadata.valid_until), epoch,
                    'retained_stereo', 'front_stereo')
    return StereoPair(meta, left, right, abs(left.metadata.stamp - right.metadata.stamp))


def replay(source, settings):
    import cv2
    cv2.setNumThreads(1)
    source = Path(source)
    rig = load_nominal_rig(settings)
    manifest = json.loads((source / 'manifest.json').read_text())
    if manifest['simulator_preflight']['settings_sha256'] != rig.settings_sha256:
        raise ValueError('settings do not match recorded capture')
    captures = json.loads((source / 'joint_summary.json').read_text())['captures']
    frames = []
    for capture in captures:
        directory = source / capture['directory']
        packet = json.loads((directory / 'packet.json').read_text())
        frames.append((capture['directory'], read_pair(directory, packet['epoch']), packet['poses']['left']))
    results = []
    for (old_dir, old, old_pose), (new_dir, new, new_pose) in zip(frames, frames[1:]):
        start = time.monotonic()
        record = {'previous': old_dir, 'current': new_dir, 'gap_s': new.metadata.stamp - old.metadata.stamp}
        try:
            cv2.setRNGSeed(0)
            motion = estimate_stereo_motion(old, new, rig)
            elapsed = time.monotonic() - start
            prev_rotation = rotation(old_pose['orientation_wxyz'])
            next_rotation = rotation(new_pose['orientation_wxyz'])
            expected = prev_rotation.T @ (np.asarray(new_pose['position_ned']) - old_pose['position_ned'])
            observed_rotation = np.asarray(motion.rotation_previous_from_current_body)
            difference = observed_rotation.T @ prev_rotation.T @ next_rotation
            angle = float(np.arccos(np.clip((np.trace(difference) - 1.) / 2., -1., 1.)))
            record.update(status='ESTIMATED', result=asdict(motion), elapsed_s=elapsed,
                          eskf_delta_previous_body_m=expected.tolist(),
                          translation_difference_m=float(np.linalg.norm(np.asarray(motion.translation_previous_body_m) - expected)),
                          rotation_difference_rad=angle)
        except (ValueError, cv2.error) as error:
            record.update(status='REJECTED', reason=str(error), elapsed_s=time.monotonic() - start)
        results.append(record)
    return {'source': str(source), 'role': 'offline_diagnostic', 'comparison': 'imu_gps_eskf_not_truth',
            'control_eligible': False, 'counts': dict(Counter(row['status'] for row in results)), 'results': results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('--settings', default='configs/simulator_settings.json')
    args = parser.parse_args()
    print(json.dumps(replay(args.source, args.settings), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
