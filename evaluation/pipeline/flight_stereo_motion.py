"""Offline flight-image VO followed by an isolated truth-only evaluation step."""
import argparse
import bisect
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from estimation.geometry import rotation
from estimation.history import slerp
from interfaces.contracts import Metadata
from perception.stereo_depth.geometry import load_nominal_rig
from perception.stereo_depth.pairing import ImageFrame, StereoPair
from perception.visual_odometry.stereo import estimate_stereo_motion


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def estimate(source, settings, method="orb"):
    """Only scene images/index and simulator settings are read in this step."""
    import cv2
    cv2.setNumThreads(1)
    source = Path(source)
    rig = load_nominal_rig(settings)
    index_path = source / 'scene_trace/index.json'
    index = json.loads(index_path.read_text())
    if index['fault'] is not None:
        raise ValueError('scene capture reports an error')
    images = {side: sorted((row for row in index['frames'] if row['sensor'] == 'front_' + side),
                           key=lambda row: row['stamp']) for side in ('left', 'right')}
    pairs, unmatched, used = [], [], set()
    files = {str(index_path): digest(index_path), str(settings): digest(Path(settings))}
    for left in images['left']:
        candidates = [(abs(left['stamp'] - row['stamp']), i, row) for i, row in enumerate(images['right']) if i not in used]
        if not candidates:
            unmatched.append(left['file']); continue
        skew, i, right = min(candidates, key=lambda item: item[0])
        if skew > .005:
            unmatched.append(left['file']); continue
        used.add(i)
        frames = []
        for side, row in (('left', left), ('right', right)):
            path = index_path.parent / row['file']
            if path.parent != index_path.parent or path.suffix != '.png':
                raise ValueError('invalid scene image path')
            data = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if data is None:
                raise ValueError('unreadable scene image')
            files[str(path)] = digest(path)
            stamp = row['stamp']
            frames.append(ImageFrame(Metadata(stamp, stamp + .2, 0, 'retained_' + side,
                                              'front_' + side + '_image'), 'bgr8', data))
        stamp = max(frame.metadata.stamp for frame in frames)
        meta = Metadata(stamp, min(frame.metadata.valid_until for frame in frames), 0,
                        'offline_pair', 'front_stereo')
        pairs.append((left['file'], StereoPair(meta, frames[0], frames[1], skew)))
    output = []
    for (previous_name, previous), (current_name, current) in zip(pairs, pairs[1:]):
        row = {'previous': previous_name, 'current': current_name,
               'previous_stamp': previous.left.metadata.stamp, 'current_stamp': current.left.metadata.stamp}
        started = time.monotonic()
        try:
            cv2.setRNGSeed(0)
            result = estimate_stereo_motion(previous, current, rig, method=method)
            row.update(status='ESTIMATED', motion=asdict(result))
        except (ValueError, cv2.error) as error:
            row.update(status='REJECTED', reason=str(error))
        row['elapsed_s'] = time.monotonic() - started
        output.append(row)
    return {'source': str(source), 'method': method, 'status_counts': dict(Counter(r['status'] for r in output)),
            'stereo_pairs': len(pairs), 'unmatched_left': unmatched,
            'unmatched_right_count': len(images['right']) - len(used), 'input_sha256': files,
            'input_role': 'official_recorded_images_and_nominal_settings',
            'truth_accessed': False, 'control_eligible': False, 'results': output}


def evaluate(estimate_result, truth_path):
    """Open truth only after estimates were saved; never feeds the VO algorithm."""
    truth_path = Path(truth_path)
    records = [json.loads(line) for line in truth_path.read_text().splitlines()]
    ordered = sorted({r['stamp']: r for r in records if r['stamp'] > 0}.values(), key=lambda row: row['stamp'])
    stamps = [r['stamp'] for r in ordered]
    def interpolate(stamp):
        i = bisect.bisect_left(stamps, stamp)
        if i == 0 or i == len(stamps): return None
        a, b = ordered[i-1], ordered[i]
        span = b['stamp'] - a['stamp']
        if span <= 0 or span > .05: return None
        fraction = (stamp - a['stamp']) / span
        position = (1-fraction)*np.array(a['position']) + fraction*np.array(b['position'])
        attitude = slerp(a['orientation'], b['orientation'], fraction)
        return position, rotation(attitude)
    result = []
    for row in estimate_result['results']:
        compared = {'previous': row['previous'], 'current': row['current'], 'status': row['status']}
        if row['status'] != 'ESTIMATED':
            compared['reason'] = row['reason']; result.append(compared); continue
        old, new = interpolate(row['previous_stamp']), interpolate(row['current_stamp'])
        if old is None or new is None:
            compared['status'] = 'TRUTH_UNAVAILABLE'; result.append(compared); continue
        delta = old[1].T @ (new[0] - old[0])
        truth_rotation = old[1].T @ new[1]
        motion = row['motion']
        predicted = np.array(motion['translation_previous_body_m'])
        residual = np.array(motion['rotation_previous_from_current_body']).T @ truth_rotation
        compared.update(status='COMPARED', truth_displacement_m=delta.tolist(),
                        truth_displacement_norm_m=float(np.linalg.norm(delta)),
                        translation_error_m=float(np.linalg.norm(predicted-delta)),
                        rotation_error_rad=float(np.arccos(np.clip((np.trace(residual)-1)/2, -1., 1.))))
        result.append(compared)
    return {'source': estimate_result['source'], 'truth_sha256': digest(truth_path),
            'role': 'isolated_post_estimate_evaluation', 'status_counts': dict(Counter(r['status'] for r in result)),
            'results': result}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    est = sub.add_parser('estimate'); est.add_argument('source'); est.add_argument('output')
    est.add_argument('--settings', default='configs/simulator_settings.json')
    est.add_argument('--method', choices=('orb', 'orb_lk', 'orb_lk_half'), default='orb')
    ev = sub.add_parser('evaluate'); ev.add_argument('estimate'); ev.add_argument('truth'); ev.add_argument('output')
    args = parser.parse_args()
    result = (estimate(args.source, args.settings, args.method) if args.command == 'estimate'
              else evaluate(json.loads(Path(args.estimate).read_text()), args.truth))
    with Path(args.output).open('x') as out: json.dump(result, out, indent=2, allow_nan=False); out.write('\n')
    print(json.dumps({'output': args.output, 'status_counts': result['status_counts']}))


if __name__ == '__main__':
    main()
