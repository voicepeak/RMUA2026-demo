"""Offline truth observations during a precontrol bootstrap; not ground-contact proof."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def evaluate(bootstrap, truth):
    start, end = bootstrap['imu_window_start_stamp'], bootstrap['imu_window_end_stamp']
    if not (math.isfinite(start) and math.isfinite(end) and 0 < start < end):
        raise ValueError('invalid bootstrap window')
    unique = {}
    for row in truth:
        stamp = row['stamp']
        if not math.isfinite(stamp) or not start <= stamp <= end:
            continue
        p = np.asarray(row['position'], dtype=float)
        q = np.asarray(row['orientation'], dtype=float)
        if p.shape != (3,) or q.shape != (4,) or not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)):
            raise ValueError('invalid truth pose')
        norm = np.linalg.norm(q)
        if not math.isfinite(norm) or norm < 1e-8:
            raise ValueError('invalid truth orientation')
        value = (p, q/norm)
        if stamp in unique:
            old = unique[stamp]
            if not np.array_equal(old[0], p) or not np.allclose(old[1], value[1], atol=1e-12, rtol=0):
                raise ValueError('conflicting duplicate truth stamp')
        else:
            unique[stamp] = value
    stamps = sorted(unique)
    if len(stamps) < 2:
        return {'usable': False, 'reason': 'INSUFFICIENT_TRUTH', 'samples': len(stamps)}
    gaps = np.diff(stamps)
    start_gap, end_gap = stamps[0]-start, end-stamps[-1]
    if start_gap > .03 or end_gap > .03 or np.max(gaps) > .05:
        return {'usable': False, 'reason': 'TRUTH_COVERAGE_GAP', 'samples': len(stamps),
                'start_gap_s': start_gap, 'end_gap_s': end_gap, 'max_sample_gap_s': float(np.max(gaps))}
    positions = np.array([unique[t][0] for t in stamps])
    orientations = np.array([unique[t][1] for t in stamps])
    distance = np.linalg.norm(positions-positions[0], axis=1)
    angles = 2*np.arccos(np.clip(np.abs(orientations @ orientations[0]), 0., 1.))
    return {'usable': True, 'samples': len(stamps), 'window_source_stamp': [start, end],
            'truth_source_stamp': [stamps[0], stamps[-1]], 'start_gap_s': start_gap,
            'end_gap_s': end_gap, 'max_sample_gap_s': float(np.max(gaps)),
            'endpoint_displacement_m': float(np.linalg.norm(positions[-1]-positions[0])),
            'max_displacement_from_first_m': float(np.max(distance)),
            'max_attitude_change_rad': float(np.max(angles)),
            'ground_contact_verified': False,
            'limitations': 'Offline truth stream source time and frame are uncalibrated; sampled poses can miss motion, ground contact and physical velocity.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    names = ['stationary_bootstrap.json', 'evaluation_truth.jsonl']
    raw = [(args.run/name).read_bytes() for name in names]
    report = evaluate(json.loads(raw[0]), [json.loads(line) for line in raw[1].splitlines()])
    report['input_sha256'] = {name: hashlib.sha256(data).hexdigest() for name, data in zip(names, raw)}
    report['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    print(json.dumps(report, indent=2))
    return 0 if report['usable'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
