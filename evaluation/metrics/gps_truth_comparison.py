"""Offline GPS/truth comparison; never used by the flight controller."""
import argparse
import bisect
import json
import math
from pathlib import Path
import numpy as np


def evaluate(gps, truth, failure_wall=None, max_stamp_delta=.015):
    if not math.isfinite(max_stamp_delta) or max_stamp_delta <= 0:
        raise ValueError('invalid timestamp tolerance')
    truth = sorted((r for r in truth if math.isfinite(r['stamp']) and r['stamp'] > 0),
                   key=lambda r: r['stamp'])
    stamps = [r['stamp'] for r in truth]
    pos, angle, skew = [], [], []
    for row in gps:
        stamp = row['stamp']
        if not math.isfinite(stamp) or stamp <= 0 or not stamps:
            continue
        i = bisect.bisect_left(stamps, stamp)
        candidates = truth[max(0, i-1):min(len(truth), i+1)]
        if not candidates:
            continue
        match = min(candidates, key=lambda x: abs(x['stamp']-stamp))
        delta = abs(match['stamp']-stamp)
        p, p_ref = np.asarray(row['position'], dtype=float), np.asarray(match['position'], dtype=float)
        q, q_ref = np.asarray(row['orientation'], dtype=float), np.asarray(match['orientation'], dtype=float)
        if (delta > max_stamp_delta or p.shape != (3,) or p_ref.shape != (3,)
                or q.shape != (4,) or q_ref.shape != (4,)
                or not all(np.all(np.isfinite(x)) for x in (p, p_ref, q, q_ref))):
            continue
        norm = np.linalg.norm(q)*np.linalg.norm(q_ref)
        if norm < 1e-8:
            continue
        pos.append(float(np.linalg.norm(p-p_ref)))
        angle.append(float(2*math.acos(min(1., abs(float(q @ q_ref)/norm)))))
        skew.append(delta)
    arrivals = sorted(r['arrival_wall_monotonic'] for r in gps
                      if math.isfinite(r['arrival_wall_monotonic']))
    gaps = np.diff(arrivals)
    result = {'gps_samples': len(gps), 'matched_samples': len(pos), 'unmatched_samples': len(gps)-len(pos),
              'max_allowed_stamp_delta_s': max_stamp_delta,
              'gps_after_fault_arrival_count': (sum(t > failure_wall for t in arrivals)
                                                if failure_wall is not None else None),
              'truth_use': 'offline_only', 'gps_fallback_validated': False}
    if pos:
        result.update(position_error_median_max_m=[float(np.median(pos)), max(pos)],
                      orientation_error_median_max_rad=[float(np.median(angle)), max(angle)],
                      match_stamp_delta_max_s=max(skew))
    if len(gaps):
        result['arrival_gap_median_max_s'] = [float(np.median(gaps)), float(max(gaps))]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    def rows(name):
        return [json.loads(line) for line in (args.run/name).read_text().splitlines()]
    fault_path = args.run/'fault_observation.json'
    fault = json.loads(fault_path.read_text()).get('failure_wall_monotonic') if fault_path.exists() else None
    print(json.dumps(evaluate(rows('evaluation_gps.jsonl'), rows('evaluation_truth.jsonl'), fault), indent=2))


if __name__ == '__main__':
    main()
