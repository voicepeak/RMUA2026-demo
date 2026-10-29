"""Offline finite-difference GPS attitude against recorded IMU angular rates."""
import argparse
import bisect
from collections import defaultdict
import json
import math
from pathlib import Path
import numpy as np
from estimation.geometry import multiply, unit


def relative_rate(first, second, dt):
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('invalid time interval')
    q0, q1 = unit(first), unit(second)
    q = unit(multiply(q0 * np.array([1., -1., -1., -1.]), q1))
    if q[0] < 0:
        q = -q
    length = float(np.linalg.norm(q[1:]))
    return np.zeros(3) if length < 1e-12 else q[1:] * (2*math.atan2(length, q[0])/(length*dt))


def evaluate(gps, imu, failure_wall=None):
    samples = sorted(gps, key=lambda r: r['stamp'])
    groups = defaultdict(list)
    for row in imu:
        groups[row['stamp']].append(row)
    conflicts = sorted(stamp for stamp, rows in groups.items()
                       if any(row['omega_body'] != rows[0]['omega_body'] for row in rows[1:]))
    imu = sorted((rows[0] for stamp, rows in groups.items() if stamp not in conflicts),
                 key=lambda r: r['stamp'])
    stamps = [r['stamp'] for r in imu]
    errors, estimates, references, after = [], [], [], []
    for a, b in zip(samples, samples[1:]):
        t0, t1 = a['stamp'], b['stamp']
        dt = t1-t0
        if not .07 <= dt <= .15:
            continue
        if any(t0 <= stamp <= t1 for stamp in conflicts):
            continue
        left, right = bisect.bisect_right(stamps, t0)-1, bisect.bisect_left(stamps, t1)
        if left < 0 or right >= len(imu):
            continue
        window = imu[left:right+1]
        times = np.array([r['stamp']-t0 for r in window])
        rates = np.array([r['omega_body'] for r in window], dtype=float)
        if (rates.shape != (len(window), 3) or not np.all(np.isfinite(rates))
                or np.any(np.diff(times) <= 0) or np.max(np.diff(times)) > .03):
            continue
        try:
            estimate = relative_rate(a['orientation'], b['orientation'], dt)
        except ValueError:
            continue
        grid = np.r_[0., times[(times > 0) & (times < dt)], dt]
        interpolated = np.column_stack([np.interp(grid, times, rates[:, i]) for i in range(3)])
        # Each GPS difference is an interval average, not an instantaneous rate.
        reference = np.sum((interpolated[1:]+interpolated[:-1])*.5*np.diff(grid)[:, None], axis=0)/dt
        error = float(np.linalg.norm(estimate-reference))
        errors.append(error); estimates.append(float(np.linalg.norm(estimate)))
        references.append(float(np.linalg.norm(reference)))
        if failure_wall is not None and a['arrival_wall_monotonic'] > failure_wall:
            after.append(error)
    def summary(values):
        return [float(np.median(values)), float(np.percentile(values, 95)), float(max(values))] if values else None
    return {'gps_pairs': max(0, len(samples)-1), 'matched_pairs': len(errors),
            'conflicting_imu_timestamps': len(conflicts),
            'matched_pairs_after_fault': len(after),
            'gps_rate_norm_median_p95_max_rad_s': summary(estimates),
            'imu_interval_mean_rate_norm_median_p95_max_rad_s': summary(references),
            'rate_error_norm_median_p95_max_rad_s': summary(errors),
            'after_fault_error_norm_median_p95_max_rad_s': summary(after),
            'limits': 'GPS attitude difference uses shortest quaternion rotation in initial body frame; IMU rate is an approximate interval mean, without transport compensation, clock calibration or sensor independence proof. Offline diagnostic only.',
            'gps_fallback_validated': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    def rows(name):
        return [json.loads(line) for line in (args.run/name).read_text().splitlines()]
    fault_path = args.run/'fault_observation.json'
    fault = json.loads(fault_path.read_text()).get('failure_wall_monotonic') if fault_path.exists() else None
    print(json.dumps(evaluate(rows('evaluation_gps.jsonl'), rows('evaluation_imu.jsonl'), fault), indent=2))


if __name__ == '__main__':
    main()
