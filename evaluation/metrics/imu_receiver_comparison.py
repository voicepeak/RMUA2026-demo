"""Compare callback arrival times for equal recorded IMU messages, offline only."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import numpy as np


def key(row):
    try:
        stamp = row['stamp']
        accel, omega = tuple(row['acceleration_body']), tuple(row['omega_body'])
        if (len(accel) != 3 or len(omega) != 3 or
                not all(isinstance(x, (int, float)) and math.isfinite(x) for x in (stamp, *accel, *omega))):
            return None
        return stamp, accel, omega
    except (KeyError, TypeError):
        return None


def evaluate(telemetry, observed):
    index = defaultdict(list)
    for row in observed:
        identity = key(row)
        arrival = row.get('arrival_wall_monotonic')
        if identity is not None and isinstance(arrival, (int, float)) and math.isfinite(arrival):
            index[identity].append(arrival)
    relative, observed_to_publish = [], []
    missing = ambiguous = invalid = 0
    for row in telemetry:
        raw = row.get('raw', {})
        identity = key(raw)
        arrival = raw.get('imu_callback_arrival_wall_monotonic')
        publish = row.get('publish_end_wall_monotonic')
        if (identity is None or not all(isinstance(t, (int, float)) and math.isfinite(t)
                                        for t in (arrival, publish)) or arrival > publish):
            invalid += 1
            continue
        candidates = index[identity]
        if not candidates:
            missing += 1
        elif len(candidates) != 1:
            ambiguous += 1
        else:
            relative.append(arrival-candidates[0])
            observed_to_publish.append(publish-candidates[0])
    def quantiles(values):
        return [float(x) for x in np.percentile(values, (0, 50, 95, 99, 100))] if values else None
    return {'commands': len(telemetry), 'observer_messages': len(observed),
            'unique_matches': len(relative), 'missing_matches': missing,
            'ambiguous_matches': ambiguous, 'invalid_telemetry': invalid,
            'controller_minus_observer_arrival_min_p50_p95_p99_max_s': quantiles(relative),
            'observer_arrival_to_publish_end_min_p50_p95_p99_max_s': quantiles(observed_to_publish),
            'interpretation': 'Exact source stamp, acceleration and angular velocity match; duplicates excluded. Signed differences between independent subscriber callbacks on the same host, not generation or delivery latency. Same content does not prove unique physical sampling or independent upstream transport.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    def rows(name):
        return [json.loads(line) for line in (args.run/name).read_text().splitlines()]
    print(json.dumps(evaluate(rows('pwm_telemetry.jsonl'), rows('evaluation_imu.jsonl')), indent=2))


if __name__ == '__main__':
    main()
