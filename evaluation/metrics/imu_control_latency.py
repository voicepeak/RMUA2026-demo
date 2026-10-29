"""Offline local callback-to-PWM timing; not sensor-to-actuator latency."""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def evaluate(rows):
    arrival_to_state, state_to_publish, arrival_to_publish = [], [], []
    missing = invalid = 0
    for row in rows:
        try:
            arrival = row['raw']['imu_callback_arrival_wall_monotonic']
            state = row['raw']['state_snapshot_wall_monotonic']
            begin = row['publish_begin_wall_monotonic']
            end = row['publish_end_wall_monotonic']
        except KeyError:
            missing += 1
            continue
        if not all(isinstance(t, (int, float)) and math.isfinite(t) for t in (arrival, state, begin, end)) \
                or not arrival <= state <= begin <= end:
            invalid += 1
            continue
        arrival_to_state.append(state-arrival)
        state_to_publish.append(end-state)
        arrival_to_publish.append(end-arrival)
    def quantiles(values):
        return [float(x) for x in np.percentile(values, (50, 95, 99, 100))] if values else None
    return {'commands': len(rows), 'valid_samples': len(arrival_to_publish), 'missing_samples': missing,
            'invalid_samples': invalid,
            'arrival_to_state_p50_p95_p99_max_s': quantiles(arrival_to_state),
            'state_to_publish_end_p50_p95_p99_max_s': quantiles(state_to_publish),
            'arrival_to_publish_end_p50_p95_p99_max_s': quantiles(arrival_to_publish),
            'timing_complete': bool(rows) and not missing and not invalid,
            'interpretation': 'Local monotonic callback entry through state snapshot and publish return; excludes source generation, ROS pre-callback delay, backend receipt and actuator application.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    rows = [json.loads(line) for line in (args.run/'pwm_telemetry.jsonl').read_text().splitlines()]
    print(json.dumps(evaluate(rows), indent=2))


if __name__ == '__main__':
    main()
