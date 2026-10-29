"""Describe debug PWM observations after a control fault; not actuator verification."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from evaluation.metrics.command_trace import decode


def evaluate(telemetry, debug, fault_wall):
    if not telemetry or not math.isfinite(fault_wall):
        raise ValueError('missing telemetry or invalid fault time')
    last = telemetry[-1]['command']['values']
    if len(last) != 4 or not all(math.isfinite(x) for x in last):
        raise ValueError('invalid last PWM')
    after = sorted((r for r in debug if r['arrival_wall_monotonic'] > fault_wall),
                   key=lambda r: r['arrival_wall_monotonic'])
    decoded = [decode(row) for row in after]
    def matches(item):
        return item is not None and all(math.isfinite(x) for x in item['values']) and all(
            abs(x-y) <= 1e-6 for x, y in zip(item['values'], last))
    suffix_start = len(after)
    while suffix_start and matches(decoded[suffix_start-1]):
        suffix_start -= 1
    return {'post_fault_records': len(after), 'undecodable_records': sum(x is None for x in decoded),
            'matching_last_command_records': sum(matches(x) for x in decoded),
            'matching_suffix_records': len(after) - suffix_start,
            'matching_suffix_first_arrival_after_fault_s': (after[suffix_start]['arrival_wall_monotonic'] - fault_wall
                                                           if suffix_start < len(after) else None),
            'last_record_arrival_after_fault_s': (after[-1]['arrival_wall_monotonic'] - fault_wall if after else None),
            'match_absolute_tolerance': 1e-6,
            'limitations': 'Debug PWM is an observed diagnostic stream, not verified motor output or physical force; callback arrival is not motor time.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    names = ['pwm_telemetry.jsonl', 'evaluation_actuation.jsonl', 'fault_observation.json']
    raw = [(args.run/name).read_bytes() for name in names]
    telemetry, debug = [[json.loads(line) for line in data.splitlines()] for data in raw[:2]]
    fault = json.loads(raw[2])
    result = evaluate(telemetry, debug, fault['failure_wall_monotonic'])
    result['input_sha256'] = {name: hashlib.sha256(data).hexdigest() for name, data in zip(names, raw)}
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
