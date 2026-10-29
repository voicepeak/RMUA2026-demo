"""Offline angular-rate arrival timing near a recorded fault; not a recovery simulation."""
import argparse
import json
import math
from pathlib import Path


def evaluate(imu, telemetry, failure_wall):
    if not math.isfinite(failure_wall):
        raise ValueError('invalid fault time')
    window = sorted((r for r in imu if failure_wall-.25 <= r['arrival_wall_monotonic'] <= failure_wall+.08),
                    key=lambda r: r['arrival_wall_monotonic'])
    crossings = {}
    for threshold in (.35, 1., 2.5):
        event = next((r for r in window if len(r['omega_body']) == 3
                      and all(math.isfinite(x) for x in r['omega_body'])
                      and math.sqrt(sum(x*x for x in r['omega_body'])) > threshold), None)
        if event is None:
            crossings[str(threshold)] = None
            continue
        arrival = event['arrival_wall_monotonic']
        subsequent = [r for r in telemetry if arrival <= r['publish_end_wall_monotonic'] < failure_wall]
        crossings[str(threshold)] = {'arrival_minus_fault_s': arrival-failure_wall,
            'source_stamp': event['stamp'], 'raw_angular_rate_norm_rad_s': math.sqrt(sum(x*x for x in event['omega_body'])),
            'published_commands_after_arrival_before_fault': len(subsequent)}
    return {'window_wall_s': [-.25, .08], 'recorded_imu_samples_in_window': len(window),
            'crossings': crossings, 'recovery_tested': False,
            'limitations': 'First raw IMU sample above each threshold in this window; independent observer arrival is not controller arrival or actuator response. Sample gaps, bias and prior threshold crossings are not resolved; no counterfactual recovery or causal claim.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    def rows(name):
        return [json.loads(line) for line in (args.run/name).read_text().splitlines()]
    fault = json.loads((args.run/'fault_observation.json').read_text())
    print(json.dumps(evaluate(rows('evaluation_imu.jsonl'), rows('pwm_telemetry.jsonl'),
                              fault['failure_wall_monotonic']), indent=2))


if __name__ == '__main__':
    main()
