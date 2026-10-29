"""Offline startup replay; hypothetical GPS position seeding is not a flight policy."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from estimation.imu_gps import ImuGpsEstimator


def replay(summary, imu, gps):
    info = summary['startup_diagnostics']
    accounting = summary['startup_velocity']
    if not accounting['sealed'] or accounting['gps_events_dropped'] or any(accounting['bootstrap_delta']):
        raise ValueError('requires untruncated, non-bootstrap startup')
    start, end = info['first_initialized_imu_stamp'], accounting['stamp_at_seal']
    selected = [row for row in imu if start <= row['stamp'] <= end]
    selected.sort(key=lambda row: row['stamp'])
    stamps = [row['stamp'] for row in selected]
    if (len(selected) != accounting['imu_updates'] or not selected or stamps[0] != start or stamps[-1] != end
            or any(a >= b for a, b in zip(stamps, stamps[1:]))):
        raise ValueError('independent IMU records do not match startup count and bounds')
    events = accounting['gps_events']
    if len(events) != accounting['gps_updates'] or not events:
        raise ValueError('missing accepted GPS events')
    event_by_state = {}
    for event in events:
        matches = [r for r in gps if r['stamp'] == event['gps_stamp'] and r['position'] == event['gps_position']]
        if len(matches) != 1 or event['state_stamp'] not in stamps:
            raise ValueError('GPS or state stamp is ambiguous or missing')
        event_by_state.setdefault(event['state_stamp'], []).append((event, matches[0]))
    output = {}
    for mode in ('recorded_initial_pose', 'hypothetical_first_gps_position'):
        estimator = ImuGpsEstimator()
        estimator.initialize(info['initial_position'], info['initial_orientation_wxyz'])
        seeded = False
        for row in selected:
            estimator.propagate(row['stamp'], row['acceleration_body'], row['omega_body'])
            for event, sample in event_by_state.get(row['stamp'], []):
                if mode == 'hypothetical_first_gps_position' and not seeded:
                    # Counterfactual only: assumes this first GPS position is a valid
                    # same-frame initial position, which has NOT been established.
                    estimator.p = np.asarray(sample['position'], dtype=float).copy()
                    seeded = True
                if not estimator.update_gps(event['gps_stamp'], sample['position'], sample['orientation']):
                    raise ValueError('GPS rejected in replay: ' + mode)
        output[mode] = {'position_at_seal': estimator.p.tolist(), 'velocity_at_seal': estimator.v.tolist()}
    expected = np.asarray(accounting['velocity_at_seal'])
    residual = np.asarray(output['recorded_initial_pose']['velocity_at_seal']) - expected
    output['recorded_velocity_residual_m_s'] = residual.tolist()
    output['replay_agrees_with_recorded_velocity'] = bool(np.linalg.norm(residual) < 1e-8)
    output['imu_records'] = len(selected)
    output['gps_events'] = len(events)
    output['limitations'] = ('Independent receiver rows matched by timestamp and payload, not the controller callback stream; '
        'agreement of final velocity does not establish all internal states. Hypothetical first-GPS seeding assumes an '
        'unverified common frame and trusted GPS; no outlier, control, actuator or safety behavior is tested.')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    names = ['pwm_summary.json', 'evaluation_imu.jsonl', 'evaluation_gps.jsonl']
    raw = [(args.run / name).read_bytes() for name in names]
    report = replay(json.loads(raw[0]), *[[json.loads(line) for line in x.splitlines()] for x in raw[1:]])
    report['input_sha256'] = {n: hashlib.sha256(x).hexdigest() for n, x in zip(names, raw)}
    report['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
