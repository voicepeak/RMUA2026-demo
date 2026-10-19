"""Offline temporal matching; truth files are opened only AFTER the control run."""
import argparse
import json
from pathlib import Path
import numpy as np


def evaluate(telemetry, truth, settled_after=4.):
    ordered = sorted({r['stamp']: r for r in truth if r['stamp'] > 0}.values(),
                     key=lambda r: r['stamp'])
    stamps = np.array([r['stamp'] for r in ordered])
    errors, tracking, attitudes = [], [], []
    matched = 0
    for row in telemetry:
        stamp = row['command']['stamp']
        index = int(np.searchsorted(stamps, stamp))
        if index == 0 or index == len(ordered):
            continue
        a, b = ordered[index-1], ordered[index]
        if b['stamp']-a['stamp'] > .05:
            continue
        t = (stamp-a['stamp'])/(b['stamp']-a['stamp'])
        position = (1-t)*np.array(a['position'])+t*np.array(b['position'])
        qa, qb = np.array(a['orientation']), np.array(b['orientation'])
        if qa @ qb < 0:
            qb = -qb
        orientation = (1-t)*qa+t*qb
        orientation /= np.linalg.norm(orientation)
        estimated = np.array(row['state']['position_ned'])
        matched += 1
        if row['wall_elapsed'] >= settled_after:
            errors.append(estimated-position)
            dot = abs(np.array(row['state']['orientation_wxyz']) @ orientation)
            attitudes.append(2*np.arccos(np.clip(dot, 0., 1.)))
            if 'reference' in row:
                tracking.append(position-np.array(row['reference']))
    timestamp_key = 'publish_wall_elapsed' if telemetry and all('publish_wall_elapsed' in r for r in telemetry) else 'wall_elapsed'
    intervals = np.diff([r[timestamp_key] for r in telemetry])
    result = {'matched_commands': matched, 'commands': len(telemetry),
              'match_fraction': matched/len(telemetry) if telemetry else 0.,
              'settled_after_wall_s': settled_after, 'settled_samples': len(errors),
              'command_timing_measurement': 'after_publish' if timestamp_key == 'publish_wall_elapsed' else 'loop_start_before_graph_query',
              'official_result': 'UNKNOWN', 'truth_use': 'offline_evaluation_only'}
    if len(intervals):
        result['command_wall_interval_p50_p95_p99_s'] = np.percentile(intervals, (50, 95, 99)).tolist()
        result['minimum_command_wall_interval_s'] = float(intervals.min())
    if errors:
        result['state_position_rmse_m'] = np.sqrt(np.mean(np.array(errors)**2, axis=0)).tolist()
        result['state_attitude_rmse_rad'] = float(np.sqrt(np.mean(np.array(attitudes)**2)))
    if tracking:
        result['truth_tracking_rmse_m'] = np.sqrt(np.mean(np.array(tracking)**2, axis=0)).tolist()
        result['truth_tracking_max_norm_m'] = float(np.linalg.norm(tracking, axis=1).max())
    result['evaluation_valid'] = result['match_fraction'] >= .9 and len(errors) >= 100
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    def rows(name):
        return [json.loads(line) for line in (args.run/name).read_text().splitlines()]
    result = evaluate(rows('pwm_telemetry.jsonl'), rows('evaluation_truth.jsonl'))
    (args.run/'closed_loop_metrics.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if result['evaluation_valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
