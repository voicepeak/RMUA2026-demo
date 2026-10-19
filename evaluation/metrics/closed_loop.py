"""Offline temporal matching; truth files are opened only AFTER the control run."""
import argparse
import json
from pathlib import Path
import numpy as np


def evaluate(telemetry, truth, settled_after=4.):
    ordered = sorted({r['stamp']: r for r in truth if r['stamp'] > 0}.values(),
                     key=lambda r: r['stamp'])
    stamps = np.array([r['stamp'] for r in ordered])
    errors, tracking, attitudes, yaw_errors, imu_attitudes = [], [], [], [], []
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
            raw_q=row.get('raw',{}).get('imu_orientation_wxyz')
            if raw_q is not None:
                q=np.asarray(raw_q,dtype=float)
                norm=np.linalg.norm(q)
                if q.shape==(4,) and np.all(np.isfinite(q)) and abs(norm-1)<.01:
                    dot=abs(q @ orientation/norm)
                    imu_attitudes.append(2*np.arccos(np.clip(dot,0.,1.)))
            if 'reference' in row:
                tracking.append(position-np.array(row['reference']))
            if 'yaw_reference' in row:
                w,x,y,z=orientation
                yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
                yaw_errors.append((yaw-row['yaw_reference']+np.pi) % (2*np.pi)-np.pi)
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
    if yaw_errors:
        result['truth_yaw_tracking_rmse_rad']=float(np.sqrt(np.mean(np.array(yaw_errors)**2)))
        result['truth_yaw_tracking_max_abs_rad']=float(np.abs(yaw_errors).max())
    if imu_attitudes:
        result['imu_orientation_unit_valid_fraction']=len(imu_attitudes)/len(errors)
        result['imu_orientation_truth_rmse_rad']=float(np.sqrt(np.mean(np.array(imu_attitudes)**2)))
        result['imu_orientation_fused_in_estimator']=False
    result['evaluation_valid'] = result['match_fraction'] >= .9 and len(errors) >= 100
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    def rows(name):
        return [json.loads(line) for line in (args.run/name).read_text().splitlines()]
    manifest = json.loads((args.run/'manifest.json').read_text())
    mode = manifest['config'].get('control_mode','PWM')
    telemetry=rows('vel_telemetry.jsonl' if mode == 'VEL' else 'pwm_telemetry.jsonl')
    result = evaluate(telemetry, rows('evaluation_truth.jsonl'))
    result['control_mode'] = mode
    if manifest['config'].get('experiment',{}).get('braking'):
        from evaluation.metrics.braking import measure_braking
        result['braking_response']=measure_braking(telemetry,rows('evaluation_truth.jsonl'))
    if mode == 'VEL':
        samples=[r for r in telemetry if 5.4 <= r.get('sim_elapsed',0) < 6.8
                 and r.get('yaw_rate_reference',0) > 0 and abs(r['command']['values'][3]) > 1.]
        if len(samples)>20:
            command=np.array([r['command']['values'][3] for r in samples])
            omega=np.array([r['raw']['omega_body'][2] for r in samples])
            gain=float(command @ omega/(command @ command))
            result['vel_yaw_response_gain_rad_per_command_unit']=gain
            result['vel_yaw_expected_rad_per_degree']=float(np.pi/180)
            result['vel_yaw_degrees_response_within_20pct']=abs(gain/(np.pi/180)-1) < .2
    (args.run/'closed_loop_metrics.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if result['evaluation_valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
