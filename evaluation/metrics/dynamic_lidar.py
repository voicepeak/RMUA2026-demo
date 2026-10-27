"""Audit timing and read-only policy of dynamic LiDAR diagnostics."""
from collections import Counter
import json
from pathlib import Path
import sys

from interfaces.contracts import Metadata
from perception.stereo_depth.transport import freshness_reason


def audit(run):
    run = Path(run)
    summary = json.loads((run / 'dynamic_lidar_summary.json').read_text())
    rows = [json.loads(line) for line in (run / 'dynamic_lidar_results.jsonl').read_text().splitlines()]
    errors = []
    expected = sorted(['/airsim_node/drone_1/lidar', '/airsim_node/drone_1/gps',
                       '/airsim_node/drone_1/imu/imu', '/airsim_node/initial_pose'])
    if summary.get('planning_diagnostic'): expected.append('/airsim_node/end_goal'); expected.sort()
    graph = summary['node_graph'] or {}
    if graph.get('subscriptions') != expected or set(graph.get('publishers', [])) - {'/rosout'}:
        errors.append('GRAPH')
    if (summary['faults'] or summary['control_published'] or summary['debug_inputs_used']
            or summary['control_eligible'] or summary['acceptance_eligible']
            or len(rows) != summary['results'] or dict(Counter(r['status'] for r in rows)) != summary['statuses']):
        errors.append('SUMMARY')
    for row in rows:
        if row.get('control_eligible') is not False:
            errors.append('CONTROL_ELIGIBILITY')
        planner = row.get('planner')
        if summary.get('planning_diagnostic') and row['status'] == 'PROCESSED' and planner is None:
            errors.append('PLANNER_MISSING')
        if planner and planner.get('control_eligible', False):
            errors.append('PLANNER_CONTROL_ELIGIBILITY')
        if planner and 'goal' in planner and (planner['goal']['epoch'] != row['check_epoch']
                or not 0 <= row['check_wall']-planner['goal']['last_arrival_wall'] < .5):
            errors.append('PLANNER_GOAL_STALE')
        if row['status'] != 'PROCESSED':
            continue
        meta = Metadata(row['source_stamp'], row['source_valid_until'], row['source_epoch'], 'audit')
        if freshness_reason(meta, row['check_imu_stamp'], row['check_epoch'], row['clock_fresh_at_check'],
                            row['source_arrival_wall'], row['check_wall']):
            errors.append('SOURCE_EXPIRED')
        if (row['pose_stamp'] != row['source_stamp'] or row['pose_bracket_stamps'][1] > row['check_imu_stamp']
                or row['dynamic_check']['safety_certified']):
            errors.append('POSE_OR_CERTIFICATION')
        for track in row['tracks']:
            if (track['epoch'] != row['check_epoch'] or not track['stamp'] <= row['check_imu_stamp'] < track['valid_until']
                    or track['safety_certified'] or track['control_eligible']):
                errors.append('TRACK_INVALID')
    return {'audit_valid': bool(rows and not errors), 'errors': errors,
            'rows': len(rows), 'statuses': dict(Counter(r['status'] for r in rows)),
            'rejection_reasons': dict(Counter(r['reason'] for r in rows if r['status'] == 'REJECTED')),
            'control_eligible': False, 'safety_certified': False}


def main():
    run = Path(sys.argv[1]); result = audit(run)
    (run / 'dynamic_lidar_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['audit_valid'] else 2


if __name__ == '__main__': raise SystemExit(main())
