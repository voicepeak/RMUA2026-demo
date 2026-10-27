"""Audit recorded online visual-motion timing and subscription boundaries."""
import json
import sys
from collections import Counter
from pathlib import Path

from perception.stereo_depth.transport import freshness_reason
from interfaces.contracts import Metadata


def audit(run):
    run = Path(run)
    summary = json.loads((run / 'visual_motion_summary.json').read_text())
    rows = [json.loads(line) for line in (run / 'visual_motion_results.jsonl').read_text().splitlines()]
    errors = []
    if len(rows) != summary['results'] or Counter(row['status'] for row in rows) != Counter(summary['statuses']):
        errors.append('ROW_COUNT')
    for row in rows:
        if row['control_eligible'] or row['source_skew_s'] > .005:
            errors.append('SOURCE_OR_AUTHORITY')
        if row['status'] in ('ESTIMATED', 'GATE_CANDIDATES'):
            metadata = Metadata(row['source_stamp'], row['source_valid_until'], row['source_epoch'], 'audit')
            reason = freshness_reason(metadata, row['check_imu_stamp'], row['check_epoch'],
                row['clock_fresh_at_check'], row['source_arrival_wall'], row['check_wall'])
            if reason != row['timing_rejection'] or row['timing_current'] != (reason is None):
                errors.append('TIMING')
            if row['status'] == 'ESTIMATED' and (row['motion']['current_stamp'] != row['source_pair_stamp'] or row['motion']['control_eligible']):
                errors.append('MOTION_IDENTITY')
            if row['status'] == 'GATE_CANDIDATES' and any(c['metadata']['stamp'] != row['source_stamp'] or c['metadata']['valid_until'] != row['source_valid_until'] or c['control_eligible'] for c in row['candidates']):
                errors.append('GATE_IDENTITY')
    graph = summary.get('node_graph') or {}
    expected = sorted(['/airsim_node/drone_1/front_left/Scene', '/airsim_node/drone_1/front_right/Scene',
                       '/airsim_node/drone_1/imu/imu'])
    if graph.get('subscriptions') != expected or set(graph.get('publishers', [])) - {'/rosout'}:
        errors.append('GRAPH')
    if summary['faults'] or summary['control_published'] or summary['debug_inputs_used']:
        errors.append('FAULT_OR_POLICY')
    timed = sum(row.get('timing_current', False) for row in rows)
    if timed != summary['timing_current_results']:
        errors.append('TIMING_COUNT')
    return {'audit_valid': bool(rows and not errors), 'errors': errors, 'rows': len(rows),
            'timing_current_results': timed, 'status_counts': dict(Counter(r['status'] for r in rows)),
            'control_eligible': False, 'acceptance_eligible': False}


def main():
    run = Path(sys.argv[1]); result = audit(run)
    (run / 'visual_motion_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['audit_valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
