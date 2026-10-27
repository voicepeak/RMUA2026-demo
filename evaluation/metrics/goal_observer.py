"""Reconstruct and audit official goal observations."""
from dataclasses import asdict
import json
from pathlib import Path
import sys

from planning.mission.goal_tracker import GoalTracker


def audit(run):
    run = Path(run)
    summary = json.loads((run / 'goal_summary.json').read_text())
    rows = [json.loads(s) for s in (run / 'goal_observations.jsonl').read_text().splitlines()]
    tracker = GoalTracker(); errors = []
    for row in rows:
        if row['epoch'] != tracker.epoch:
            tracker.reset(row['epoch'])
        result = tracker.observe(tuple(row['position']), row['source_stamp'], row['source_frame'],
                                 row['arrival_wall'], row['imu_stamp'], row['epoch'])
        if json.loads(json.dumps(asdict(result))) != row['snapshot']:
            errors.append('REPLAY_MISMATCH')
    graph = summary['node_graph'] or {}
    if (graph.get('subscriptions') != sorted(['/airsim_node/end_goal','/airsim_node/drone_1/imu/imu'])
            or set(graph.get('publishers', [])) - {'/rosout'}): errors.append('GRAPH')
    if (summary['faults'] or summary['control_published'] or summary['debug_inputs_used']
            or summary['stage_completion_inferred'] or len(rows) != summary['rows']): errors.append('SUMMARY')
    return {'audit_valid': bool(rows and not errors), 'errors': errors, 'rows': len(rows),
            'source_stamp_zero_count': sum(row['source_stamp'] == 0 for row in rows),
            'official_stage_confirmed': False, 'control_eligible': False}


def main():
    run = Path(sys.argv[1]); result = audit(run)
    (run / 'goal_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['audit_valid'] else 2


if __name__ == '__main__': raise SystemExit(main())
