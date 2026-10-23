"""Replay retained ESKF history and camera timestamp lookups without truth."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np

from estimation.history import HistoryUnavailable, PoseHistory
from interfaces.contracts import Health, Metadata, StateEstimate


def state_from_json(data):
    metadata = dict(data['metadata'])
    metadata['covariance_or_bounds'] = tuple(metadata['covariance_or_bounds'])
    return StateEstimate(Metadata(**metadata), tuple(data['position_ned']), tuple(data['velocity_ned']),
        tuple(data['orientation_wxyz']), tuple(data['angular_velocity_body']), Health(data['health']))


def audit(run):
    rows = lambda name: [json.loads(s) for s in (run / name).read_text().splitlines()]
    sources = rows('history_states.jsonl')
    queries = rows('camera_history.jsonl')
    summary = json.loads((run / 'pose_history_summary.json').read_text())
    history = PoseHistory()
    index = matched = agreed = 0
    divergences = []
    brackets = []
    for number, row in enumerate(queries):
        while index < len(sources) and sources[index]['state']['metadata']['stamp'] <= row['query_now']:
            source = sources[index]
            state = state_from_json(source['state'])
            if state.metadata.epoch != history.epoch:
                history.reset(state.metadata.epoch)
            history.append(state, source['arrival_wall_monotonic'])
            index += 1
        expected = None
        reason = None
        try:
            expected = history.lookup(row['image_stamp'], row['query_now'], row['query_wall_monotonic'], row['epoch'])
        except HistoryUnavailable as error:
            reason = str(error)
        encoded = json.loads(json.dumps(asdict(expected))) if expected else None
        same = encoded == row['historical_pose'] and reason == row['reason']
        if same:
            agreed += 1
        else:
            divergences.append(number)
        if expected:
            matched += 1
            brackets.append(expected.bracket_stamps[1] - expected.bracket_stamps[0])
    expected_inputs = sorted(('/airsim_node/initial_pose', '/airsim_node/drone_1/gps',
                             '/airsim_node/drone_1/imu/imu', '/airsim_node/drone_1/front_left/Scene'))
    graph = summary.get('node_graph') or {}
    valid = (bool(queries) and matched > 0 and agreed == len(queries) and summary['fault'] is None
             and len(queries) == summary['images'] and matched == summary['historical_matches']
             and len(sources) == summary['source_states']
             and graph.get('subscriptions') == expected_inputs
             and not set(graph.get('publishers', [])) - {'/rosout'}
             and not summary['control_published'] and not summary['debug_inputs_used']
             and all(not r['current_control_eligible'] and not r['world_projection_applied'] for r in queries))
    def percentiles(values):
        return [float(x) for x in np.percentile(values, (50, 95, 99, 100))] if values else None
    result = {'audit_valid': bool(valid), 'source_states': len(sources), 'camera_queries': len(queries),
              'replayed_historical_matches': matched, 'queries_agreeing_with_replay': agreed,
              'divergent_query_indices': divergences, 'unavailable_reasons': summary['unavailable_reasons'],
              'match_fraction': matched / len(queries) if queries else 0.,
              'header_age_p50_p95_p99_max_s': percentiles([r['query_now'] - r['image_stamp'] for r in queries]),
              'lookup_elapsed_p50_p95_p99_max_s': percentiles([r['lookup_elapsed_wall_s'] for r in queries]),
              'maximum_interpolation_bracket_s': max(brackets) if brackets else None,
              'current_control_eligible': False, 'historical_uncertainty_calibrated': False,
              'image_exposure_time_calibrated': False, 'world_projection_verified': False,
              'acceptance_eligible': False,
              'analysis_source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (run / 'pose_history_audit.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    result = audit(args.run)
    print(json.dumps(result, indent=2))
    return 0 if result['audit_valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
