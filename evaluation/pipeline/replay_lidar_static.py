"""Replay recorded joint sensor capture through the read-only static probe."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path

from adapters.ros_noetic.capture_io import load_message
from adapters.ros_noetic.lidar_cloud import decode_xyz
from estimation.history import PoseHistory
from evaluation.metrics.pose_history import state_from_json
from evaluation.pipeline.lidar_static import LidarStaticProbe
from interfaces.contracts import TrajectorySample
from perception.stereo_depth.geometry import load_nominal_rig
from planning.spacetime_local.static_candidates import LocalPlannerLimits
from planning.trajectory_validator.static_sweep import SweepBounds


def replay(source, settings):
    source = Path(source)
    rig = load_nominal_rig(settings)
    manifest = json.loads((source / 'manifest.json').read_text())
    if manifest['simulator_preflight']['settings_sha256'] != rig.settings_sha256:
        raise ValueError('settings do not match recorded capture')
    summary = json.loads((source / 'joint_summary.json').read_text())
    states = [json.loads(line) for line in (source / 'joint_states.jsonl').read_text().splitlines()]
    history = PoseHistory()
    limits = LocalPlannerLimits(SweepBounds((.1, .1, .1), .02, 2.), 1.,
                                segment_durations_s=(.05,), lateral_offsets_m=())
    probe = LidarStaticProbe(history, rig.lidar_pose, limits)
    index, rows = 0, []
    for capture in summary['captures']:
        directory = source / capture['directory']
        packet = json.loads((directory / 'packet.json').read_text())
        now, wall = packet['query_now'], packet['query_wall']
        while index < len(states) and states[index]['arrival_wall_monotonic'] <= wall:
            record = states[index]
            state = state_from_json(record['state'])
            if state.metadata.stamp > now:
                break
            history.append(state, record['arrival_wall_monotonic'])
            index += 1
        sources = history.snapshot_sources()
        if not sources:
            rows.append({'capture': capture['directory'], 'reason': 'NO_SOURCE_STATE'})
            continue
        state = sources[-1][0]
        cloud = decode_xyz(load_message(directory / 'lidar.json'), packet['epoch'])
        ref = TrajectorySample(now, state.position_ned, state.velocity_ned, (0., 0., 0.), 0.)
        goal = tuple(a + b for a, b in zip(state.position_ned, (.0001, 0., 0.)))
        result = probe.evaluate(cloud, state, ref, goal, now, wall, packet['epoch'])
        rows.append({'capture': capture['directory'], 'scan_stamp': cloud.metadata.stamp,
                     'query_now': now, 'result': asdict(result)})
    return {'source': str(source), 'role': 'read_only_diagnostic', 'control_published': False,
            'current_control_eligible': False, 'captures': rows}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source')
    parser.add_argument('--settings', default='configs/simulator_settings.json')
    args = parser.parse_args()
    print(json.dumps(replay(args.source, args.settings), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
