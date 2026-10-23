"""Offline independent ROS-decoder and endpoint-only observation audit."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from adapters.ros_noetic.lidar_cloud import decode_xyz


def audit(run):
    import rospy
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs.point_cloud2 import read_points

    summary = json.loads((run / 'lidar_summary.json').read_text())
    schema = json.loads((run / 'lidar_first_message.json').read_text())
    message = PointCloud2()
    message.header.stamp = rospy.Time.from_sec(schema['stamp'])
    message.header.frame_id = schema['frame_id']
    for key in ('width', 'height', 'row_step', 'point_step', 'is_dense', 'is_bigendian'):
        setattr(message, key, schema[key])
    message.fields = [PointField(**f) for f in schema['fields']]
    message.data = (run / 'lidar_first_message.bin').read_bytes()
    rows = [json.loads(s) for s in (run / 'lidar_frames.jsonl').read_text().splitlines()]
    actual = decode_xyz(message, rows[0]['epoch'])
    reference = np.asarray(list(read_points(message, field_names=('x', 'y', 'z'), skip_nans=False)),
                           dtype=float).reshape(-1, 3)
    reference = reference[np.isfinite(reference).all(axis=1)]
    keys = {tuple(math.floor(float(x) / .25) for x in p) for p in reference
            if 0 < math.hypot(*p) < 30}
    snapshot = json.loads((run / 'lidar_endpoint_snapshot.json').read_text())
    cells = snapshot['cells']
    valid = (bool(rows) and summary['fault'] is None
             and len(rows) == summary['processed_frames']
             and np.array_equal(reference, actual.points)
             and len(keys) == rows[0]['occupied_voxels']
             and len(cells) == rows[-1]['occupied_voxels']
             and snapshot['metadata']['stamp'] == rows[-1]['stamp']
             and snapshot['metadata']['epoch'] == rows[-1]['epoch']
             and snapshot['metadata']['frame_id'] == 'lidar'
             and snapshot['resolution_m'] == .25
             and len({tuple(c['key']) for c in cells}) == len(cells)
             and all(c['status'] == 'OCCUPIED' and c['metadata'] == snapshot['metadata'] for c in cells)
             and all(r['free_voxels'] == 0 and not r['motion_compensated']
                     and not r['world_transform_applied'] and r['frame_id'] == 'lidar'
                     and r['input_points'] == r['finite_points'] + r['nonfinite_points']
                     and r['arrival_wall_monotonic'] <= r['processing_begin_wall_monotonic']
                     <= r['processing_end_wall_monotonic'] for r in rows)
             and not summary['control_published'] and not summary['debug_inputs_used']
             and summary['node_graph']['subscriptions'] == ['/airsim_node/drone_1/lidar']
             and not set(summary['node_graph']['publishers']) - {'/rosout'})
    durations = [r['processing_end_wall_monotonic'] - r['processing_begin_wall_monotonic'] for r in rows]
    arrivals = [r['arrival_wall_monotonic'] for r in rows]
    result = {'audit_valid': bool(valid), 'processed_frames': len(rows),
              'first_cloud_matches_ros_reference_decoder': bool(np.array_equal(reference, actual.points)),
              'first_cloud_finite_points': len(reference),
              'first_cloud_reference_endpoint_voxels': len(keys),
              'last_cloud_endpoint_voxels': len(cells),
              'processing_p50_p95_p99_max_s': [float(x) for x in np.percentile(durations, (50, 95, 99, 100))],
              'processed_arrival_span_s': arrivals[-1] - arrivals[0],
              'observed_processed_frequency_hz': (len(rows) - 1) / (arrivals[-1] - arrivals[0]) if len(rows) > 1 else None,
              'sensor_frame': 'lidar', 'world_transform_verified': False,
              'motion_compensated': False, 'free_space_certified': False,
              'flight_clearance_eligible': False, 'acceptance_eligible': False,
              'header_stamp_to_arrival_latency_calibrated': False,
              'analysis_source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (run / 'lidar_audit.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
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
