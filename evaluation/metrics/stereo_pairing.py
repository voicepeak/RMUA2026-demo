"""Offline stereo pairing audit and independent cv_bridge decoding check."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from adapters.ros_noetic.stereo_images import decode_image


def percentiles(values):
    return [float(x) for x in np.percentile(values, (50, 95, 99, 100))] if values else None


def audit(run):
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    summary = json.loads((run / 'stereo_summary.json').read_text())
    rows = [json.loads(s) for s in (run / 'stereo_pairs.jsonl').read_text().splitlines()]
    timing_path = run / 'stereo_image_timing.jsonl'
    timing = [json.loads(s) for s in timing_path.read_text().splitlines()] if timing_path.exists() else []
    matches = {}
    for side in ('left', 'right'):
        if not (run / ('stereo_first_' + side + '.json')).exists():
            matches[side] = False
            continue
        header = json.loads((run / ('stereo_first_' + side + '.json')).read_text())
        message = Image()
        message.header.stamp = rospy.Time.from_sec(header['stamp'])
        message.header.frame_id = header['frame_id']
        for key in ('width', 'height', 'step', 'encoding', 'is_bigendian'):
            setattr(message, key, header[key])
        message.data = (run / ('stereo_first_' + side + '.bin')).read_bytes()
        reference = CvBridge().imgmsg_to_cv2(message, desired_encoding='passthrough')
        if reference.ndim == 2:
            reference = reference[:, :, None]
        matches[side] = bool(np.array_equal(reference, decode_image(message, side, 0).pixels))
    graph = summary.get('node_graph') or {}
    budget = summary.get('image_age_budget_s', .15)
    graph_valid = (graph.get('subscriptions') == sorted(
                 ['/airsim_node/drone_1/front_left/Scene', '/airsim_node/drone_1/front_right/Scene',
                  '/airsim_node/drone_1/imu/imu'])
                 and not set(graph.get('publishers', [])) - {'/rosout'})
    timing_valid = (bool(timing) and len(timing) == sum(summary['decoded_frames'].values())
                    and all(r['arrival_wall_monotonic'] <= r['decode_begin_wall_monotonic']
                            <= r['decode_end_wall_monotonic'] for r in timing))
    pairs_valid = (bool(rows) and summary['pairs'] == len(rows) and summary['max_skew_s'] == .005
             and budget in (.15, .2)
             and all(0 <= r['header_skew_s'] <= .005
                     and abs(abs(r['left_stamp'] - r['right_stamp']) - r['header_skew_s']) < 1e-12
                     and r['pair_stamp'] == max(r['left_stamp'], r['right_stamp'])
                     and r['pair_stamp'] <= r['imu_clock_stamp'] < r['valid_until']
                     and r['valid_until'] == min(r['left_stamp'], r['right_stamp']) + budget
                     and r['processing_begin_wall_monotonic'] <= r['emitted_wall_monotonic']
                     and not r['exposure_sync_verified'] and not r['calibrated_rectification_verified'] for r in rows)
             and all(a[side + '_stamp'] < b[side + '_stamp'] for a, b in zip(rows, rows[1:])
                     for side in ('left', 'right'))
             )
    inputs_valid = (all(matches.values()) and timing_valid and graph_valid
                    and not summary['control_published'] and not summary['debug_inputs_used'])
    valid = inputs_valid and pairs_valid and summary['fault'] is None
    result = {'audit_valid': bool(valid), 'pairs': len(rows), 'cv_bridge_first_images_match': matches,
              'input_records_valid': bool(inputs_valid), 'pair_records_valid': bool(pairs_valid),
              'image_age_budget_s': budget, 'default_150ms_pipeline_passed': bool(valid and budget == .15),
              'source_fault': summary['fault'], 'image_timing_records': len(timing),
              'header_skew_p50_p95_p99_max_s': percentiles([r['header_skew_s'] for r in rows]),
              'pair_age_on_imu_clock_p50_p95_p99_max_s': percentiles(
                  [r['imu_clock_stamp'] - min(r['left_stamp'], r['right_stamp']) for r in rows]),
              'pair_processing_p50_p95_p99_max_s': percentiles(
                  [r['emitted_wall_monotonic'] - r['processing_begin_wall_monotonic'] for r in rows]),
              'pair_processing_interval_scope': 'Emission loop only; images may have been decoded in earlier loops. Not end-to-end latency.',
              'image_decode_p50_p95_p99_max_s': percentiles(
                  [r['decode_end_wall_monotonic'] - r['decode_begin_wall_monotonic'] for r in timing]),
              'image_header_age_at_callback_p50_p95_p99_max_s': percentiles(
                  [r['imu_stamp_at_callback'] - r['stamp'] for r in timing if r['imu_stamp_at_callback'] is not None]),
              'image_header_age_at_processing_p50_p95_p99_max_s': percentiles(
                  [r['imu_stamp_at_processing'] - r['stamp'] for r in timing]),
              'image_pending_wall_age_p50_p95_p99_max_s': percentiles(
                  [r['decode_begin_wall_monotonic'] - r['arrival_wall_monotonic'] for r in timing]),
              'image_shapes': sorted({tuple(r['image_shape']) for r in rows}),
              'exposure_sync_verified': False, 'metric_depth_verified': False,
              'acceptance_eligible': False,
              'analysis_source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (run / 'stereo_pairing_audit.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
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
