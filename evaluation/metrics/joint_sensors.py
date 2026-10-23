"""Audit exact same-scene inputs, replay poses and compare nominal projections.

Disparity residuals against sparse LiDAR are diagnostic: sparse visibility,
exposure alignment, metric extrinsics and per-point scan time are unverified.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np

from adapters.ros_noetic.capture_io import load_message
from adapters.ros_noetic.lidar_cloud import decode_xyz
from adapters.ros_noetic.stereo_images import decode_image
from estimation.history import HistoricalPose, PoseHistory
from evaluation.metrics.pose_history import state_from_json
from interfaces.contracts import Metadata
from perception.stereo_depth.geometry import load_nominal_rig
from perception.stereo_depth.lidar_projection import project_lidar_into_stereo, sparse_visible, sample_disparity
from perception.stereo_depth.pairing import StereoPair
from perception.stereo_depth.sgbm import compute_diagnostic_disparity


def restore_pose(data):
    if data['frame_id'] != 'world_ned' or data['current_control_eligible'] or data['uncertainty_calibrated']:
        raise ValueError('unexpected historical pose eligibility')
    return HistoricalPose(data['stamp'], data['epoch'], tuple(data['position_ned']),
        tuple(data['orientation_wxyz']), tuple(data['bracket_stamps']),
        tuple(tuple(d) for d in data['bracket_covariance_diagonals']), data['interpolation_fraction'],
        data['position_correction_residual_m'], data['rotation_correction_residual_rad'])


def audit(run):
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.point_cloud2 import read_points

    cv2.setNumThreads(1)
    manifest = json.loads((run / 'manifest.json').read_text())
    summary = json.loads((run / 'joint_summary.json').read_text())
    source_rows = [json.loads(s) for s in (run / 'joint_states.jsonl').read_text().splitlines()]
    raw_rows = [json.loads(s) for s in (run / 'joint_raw_sensors.jsonl').read_text().splitlines()]
    rig = load_nominal_rig(Path(__file__).resolve().parents[2] / 'configs/simulator_settings.json')
    if rig.settings_sha256 != manifest['simulator_preflight']['settings_sha256']:
        raise ValueError('nominal settings differ from the captured scene')
    history = PoseHistory()
    source_index = 0
    packets = []
    used_stamps = {tag: set() for tag in ('left','right','lidar')}
    valid = bool(summary['captures']) and summary['fault'] is None and summary['raw_queue_drops'] == 0
    for entry in summary['captures']:
        relative = Path(entry['directory'])
        if relative.is_absolute() or '..' in relative.parts:raise ValueError('invalid packet directory')
        directory = run / relative
        packet_path = directory / 'packet.json'
        if hashlib.sha256(packet_path.read_bytes()).hexdigest() != entry['packet_sha256']:
            raise ValueError('joint packet changed')
        packet = json.loads(packet_path.read_text())
        epoch = packet['epoch']
        messages = {tag: load_message(directory / (tag + '.json')) for tag in used_stamps}
        exact_images = True
        frames = {}
        for side in ('left','right'):
            frames[side] = decode_image(messages[side], side, epoch, packet['image_age_budget_s'])
            independent = CvBridge().imgmsg_to_cv2(messages[side], desired_encoding='passthrough')
            if independent.ndim == 2:independent = independent[:, :, None]
            exact_images &= np.array_equal(independent, frames[side].pixels)
        cloud = decode_xyz(messages['lidar'], epoch)
        reference = np.asarray(list(read_points(messages['lidar'], field_names=('x','y','z'), skip_nans=False)), dtype=float).reshape(-1,3)
        reference = reference[np.isfinite(reference).all(axis=1)]
        exact_cloud = np.array_equal(reference, cloud.points)
        while source_index < len(source_rows) and source_rows[source_index]['state']['metadata']['stamp'] <= packet['query_now']:
            row = source_rows[source_index];state = state_from_json(row['state'])
            if state.metadata.epoch != history.epoch:history.reset(state.metadata.epoch)
            history.append(state, row['arrival_wall_monotonic']);source_index += 1
        poses = {}
        pose_equal = True
        for tag, stamp in packet['stamps'].items():
            pose = history.lookup(stamp, packet['query_now'], packet['query_wall'], epoch)
            pose_equal &= json.loads(json.dumps(asdict(pose))) == packet['poses'][tag]
            poses[tag] = restore_pose(packet['poses'][tag])
            valid &= stamp == messages[tag].header.stamp.to_sec() and stamp not in used_stamps[tag]
            used_stamps[tag].add(stamp)
        def metadata(data):
            data = dict(data);data['covariance_or_bounds'] = tuple(data['covariance_or_bounds'])
            return Metadata(**data)
        ml, mr, mc = (metadata(packet[tag + '_source_metadata']) for tag in ('left','right','lidar'))
        source_meta_equal = (ml == frames['left'].metadata and mr == frames['right'].metadata and mc == cloud.metadata)
        timing_valid = (ml.current(packet['selection_now'], epoch) and mr.current(packet['selection_now'], epoch)
            and abs(ml.stamp-mr.stamp) <= .005
            and max(abs(mc.stamp-ml.stamp), abs(mc.stamp-mr.stamp)) <= .02
            and abs(ml.stamp-mr.stamp) == packet['stereo_header_skew_s']
            and abs(mc.stamp-ml.stamp) == packet['left_lidar_header_skew_s']
            and abs(mc.stamp-mr.stamp) == packet['right_lidar_header_skew_s']
            and mc.current(packet['selection_now'],epoch) == packet['lidar_current_at_selection'])
        pair = StereoPair(Metadata(max(ml.stamp,mr.stamp),min(ml.valid_until,mr.valid_until),epoch,
                                  'retained_joint_stereo','front_stereo'),frames['left'],frames['right'],abs(ml.stamp-mr.stamp))
        disparity = compute_diagnostic_disparity(pair)
        luv,lz,ruv,rz,in_view = project_lidar_into_stereo(cloud.points,poses['lidar'],poses['left'],poses['right'],rig)
        visible = (sparse_visible(luv,lz,np.isfinite(luv).all(axis=1),rig.left.width,rig.left.height)
                   & sparse_visible(ruv,rz,np.isfinite(ruv).all(axis=1),rig.right.width,rig.right.height))
        sampled, compared = sample_disparity(luv,disparity.disparity_px,disparity.valid,visible)
        expected = np.full(len(luv),np.nan)
        expected[in_view] = luv[in_view,0]-ruv[in_view,0]
        compared[compared] = expected[compared] > 0
        residual = np.full(len(luv),np.nan)
        residual[compared] = sampled[compared]-expected[compared]
        np.savez_compressed(directory/'projection_diagnostics.npz', left_uv=luv,right_uv=ruv,
            in_view=in_view,sparse_visible=visible,comparison_valid=compared,
            expected_disparity_px=expected,measured_disparity_px=sampled,residual_px=residual)
        values = residual[compared]
        packet_valid = bool(exact_images and exact_cloud and pose_equal and source_meta_equal and timing_valid
            and not packet['current_control_eligible'] and not packet['per_point_motion_compensation_applied']
            and not packet['exposure_sync_verified'])
        valid &= packet_valid
        packets.append({'directory':entry['directory'],'input_records_valid':packet_valid,
            'images_match_cv_bridge':bool(exact_images),'cloud_matches_ros_decoder':bool(exact_cloud),
            'poses_match_source_replay':bool(pose_equal),'metadata_matches_sources':bool(source_meta_equal),
            'header_timing_valid':bool(timing_valid),'left_lidar_header_skew_s':abs(ml.stamp-mc.stamp),
            'right_lidar_header_skew_s':abs(mr.stamp-mc.stamp),'lidar_points':len(cloud.points),
            'points_in_both_views':int(in_view.sum()),'sparse_visible_points':int(visible.sum()),
            'comparison_points':int(compared.sum()),
            'disparity_residual_p10_p50_p90_px':[float(v) for v in np.percentile(values,(10,50,90))] if len(values) else None,
            'absolute_disparity_residual_median_px':float(np.median(np.abs(values))) if len(values) else None,
            'estimated_header_pose_alignment_applied':True,'per_point_motion_compensation_applied':False,
            'metric_calibration_verified':False,'visibility_verified':False,'real_scene_accuracy_verified':False})
    graph = summary.get('node_graph') or {}
    expected_inputs = sorted(('/airsim_node/initial_pose','/airsim_node/drone_1/gps','/airsim_node/drone_1/imu/imu',
        '/airsim_node/drone_1/front_left/Scene','/airsim_node/drone_1/front_right/Scene','/airsim_node/drone_1/lidar'))
    valid &= (graph.get('subscriptions') == expected_inputs and not set(graph.get('publishers',[]))-{'/rosout'}
              and not summary['control_published'] and not summary['debug_inputs_used']
              and len(raw_rows)==summary['raw_sensor_records'] and len(source_rows)==summary['source_states'])
    raw_counts = {tag:sum(r['sensor']==tag for r in raw_rows) for tag in ('imu','gps')}
    valid &= all(raw_counts[tag]==summary['received'].get(tag,0) for tag in raw_counts)
    imu_stamps = [r['stamp'] for r in raw_rows if r['sensor']=='imu']
    deltas = np.diff(imu_stamps)
    result = {'input_audit_valid':bool(valid),'captures':len(packets),'packets':packets,
        'source_states':len(source_rows),'raw_sensor_records':raw_counts,'raw_queue_drops':summary['raw_queue_drops'],
        'raw_recording_is_lossy':True,'raw_imu_max_stamp_gap_s':float(deltas.max()) if len(deltas) else None,
        'raw_imu_nonincreasing_records':int((deltas<=0).sum()),
        'nominal_settings_sha256':rig.settings_sha256,'exposure_sync_verified':False,
        'per_point_motion_compensation_applied':False,'metric_calibration_verified':False,
        'sparse_visibility_is_complete_proof':False,'world_projection_verified':False,
        'current_control_eligible':False,'acceptance_eligible':False,
        'interpretation':'Payload and history audit is distinct from geometry accuracy. Residuals compare diagnostic SGBM with nominal LiDAR projection after estimated header-pose alignment; sparse z-buffers do not prove visibility. Point timing, exposure and calibration remain unverified.',
        'analysis_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (run/'joint_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path)
    args = parser.parse_args();result = audit(args.run)
    print(json.dumps(result,indent=2));return 0 if result['input_audit_valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
