"""Prepare image-only inputs and independently evaluate nominal sparse residuals."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from evaluation.seed_runner.manifest import file_hash
from perception.stereo_depth.lidar_projection import sample_disparity


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def prepare(run):
    from adapters.ros_noetic.capture_io import load_message
    from adapters.ros_noetic.lidar_cloud import decode_xyz
    from adapters.ros_noetic.stereo_images import decode_image
    from evaluation.metrics.joint_sensors import restore_pose
    from perception.stereo_depth.geometry import load_nominal_rig
    from perception.stereo_depth.learned import rgb_pixels
    from perception.stereo_depth.lidar_projection import project_lidar_into_stereo, sparse_visible

    root = Path(__file__).resolve().parents[2]
    manifest = json.loads((run/'manifest.json').read_text())
    rig = load_nominal_rig(root/'configs/simulator_settings.json')
    (run/'inputs').mkdir(); (run/'references').mkdir()
    records = []
    for source_name in manifest['config']['learned_stereo']['source_runs']:
        source = root/source_name
        original = json.loads((source/'manifest.json').read_text())
        if original['simulator_preflight']['settings_sha256'] != rig.settings_sha256:
            raise ValueError('captured geometry settings differ')
        summary = json.loads((source/'joint_summary.json').read_text())
        audit = json.loads((source/'joint_audit.json').read_text())
        if not audit['input_audit_valid'] or summary['fault']:
            raise ValueError('source capture has not passed the independent payload audit')
        for entry in summary['captures']:
            directory = source/entry['directory']
            packet = json.loads((directory/'packet.json').read_text())
            if file_hash(directory/'packet.json') != entry['packet_sha256']:
                raise ValueError('capture packet changed')
            frames = {side: decode_image(load_message(directory/(side+'.json')), side,
                packet['epoch'], packet['image_age_budget_s']) for side in ('left','right')}
            for side, frame in frames.items():
                if json.loads(json.dumps(asdict(frame.metadata))) != packet[side+'_source_metadata']:
                    raise ValueError('source metadata changed')
            key = '%04d' % len(records)
            np.savez_compressed(run/'inputs'/(key+'.npz'),
                left=rgb_pixels(frames['left']), right=rgb_pixels(frames['right']))
            # Recompute all sparse nominal references, without consulting SGBM masks.
            cloud = decode_xyz(load_message(directory/'lidar.json'), packet['epoch'])
            poses = {tag: restore_pose(data) for tag, data in packet['poses'].items()}
            luv,lz,ruv,rz,in_view = project_lidar_into_stereo(cloud.points,
                poses['lidar'],poses['left'],poses['right'],rig)
            visible = (sparse_visible(luv,lz,np.isfinite(luv).all(axis=1),rig.left.width,rig.left.height)
                & sparse_visible(ruv,rz,np.isfinite(ruv).all(axis=1),rig.right.width,rig.right.height))
            expected = np.full(len(luv),np.nan)
            expected[in_view] = luv[in_view,0]-ruv[in_view,0]
            candidates = visible & in_view & np.isfinite(expected)
            candidates[candidates] = expected[candidates] > 0
            np.savez_compressed(run/'references'/(key+'.npz'),
                left_uv=luv, expected_disparity_px=expected, candidates=candidates)
            records.append({'key':key,'source_run':source_name,'packet_directory':entry['directory'],
                'seed':original['seed'],'left_metadata':packet['left_source_metadata'],
                'right_metadata':packet['right_source_metadata'], 'epoch':packet['epoch'],
                'image_sha256':file_hash(run/'inputs'/(key+'.npz')),
                'reference_sha256':file_hash(run/'references'/(key+'.npz'))})
    write_json(run/'inputs'/'index.json', records)
    return records


def residual_stats(values):
    values = np.asarray(values)
    return {'count':int(len(values)),
        'absolute_median_px':float(np.median(np.abs(values))) if len(values) else None,
        'absolute_p90_px':float(np.percentile(np.abs(values),90)) if len(values) else None,
        'signed_median_px':float(np.median(values)) if len(values) else None}


def compare_reference(uv, expected, candidates, fields):
    """No residual rejection: compare own support and the exact shared support."""
    sampled, masks = {}, {}
    for name, (disparity, valid) in fields.items():
        sampled[name], masks[name] = sample_disparity(uv, disparity, valid, candidates)
    shared = np.logical_and.reduce(list(masks.values()))
    return {'reference_candidates':int(candidates.sum()), 'methods':{
        name:{'own_support':residual_stats(sampled[name][masks[name]]-expected[masks[name]]),
              'shared_support':residual_stats(sampled[name][shared]-expected[shared])}
        for name in fields}}, masks


def evaluate(run):
    records = json.loads((run/'inputs'/'index.json').read_text())
    runtime = json.loads((run/'predictions'/'runtime.json').read_text())
    packets, pooled = [], {method:[] for method in ('sgbm','ffs')}
    for record in records:
        key = record['key']
        reference_path, image_path = run/'references'/(key+'.npz'), run/'inputs'/(key+'.npz')
        if (file_hash(reference_path) != record['reference_sha256']
                or file_hash(image_path) != record['image_sha256']):
            raise ValueError('evaluation inputs changed')
        with np.load(reference_path,allow_pickle=False) as ref, np.load(run/'predictions'/(key+'.npz'),allow_pickle=False) as pred:
            fields = {name:(pred[name+'_disparity'],pred[name+'_valid']) for name in ('sgbm','ffs')}
            comparison, masks = compare_reference(ref['left_uv'],ref['expected_disparity_px'],ref['candidates'],fields)
            shared = masks['sgbm'] & masks['ffs']
            for name,(disp,valid) in fields.items():
                if not (np.isnan(disp[~valid]).all() and np.isfinite(disp[valid]).all() and (disp[valid]>0).all()):
                    raise ValueError('invalid unknown or valid disparity pixels')
                samples,_ = sample_disparity(ref['left_uv'],disp,valid,ref['candidates'])
                pooled[name].extend((samples[shared]-ref['expected_disparity_px'][shared]).tolist())
            comparison.update(key=key,seed=record['seed'],source_run=record['source_run'],
                packet_directory=record['packet_directory'],
                source_metadata={'left':record['left_metadata'],'right':record['right_metadata']},
                prediction_sha256=file_hash(run/'predictions'/(key+'.npz')),
                coverage={name:float(valid.mean()) for name,(_,valid) in fields.items()},
                raw_ffs_positive_fraction=float((np.isfinite(pred['ffs_forward'])&(pred['ffs_forward']>0)).mean()),
                ffs_lr_only_fraction=float(pred['ffs_lr_valid'].mean()))
            packets.append(comparison)
    report = {'computed':True,'packets':packets,'runtime':runtime,
        'pooled_shared_residuals':{name:residual_stats(values) for name,values in pooled.items()},
        'historical_only':True,'reference_type':'nominal_lidar_after_estimated_header_pose_alignment',
        'reference_selection_uses_disparity_or_residual':False,
        'development_samples_not_heldout':True,'metric_calibration_verified':False,
        'exposure_sync_verified':False,'per_point_motion_compensation_applied':False,
        'sparse_visibility_is_complete_proof':False,'real_scene_accuracy_verified':False,
        'confidence_calibrated':False,'online_latency_verified':False,
        'current_control_eligible':False,'acceptance_eligible':False,
        'interpretation':'Nominal residuals are not ground-truth accuracy. Shared-support errors use identical reference points without residual filtering. Coverage is not confidence or FREE. Timing is offline without simulator contention or ROS transport. Original source leases are retained.'}
    write_json(run/'learned_stereo_report.json',report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--stage',choices=('prepare','evaluate'),required=True)
    args = parser.parse_args()
    prepare(args.run_dir) if args.stage=='prepare' else evaluate(args.run_dir)


if __name__ == '__main__':
    main()
