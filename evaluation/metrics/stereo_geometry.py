"""Historical sparse epipolar checks and nominal configuration triangulation."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from interfaces.contracts import Metadata
from perception.stereo_depth.pairing import ImageFrame
from perception.stereo_depth.geometry import load_nominal_rig
from perception.stereo_depth.epipolar import measure_sparse_epipolar


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def benchmark(run):
    import cv2

    root = Path('/workspace/main')
    manifest = json.loads((run / 'manifest.json').read_text())
    config = manifest['config']['geometry_experiment']
    source, settings = root / config['source_run'], root / config['settings_file']
    before = {name: digest(source / name) for name in config['input_sha256']}
    if before != config['input_sha256'] or digest(settings) != config['settings_sha256']:
        raise ValueError('historical image input or nominal settings changed')
    rig = load_nominal_rig(settings)
    with (source / 'stereo_pairs.jsonl').open() as stream:
        row = json.loads(stream.readline())
    summary = json.loads((source / 'stereo_summary.json').read_text())
    lifetime = summary.get('image_age_budget_s', .15)
    with np.load(source / 'stereo_first_pair.npz', allow_pickle=False) as saved:
        def image(side):
            stamp = row[side + '_stamp']
            return ImageFrame(Metadata(stamp, stamp + lifetime, row['epoch'], 'retained_front_' + side,
                                      'front_' + side + '_image'), row['encoding'], saved[side])
        left_frame, right_frame = image('left'), image('right')
    if (left_frame.pixels.shape[:2] != (rig.left.height, rig.left.width)
            or right_frame.pixels.shape[:2] != (rig.right.height, rig.right.width)):
        raise ValueError('saved image shape differs from nominal settings')
    def gray(frame):
        if frame.encoding == 'mono8':
            return frame.pixels[:, :, 0].copy()
        conversion = {'bgr8': cv2.COLOR_BGR2GRAY, 'rgb8': cv2.COLOR_RGB2GRAY,
                      'bgra8': cv2.COLOR_BGRA2GRAY, 'rgba8': cv2.COLOR_RGBA2GRAY}[frame.encoding]
        return cv2.cvtColor(frame.pixels, conversion)
    left_gray, right_gray = gray(left_frame), gray(right_frame)
    cv2.setNumThreads(1)
    outputs, durations = [], []
    for _ in range(2):
        cv2.setRNGSeed(0)
        begin = time.monotonic()
        outputs.append(measure_sparse_epipolar(left_gray, right_gray))
        durations.append(time.monotonic() - begin)
    report, left, right, inliers, fundamental = outputs[0]
    repeated = outputs[1]
    repeat_same = (report == repeated[0] and np.array_equal(left, repeated[1])
                   and np.array_equal(right, repeated[2]) and np.array_equal(inliers, repeated[3])
                   and ((fundamental is None and repeated[4] is None)
                        or np.array_equal(fundamental, repeated[4])))
    body, nominal_valid, sensitivity = rig.triangulate_body(left, right, max_depth_m=30., max_vertical_px=1.)
    nominal_valid &= inliers
    body[~nominal_valid] = np.nan
    sensitivity[~nominal_valid] = np.nan
    z = rig.left.body_to_optical(body[nominal_valid])[:, 2]
    np.savez_compressed(run / 'stereo_geometry_points.npz', left_uv=left, right_uv=right,
        fundamental_inliers=inliers, nominal_body_frd_m=body, nominal_valid=nominal_valid,
        nominal_depth_sensitivity_m_per_px=sensitivity)
    def quantiles(values):
        return [float(v) for v in np.percentile(values, (10, 50, 90))] if len(values) else None
    after = {name: digest(source / name) for name in before}
    report.update({'source_run': config['source_run'], 'historical_input_only': True,
        'opencv_version': cv2.__version__, 'opencv_threads': cv2.getNumThreads(), 'rng_seed': 0,
        'input_sha256': before, 'input_unchanged': before == after,
        'settings_unchanged': digest(settings) == config['settings_sha256'], 'nominal_rig': asdict(rig),
        'compute_durations_s': durations, 'repeat_deterministic': bool(repeat_same),
        'fundamental_matrix': fundamental.tolist() if fundamental is not None else None,
        'nominal_local_triangulated_points': int(nominal_valid.sum()),
        'nominal_optical_depth_p10_p50_p90_m': quantiles(z),
        'nominal_depth_sensitivity_p10_p50_p90_m_per_px': quantiles(sensitivity[nominal_valid]),
        'unknown_nominal_points_are_nan': bool(np.isnan(body[~nominal_valid]).all()),
        'exposure_sync_verified': False, 'real_scene_metric_accuracy_measured': False,
        'current_control_eligible': False, 'world_projection_applied': False,
        'interpretation': 'Sparse feature agreement supports an epipolar investigation, not full rectification or metric calibration. Nominal triangulation uses the public settings model. Depth sensitivity is an analytic derivative, not calibrated uncertainty. No ground-truth, LiDAR comparison, occupancy clearing or control use.'})
    (run / 'stereo_geometry_report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    return 0 if repeat_same and report['input_unchanged'] and report['settings_unchanged'] else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    return benchmark(args.run_dir)


if __name__ == '__main__':
    raise SystemExit(main())
