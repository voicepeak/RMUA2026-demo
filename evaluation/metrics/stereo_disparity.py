"""Offline pixel-disparity benchmark on retained official-camera input only."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from interfaces.contracts import Metadata
from perception.stereo_depth.pairing import ImageFrame, StereoPair
from perception.stereo_depth.sgbm import SgbmConfig, compute_diagnostic_disparity


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def benchmark(run):
    import cv2

    manifest = json.loads((run / 'manifest.json').read_text())
    config = manifest['config']['disparity_experiment']
    source = Path('/workspace/main') / config['source_run']
    before = {name: digest(source / name) for name in config['input_sha256']}
    if before != config['input_sha256']:
        raise ValueError('retained stereo input changed')
    summary = json.loads((source / 'stereo_summary.json').read_text())
    with (source / 'stereo_pairs.jsonl').open() as stream:
        row = json.loads(stream.readline())
    lifetime = summary.get('image_age_budget_s', .15)
    with np.load(source / 'stereo_first_pair.npz', allow_pickle=False) as saved:
        def image(side):
            stamp = row[side + '_stamp']
            return ImageFrame(Metadata(stamp, stamp + lifetime, row['epoch'],
                    'retained_official_front_' + side, 'front_' + side + '_image'),
                    row['encoding'], saved[side])
        pair = StereoPair(Metadata(row['pair_stamp'], row['valid_until'], row['epoch'],
                          'retained_stereo_pair', 'front_stereo'), image('left'), image('right'), row['header_skew_s'])
    settings = SgbmConfig()
    cv2.setNumThreads(1)
    compute_diagnostic_disparity(pair, settings)  # Separate warmup, not timed.
    durations, hashes = [], []
    for _ in range(3):
        begin = time.monotonic()
        result = compute_diagnostic_disparity(pair, settings)
        durations.append(time.monotonic() - begin)
        hashes.append(hashlib.sha256(result.disparity_px.tobytes() + result.valid.tobytes()).hexdigest())
    after = {name: digest(source / name) for name in before}
    if after != before:
        raise ValueError('retained stereo input changed during benchmark')
    np.savez_compressed(run / 'diagnostic_disparity.npz', disparity_px=result.disparity_px, valid=result.valid)
    values = result.disparity_px[result.valid]
    report = {'computed': True, 'historical_input_only': True, 'opencv_version': cv2.__version__,
              'opencv_threads': cv2.getNumThreads(), 'settings': asdict(settings),
              'input_shape': list(pair.left.pixels.shape), 'source_run': config['source_run'],
              'input_sha256': before, 'input_unchanged': before == after,
              'warmup_runs': 1, 'measured_runs': 3, 'compute_durations_s': durations,
              'compute_median_s': float(np.median(durations)), 'compute_max_s': max(durations),
              'output_sha256_per_repeat': hashes, 'repeat_deterministic': len(set(hashes)) == 1,
              'valid_pixels': int(result.valid.sum()), 'pixels': int(result.valid.size),
              'valid_fraction': float(result.valid.mean()),
              'disparity_p10_p50_p90_px': [float(v) for v in np.percentile(values, (10, 50, 90))] if len(values) else None,
              'source_metadata': asdict(result.metadata),
              'left_measurement_stamp': result.left_measurement_stamp,
              'right_measurement_stamp': result.right_measurement_stamp,
              'unknown_pixels_are_nan': bool(np.isnan(result.disparity_px[~result.valid]).all()),
              'rectification_verified': False, 'exposure_sync_verified': False,
              'metric_depth_verified': False, 'real_scene_accuracy_measured': False,
              'online_latency_verified': False, 'acceptance_eligible': False,
              'interpretation': 'Pixel disparity under unverified rectification. Coverage is not accuracy. Compute timing excludes input decoding/transport, simulator contention and GPU inference. Original validity is preserved, not refreshed by offline processing.'}
    (run / 'stereo_disparity_report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    return 0 if report['repeat_deterministic'] and report['unknown_pixels_are_nan'] else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    return benchmark(args.run_dir)


if __name__ == '__main__':
    raise SystemExit(main())
