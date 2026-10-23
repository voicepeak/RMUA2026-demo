"""Independent numerical agreement and nominal reference comparison, not calibration."""
import argparse
import json
from pathlib import Path

import numpy as np

from evaluation.metrics.learned_stereo import compare_reference


def numerical_agreement(reference,reference_valid,candidate,candidate_valid):
    common=reference_valid & candidate_valid;union=reference_valid | candidate_valid
    errors=np.abs(reference[common]-candidate[common])
    iou=float(common.sum()/union.sum()) if union.any() else 0.
    p99=float(np.percentile(errors,99)) if len(errors) else None
    return {'common_pixels':int(common.sum()),'mask_iou':iou,
        'absolute_median_px':float(np.median(errors)) if len(errors) else None,
        'absolute_p99_px':p99,'absolute_max_px':float(errors.max()) if len(errors) else None,
        'min_iou_gate':.9,'p99_error_gate_original_px':1.,
        'compatible':bool(len(errors) and np.isfinite(errors).all() and iou>=.9 and p99<=1.)}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args();run=args.run_dir;root=Path(__file__).resolve().parents[2]
    manifest=json.loads((run/'manifest.json').read_text());source=root/manifest['config']['trt_experiment']['source_run']
    records=json.loads((source/'inputs/index.json').read_text());packets=[]
    runtime=json.loads((run/'trt_runtime.json').read_text())
    for row in records:
        key=row['key']
        with np.load(source/'predictions'/(key+'.npz'),allow_pickle=False) as baseline, np.load(run/'predictions'/(key+'.npz'),allow_pickle=False) as candidate, np.load(source/'references'/(key+'.npz'),allow_pickle=False) as refs:
            agreement=numerical_agreement(baseline['ffs_disparity'],baseline['ffs_valid'],candidate['disparity'],candidate['valid'])
            fields={'pytorch':(baseline['ffs_disparity'],baseline['ffs_valid']),
                'tensorrt':(candidate['disparity'],candidate['valid'])}
            nominal,_=compare_reference(refs['left_uv'],refs['expected_disparity_px'],refs['candidates'],fields)
            packets.append({'key':key,'seed':row['seed'],'source_metadata':{'left':row['left_metadata'],'right':row['right_metadata']},
                'numerical_agreement':agreement,'nominal_comparison':nominal})
    compatible=bool(runtime['first_pair_repeat_identical'] and all(p['numerical_agreement']['compatible'] for p in packets)
        and all(p['all_raw_values_finite'] for p in runtime['measurements']))
    report={'candidate_numerically_compatible_with_uncalibrated_pytorch':compatible,'packets':packets,
        'runtime':runtime,'historical_only':True,'source_run':str(source.relative_to(root)),
        'comparison_uses_same_pixels':True,'nominal_reference_not_ground_truth':True,
        'online_latency_verified':False,'metric_calibration_verified':False,
        'current_control_eligible':False,'acceptance_eligible':False,
        'interpretation':'Fixed numerical gates compare two uncalibrated candidates, not geometry accuracy or safety. Residuals use nominal sparse LiDAR references and retain large differences. Offline timings exclude ROS, simulator contention and planning.'}
    (run/'trt_comparison.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return 0 if compatible else 2


if __name__=='__main__':raise SystemExit(main())
