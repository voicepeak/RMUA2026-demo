"""Image-only headless candidate worker; no reference data or ROS imports."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from evaluation.seed_runner.manifest import file_hash
from interfaces.contracts import Metadata
from perception.stereo_depth.learned import FastFoundationStereo, filter_learned_disparity
from perception.stereo_depth.pairing import ImageFrame, StereoPair
from perception.stereo_depth.sgbm import SgbmConfig, compute_diagnostic_disparity


def main():
    import cv2
    import torch

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--vendor',type=Path,required=True)
    parser.add_argument('--checkpoint',type=Path,required=True)
    args = parser.parse_args(); run = args.run_dir
    config = json.loads((run/'profile.json').read_text())
    records = json.loads((run/'inputs'/'index.json').read_text())
    cv2.setNumThreads(1); torch.set_num_threads(1)
    begin = time.monotonic()
    model = FastFoundationStereo(args.vendor,args.checkpoint,config['checkpoint']['sha256'],
        scale=config['profile']['scale'],iterations=config['profile']['iterations'],
        max_disp=config['profile']['max_disparity_model_pixels'])
    load_s = time.monotonic()-begin
    (run/'predictions').mkdir()
    measurements = []
    with np.load(run/'inputs'/(records[0]['key']+'.npz'),allow_pickle=False) as first:
        _,_,warmup_s = model.predict(first['left'],first['right'])
    torch.cuda.reset_peak_memory_stats()
    baseline = SgbmConfig()
    for record in records:
        key=record['key']; image_path=run/'inputs'/(key+'.npz')
        if file_hash(image_path)!=record['image_sha256']:
            raise ValueError('image input changed')
        with np.load(image_path,allow_pickle=False) as images:
            left,right=images['left'],images['right']
        begin=time.monotonic()
        forward,reverse,inference_s=model.predict(left,right)
        masked,valid,lr_valid=filter_learned_disparity(forward,reverse,
            cv2.cvtColor(left,cv2.COLOR_RGB2GRAY),cv2.cvtColor(right,cv2.COLOR_RGB2GRAY),
            config['profile']['consistency_full_resolution_px'],config['profile']['common_texture_std'])
        ffs_s=time.monotonic()-begin
        def frame(side, pixels):
            meta=dict(record[side+'_metadata']);meta['covariance_or_bounds']=tuple(meta['covariance_or_bounds'])
            return ImageFrame(Metadata(**meta),'rgb8',pixels)
        a,b=frame('left',left),frame('right',right)
        pair=StereoPair(Metadata(max(a.metadata.stamp,b.metadata.stamp),min(a.metadata.valid_until,b.metadata.valid_until),
            record['epoch'],'retained_learned_comparison','front_stereo'),a,b,abs(a.metadata.stamp-b.metadata.stamp))
        begin=time.monotonic(); sgbm=compute_diagnostic_disparity(pair,baseline)
        sgbm_s=time.monotonic()-begin
        if key == records[0]['key']:
            first_hash=hashlib.sha256(forward.tobytes()+reverse.tobytes()).hexdigest()
        np.savez_compressed(run/'predictions'/(key+'.npz'),ffs_forward=forward,
            ffs_reverse_positive=reverse,ffs_disparity=masked,ffs_valid=valid,ffs_lr_valid=lr_valid,
            sgbm_disparity=sgbm.disparity_px,sgbm_valid=sgbm.valid)
        measurements.append({'key':key,'ffs_bidirectional_resize_transfer_s':inference_s,
            'ffs_with_gray_and_mask_s':ffs_s,'sgbm_gray_bidirectional_mask_s':sgbm_s})
        print(json.dumps(measurements[-1]),flush=True)
    with np.load(run/'inputs'/(records[0]['key']+'.npz'),allow_pickle=False) as first:
        repeat_forward,repeat_reverse,repeat_s=model.predict(first['left'],first['right'])
    repeat_hash=hashlib.sha256(repeat_forward.tobytes()+repeat_reverse.tobytes()).hexdigest()
    report={'model_load_s':load_s,'warmup_bidirectional_s':warmup_s,'warmup_runs':1,
        'first_pair_repeat_identical':first_hash==repeat_hash,
        'first_pair_raw_sha256':[first_hash,repeat_hash],'repeat_inference_s':repeat_s,
        'compilation_cache_may_be_warm':True,'cold_process_startup_measured':False,
        'normalization_compatibility_default_applied':model.normalization_compatibility_default_applied,
        'volume_normalize':bool(model.model.args.normalize),
        'measurements':measurements,'gpu':torch.cuda.get_device_name(0),'torch':torch.__version__,
        'cuda_runtime':torch.version.cuda,'numpy':np.__version__,'opencv':cv2.__version__,
        'opencv_threads':cv2.getNumThreads(),'torch_cpu_threads':torch.get_num_threads(),
        'peak_cuda_allocated_bytes':torch.cuda.max_memory_allocated(),
        'peak_cuda_reserved_bytes':torch.cuda.max_memory_reserved(),
        'sgbm_config':asdict(baseline),'profile':config['profile'],
        'timing_excludes':'image loading, archive I/O, ROS transport, simulator contention, planning',
        'same_search_range':False,'sgbm_search_max_px':128,'ffs_search_max_original_px':384,
        'online_latency_verified':False}
    (run/'predictions'/'runtime.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    if first_hash!=repeat_hash:
        raise RuntimeError('first-pair GPU repeat differs; retain diagnostic outputs for investigation')


if __name__=='__main__':
    main()
