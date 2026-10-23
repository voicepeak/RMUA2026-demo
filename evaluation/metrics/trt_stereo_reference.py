"""Separate FP32 PyTorch reference to diagnose precision versus conversion drift."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from evaluation.metrics.trt_stereo_evaluate import numerical_agreement
from evaluation.seed_runner.manifest import file_hash
from perception.stereo_depth.learned import FastFoundationStereo,filter_learned_disparity


def main():
    import cv2
    import torch

    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args();run=args.run_dir;root=Path(__file__).resolve().parents[2]
    manifest=json.loads((run/'manifest.json').read_text());cfg=manifest['config']['precision_reference']
    source=root/cfg['source_run'];profile=json.loads((root/'configs/fast_foundation_stereo.json').read_text())
    runtime=root/'runs/ffs_runtime';cv2.setNumThreads(1);torch.set_num_threads(1)
    torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    model=FastFoundationStereo(runtime/'Fast-FoundationStereo',runtime/'model'/profile['checkpoint']['filename'],
        profile['checkpoint']['sha256'],mixed_precision=False)
    records=json.loads((source/'inputs/index.json').read_text());output=run/'fp32_predictions';output.mkdir()
    packets=[]
    for row in records:
        key=row['key'];path=source/'inputs'/(key+'.npz')
        if file_hash(path)!=row['image_sha256']:raise ValueError('reference image input changed')
        with np.load(path,allow_pickle=False) as pair:
            begin=time.monotonic();forward,reverse,_=model.predict(pair['left'],pair['right'])
            disp,valid,lr=filter_learned_disparity(forward,reverse,
                cv2.cvtColor(pair['left'],cv2.COLOR_RGB2GRAY),cv2.cvtColor(pair['right'],cv2.COLOR_RGB2GRAY))
        np.savez_compressed(output/(key+'.npz'),forward=forward,reverse=reverse,disparity=disp,valid=valid)
        comparisons={}
        paths={'pytorch_amp':(source/'predictions'/(key+'.npz'),'ffs_disparity','ffs_valid','ffs_forward')}
        paths.update({name:(root/location/'predictions'/(key+'.npz'),'disparity','valid','forward') for name,location in cfg['candidate_runs'].items()})
        for name,(candidate_path,dkey,vkey,fkey) in paths.items():
            with np.load(candidate_path,allow_pickle=False) as candidate:
                result=numerical_agreement(disp,valid,candidate[dkey],candidate[vkey])
                raw_error=np.abs(forward-candidate[fkey]);finite=np.isfinite(raw_error)
                result['forward_raw_p99_px']=float(np.percentile(raw_error[finite],99)) if finite.any() else None
                result['forward_raw_max_px']=float(raw_error[finite].max()) if finite.any() else None
                comparisons[name]=result
        packets.append({'key':key,'comparisons_with_pytorch_fp32':comparisons,
            'prediction_sha256':file_hash(output/(key+'.npz')),'elapsed_with_comparisons_s':time.monotonic()-begin})
    report={'packets':packets,'tf32_enabled':False,'mixed_precision':False,'compile_disabled':True,
        'same_source_images':True,'torch':torch.__version__,
        'acceptance_eligible':False,'current_control_eligible':False,
        'interpretation':'Controlled precision/conversion diagnosis on development images, not ground-truth accuracy or production acceptance.'}
    (run/'precision_reference.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
