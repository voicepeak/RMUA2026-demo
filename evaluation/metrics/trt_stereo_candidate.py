"""Image-only TensorRT inference on retained inputs; no geometry references."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from evaluation.seed_runner.manifest import file_hash
from perception.stereo_depth.learned import filter_learned_disparity
from perception.stereo_depth.tensorrt_backend import TensorRTStereo


def main():
    import cv2
    import torch
    import tensorrt as trt

    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args();run=args.run_dir;root=Path(__file__).resolve().parents[2]
    manifest=json.loads((run/'manifest.json').read_text());experiment=manifest['config']['trt_experiment']
    source=root/experiment['source_run'];build=json.loads((run/('build_'+experiment['precision']+'.json')).read_text())
    cv2.setNumThreads(1);torch.set_num_threads(1);started=time.monotonic()
    model=TensorRTStereo(run/build['engine_file'],build['engine_sha256']);load_s=time.monotonic()-started
    records=json.loads((source/'inputs/index.json').read_text());output=run/'predictions';output.mkdir()
    with np.load(source/'inputs'/(records[0]['key']+'.npz'),allow_pickle=False) as pair:
        _,_,warmup=model.predict(pair['left'],pair['right'])
    torch.cuda.reset_peak_memory_stats();measurements=[]
    for row in records:
        path=source/'inputs'/(row['key']+'.npz')
        if file_hash(path)!=row['image_sha256']:raise ValueError('retained image input changed')
        with np.load(path,allow_pickle=False) as pair:left,right=pair['left'],pair['right']
        started=time.monotonic();forward,reverse,inference=model.predict(left,right)
        disparity,valid,lr=filter_learned_disparity(forward,reverse,
            cv2.cvtColor(left,cv2.COLOR_RGB2GRAY),cv2.cvtColor(right,cv2.COLOR_RGB2GRAY))
        elapsed=time.monotonic()-started
        np.savez_compressed(output/(row['key']+'.npz'),forward=forward,reverse=reverse,
            disparity=disparity,valid=valid,lr_valid=lr)
        measurements.append({'key':row['key'],'bidirectional_resize_transfer_s':inference,
            'with_gray_and_mask_s':elapsed,'valid_fraction':float(valid.mean()),
            'all_raw_values_finite':bool(np.isfinite(forward).all() and np.isfinite(reverse).all()),
            'prediction_sha256':file_hash(output/(row['key']+'.npz'))})
        print(json.dumps(measurements[-1]),flush=True)
    with np.load(source/'inputs'/(records[0]['key']+'.npz'),allow_pickle=False) as pair:
        repeated=model.predict(pair['left'],pair['right'])
    with np.load(output/(records[0]['key']+'.npz'),allow_pickle=False) as first:
        repeat_identical=first['forward'].tobytes()==repeated[0].tobytes() and first['reverse'].tobytes()==repeated[1].tobytes()
    report={'measurements':measurements,'tensorrt':trt.__version__,'torch':torch.__version__,
        'opencv':cv2.__version__,'numpy':np.__version__,'gpu':torch.cuda.get_device_name(0),
        'engine_sha256':build['engine_sha256'],'precision':experiment['precision'],
        'load_s':load_s,'warmup_s':warmup,'first_pair_repeat_identical':repeat_identical,
        'peak_cuda_allocated_bytes':torch.cuda.max_memory_allocated(),
        'peak_cuda_reserved_bytes':torch.cuda.max_memory_reserved(),
        'source_metadata_preserved_in_index':True,'current_control_eligible':False,'acceptance_eligible':False}
    (run/'trt_runtime.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
