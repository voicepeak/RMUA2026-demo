"""Independent ONNX Runtime diagnosis on identical normalized stereo inputs."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from evaluation.metrics.trt_stereo_evaluate import numerical_agreement
from evaluation.seed_runner.manifest import file_hash
from perception.stereo_depth.learned import filter_learned_disparity,full_resolution_disparity


def normalized_input(image):
    import cv2
    import torch

    resized=cv2.resize(image,(480,360),interpolation=cv2.INTER_AREA)
    value=torch.as_tensor(resized.copy(),device='cuda').float()[None].permute(0,3,1,2).contiguous()
    value=torch.nn.functional.pad(value,(0,0,12,12),mode='replicate')
    mean=value.new_tensor([.485,.456,.406]).view(1,3,1,1)
    std=value.new_tensor([.229,.224,.225]).view(1,3,1,1)
    return ((value/255.-mean)/std).cpu().numpy()


def main():
    import cv2
    import torch  # Preloads the pinned CUDA/cuDNN libraries, as ORT documents.
    import onnxruntime as ort

    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args();run=args.run_dir;root=Path(__file__).resolve().parents[2]
    manifest=json.loads((run/'manifest.json').read_text());cfg=manifest['config']['onnx_probe']
    source=root/cfg['source_run'];onnx_run=root/cfg['onnx_run'];reference=root/cfg['fp32_reference_run']
    if file_hash(onnx_run/'stereo.onnx')!=cfg['onnx_sha256']:raise ValueError('ONNX model changed')
    cv2.setNumThreads(1);torch.set_num_threads(1)
    options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
    options.graph_optimization_level=ort.GraphOptimizationLevel.ORT_DISABLE_ALL
    options.enable_profiling=True;options.profile_file_prefix=str(run/'ort_profile')
    session=ort.InferenceSession(str(onnx_run/'stereo.onnx'),sess_options=options,
        providers=[('CUDAExecutionProvider',{'use_tf32':'0','cudnn_conv_algo_search':'HEURISTIC',
            'cudnn_conv_use_max_workspace':'0'}),'CPUExecutionProvider'])
    if session.get_providers()[0]!='CUDAExecutionProvider':raise RuntimeError('CUDA ORT provider unavailable')
    index=json.loads((source/'inputs/index.json').read_text());packets=[];output=run/'predictions';output.mkdir()
    if not cfg['keys'] or len(set(cfg['keys']))!=len(cfg['keys']) or not set(cfg['keys'])<={row['key'] for row in index}:
        raise ValueError('probe keys must select distinct retained image pairs')
    for row in index:
        key=row['key']
        if key not in cfg['keys']:continue
        path=source/'inputs'/(key+'.npz')
        if file_hash(path)!=row['image_sha256']:raise ValueError('image input changed')
        with np.load(path,allow_pickle=False) as pair:left,right=pair['left'],pair['right']
        a,b=normalized_input(left),normalized_input(right)
        started=time.monotonic()
        forward=session.run(['disparity'],{'left_image':a,'right_image':b})[0]
        reverse=session.run(['disparity'],{'left_image':np.ascontiguousarray(b[:,:,:,::-1]),
            'right_image':np.ascontiguousarray(a[:,:,:,::-1])})[0][:,:,:,::-1]
        forward=full_resolution_disparity(forward[:,:,12:-12,:].reshape(360,480),960,720,.5)
        reverse=full_resolution_disparity(reverse[:,:,12:-12,:].reshape(360,480),960,720,.5)
        disp,valid,lr=filter_learned_disparity(forward,reverse,cv2.cvtColor(left,cv2.COLOR_RGB2GRAY),cv2.cvtColor(right,cv2.COLOR_RGB2GRAY))
        np.savez_compressed(output/(key+'.npz'),forward=forward,reverse=reverse,disparity=disp,valid=valid)
        comparisons={}
        sources={'pytorch_fp32':(reference/'fp32_predictions'/(key+'.npz'),'disparity','valid','forward'),
            'pytorch_amp':(source/'predictions'/(key+'.npz'),'ffs_disparity','ffs_valid','ffs_forward')}
        for label,location in cfg['trt_runs'].items():sources[label]=(root/location/'predictions'/(key+'.npz'),'disparity','valid','forward')
        for label,(location,dkey,vkey,fkey) in sources.items():
            with np.load(location,allow_pickle=False) as saved:
                agreement=numerical_agreement(saved[dkey],saved[vkey],disp,valid)
                errors=np.abs(forward-saved[fkey]);finite=np.isfinite(errors)
                agreement['forward_raw_p99_px']=float(np.percentile(errors[finite],99)) if finite.any() else None
                agreement['forward_raw_max_px']=float(errors[finite].max()) if finite.any() else None
                comparisons[label]=agreement
        packets.append({'key':key,'image_sha256':row['image_sha256'],'normalized_input_sha256':hashlib.sha256(a.tobytes()+b.tobytes()).hexdigest(),
            'comparisons':comparisons,'elapsed_including_comparisons_s':time.monotonic()-started,
            'output_sha256':file_hash(output/(key+'.npz')),'source_metadata':{'left':row['left_metadata'],'right':row['right_metadata']}})
        print(json.dumps(packets[-1]),flush=True)
    profile_path=Path(session.end_profiling());profile=json.loads(profile_path.read_text())
    providers=Counter(event.get('args',{}).get('provider') for event in profile if event.get('args',{}).get('provider'))
    report={'ort':ort.__version__,'torch':torch.__version__,'providers':session.get_providers(),
        'provider_options':session.get_provider_options(),'profile_event_counts_by_provider':dict(providers),
        'profile_file':profile_path.name,'onnx_sha256':cfg['onnx_sha256'],'graph_optimization':'disabled',
        'packets':packets,'historical_only':True,'metric_calibration_verified':False,
        'current_control_eligible':False,'acceptance_eligible':False}
    (run/'onnx_probe.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
