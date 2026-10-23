"""Optional image-only GPU process for the bounded read-only latency experiment."""
import argparse
import json
from pathlib import Path
import resource
import socket
import time

import numpy as np

from perception.stereo_depth.learned import FastFoundationStereo,filter_learned_disparity
from perception.stereo_depth.transport import decode_images,encode_disparity,digest,send_packet,receive_packet


def main():
    import cv2
    import torch

    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--vendor',type=Path,required=True)
    parser.add_argument('--checkpoint',type=Path,required=True)
    args=parser.parse_args();run=args.run_dir
    profile=json.loads((run/'profile.json').read_text());cfg=profile['profile']
    started=time.monotonic();cv2.setNumThreads(1);torch.set_num_threads(1)
    model=FastFoundationStereo(args.vendor,args.checkpoint,profile['checkpoint']['sha256'],
        cfg['scale'],cfg['iterations'],cfg['max_disparity_model_pixels'])
    loaded=time.monotonic()
    dummy=np.zeros((720,960,3),np.uint8)
    model.predict(dummy,dummy)  # Synthetic warmup only, never an online observation.
    ready=time.monotonic();torch.cuda.reset_peak_memory_stats()
    sock=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    sock.bind(str(run/'learned.sock'));sock.listen(1);sock.settimeout(1.)
    startup={'started_wall':started,'loaded_wall':loaded,'ready_wall':ready,
        'synthetic_zero_image_warmup':True,'compilation_cache_may_be_warm':True,
        'gpu':torch.cuda.get_device_name(0),'torch':torch.__version__,'cuda_runtime':torch.version.cuda,
        'numpy':np.__version__,'opencv':cv2.__version__,
        'normalization_compatibility_default_applied':model.normalization_compatibility_default_applied,
        'volume_normalize':bool(model.model.args.normalize),'profile':cfg}
    (run/'gpu_ready.json').write_text(json.dumps(startup,indent=2)+'\n')
    jobs=0;connection=None
    try:
        while time.monotonic()-ready<180:
            try:connection,_=sock.accept();break
            except socket.timeout:continue
        if connection is None:raise RuntimeError('no owned observer connected in 180s')
        connection.settimeout(45.)
        with (run/'gpu_jobs.jsonl').open('x') as output:
            while True:
                request,payload=receive_packet(connection)
                if request.get('command')=='stop' and not payload:break
                received=time.monotonic()
                left,right=decode_images(request,payload)
                forward,reverse,inference_s=model.predict(left,right)
                disparity,valid,lr=filter_learned_disparity(forward,reverse,
                    cv2.cvtColor(left,cv2.COLOR_RGB2GRAY),cv2.cvtColor(right,cv2.COLOR_RGB2GRAY),
                    cfg['consistency_full_resolution_px'],cfg['common_texture_std'])
                body=encode_disparity(disparity,valid)
                response={'request_id':request['request_id'],'source_metadata':request['source_metadata'],
                    'input_sha256':request['input_sha256'],'output_sha256':digest(body),
                    'received_wall':received,'processing_end_wall':time.monotonic(),
                    'bidirectional_resize_transfer_s':inference_s,'common_valid_fraction':float(valid.mean()),
                    'lr_valid_fraction':float(lr.mean()),'acceptance_eligible':False,'current_control_eligible':False}
                send_packet(connection,response,body)
                output.write(json.dumps(response,allow_nan=False)+'\n');output.flush();jobs+=1
    finally:
        if connection is not None:connection.close()
        sock.close();(run/'learned.sock').unlink(missing_ok=True)
        startup.update(jobs=jobs,peak_cuda_allocated_bytes=torch.cuda.max_memory_allocated(),
            peak_cuda_reserved_bytes=torch.cuda.max_memory_reserved(),max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            acceptance_eligible=False,current_control_eligible=False)
        (run/'gpu_summary.json').write_text(json.dumps(startup,indent=2)+'\n')


if __name__=='__main__':main()
