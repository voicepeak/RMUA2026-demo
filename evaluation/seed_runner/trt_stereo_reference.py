"""Archive the controlled FP32 comparison against retained AMP and TRT outputs."""
import argparse
import json
import os
from pathlib import Path
import subprocess

from evaluation.seed_runner.controller_cleanup import stop_bound_controller
from evaluation.seed_runner.manifest import prepare_run,source_hashes,file_hash


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run',type=Path,required=True);parser.add_argument('--fp16-run',type=Path,required=True)
    parser.add_argument('--fp32-run',type=Path,required=True);args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    locations={'source_run':args.source_run.resolve(),'trt_fp16':args.fp16_run.resolve(),'trt_fp32':args.fp32_run.resolve()}
    if any(root/'runs' not in p.parents for p in locations.values()):parser.error('owned artifact runs required')
    inputs={str(p.relative_to(root)):file_hash(p) for location in locations.values() for p in location.rglob('*')
        if p.is_file() and (p.suffix=='.npz' or p.name in ('manifest.json','trt_comparison.json','index.json'))}
    container=os.environ.get('RMUA_CONTAINER','rmua_noetic');rt='/workspace/main/runs/ffs_runtime'
    image=subprocess.check_output(['docker','inspect',container,'--format','{{.Image}}'],text=True).strip()
    cfg={'precision_reference':{'source_run':str(locations['source_run'].relative_to(root)),
        'candidate_runs':{key:str(locations[key].relative_to(root)) for key in ('trt_fp16','trt_fp32')},'input_sha256':inputs}}
    run,manifest=prepare_run(root,root/'runs',None,image,cfg);inside='/workspace/main/'+str(run.relative_to(root))
    module='evaluation.metrics.trt_stereo_reference';manifest.update(stage='P2_CONTROLLED_STEREO_PRECISION_REFERENCE',acceptance_eligible=False,control_published=False)
    def save():(run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    save();print(run,flush=True);code=1
    try:
        with (run/'worker.log').open('x') as log:
            completed=subprocess.run(['docker','exec','-u','1000:1000','-w','/workspace/main',
                '-e','PYTHONPATH=/workspace/main:'+rt+'/Fast-FoundationStereo','-e','PYTHONNOUSERSITE=1',
                '-e','TORCH_COMPILE_DISABLE=1','-e','TORCHDYNAMO_DISABLE=1','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1',
                '-e','TRITON_LIBCUDA_PATH=/opt/rmua-driver/usr/lib/x86_64-linux-gnu',
                '-e','TRITON_CACHE_DIR='+rt+'/cache/triton_driver_bound','-e','TORCHINDUCTOR_CACHE_DIR='+rt+'/cache/inductor',
                container,rt+'/venv_trt/bin/python',
                '-m',module,'--run-dir',inside],stdout=log,stderr=subprocess.STDOUT,timeout=180)
        code=completed.returncode;manifest['run_state']='COMPUTED' if code==0 else 'FAILED'
    finally:
        manifest['terminated_owned_workers']=stop_bound_controller(container,module,inside)
        manifest['source_sha256_after']=source_hashes(root);manifest['source_unchanged']=manifest['source_sha256_before']==manifest['source_sha256_after']
        manifest['input_unchanged']=inputs=={name:file_hash(root/name) for name in inputs}
        manifest['result_exit_code']=code;save()
    return code if manifest['source_unchanged'] and manifest['input_unchanged'] else 2


if __name__=='__main__':raise SystemExit(main())
