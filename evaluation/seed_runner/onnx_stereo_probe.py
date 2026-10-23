"""Freeze sources and known artifacts before an independent ONNX Runtime probe."""
import argparse
import json
import os
from pathlib import Path
import subprocess

from evaluation.seed_runner.controller_cleanup import stop_bound_controller
from evaluation.seed_runner.manifest import file_hash,prepare_run,source_hashes


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source-run','fp32-reference-run','fp16-run','fp32-run'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--keys',nargs='+',default=['0000'])
    parser.add_argument('--layers',nargs='+')
    parser.add_argument('--reuse-subgraph-run',type=Path);args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    locations={key:getattr(args,key).resolve() for key in ('source_run','fp32_reference_run','fp16_run','fp32_run')}
    if any(root/'runs' not in p.parents for p in locations.values()):parser.error('owned run artifacts required')
    inputs={str(p.relative_to(root)):file_hash(p) for location in locations.values() for p in location.rglob('*')
        if p.is_file() and (p.suffix in ('.npz','.onnx') or p.name in ('manifest.json','index.json'))}
    reuse=args.reuse_subgraph_run.resolve() if args.reuse_subgraph_run else None
    if reuse:
        if root/'runs' not in reuse.parents or not args.layers:parser.error('owned subgraph run and layer selection required')
        original=json.loads((reuse/'manifest.json').read_text())['config']['onnx_probe']
        if original['layers']!=args.layers or original['onnx_sha256']!=file_hash(locations['fp16_run']/'stereo.onnx'):
            raise ValueError('reused subgraph has different source or selected tensors')
        for name in ('subgraph.onnx','subgraph.engine','manifest.json'):inputs[str((reuse/name).relative_to(root))]=file_hash(reuse/name)
    cfg={'source_run':str(locations['source_run'].relative_to(root)),
        'fp32_reference_run':str(locations['fp32_reference_run'].relative_to(root)),
        'onnx_run':str(locations['fp16_run'].relative_to(root)),'onnx_sha256':file_hash(locations['fp16_run']/'stereo.onnx'),
        'trt_runs':{key:str(locations[key].relative_to(root)) for key in ('fp16_run','fp32_run')},
        'input_sha256':inputs,'keys':args.keys,'layers':args.layers,
        'reuse_subgraph_run':str(reuse.relative_to(root)) if reuse else None}
    container=os.environ.get('RMUA_CONTAINER','rmua_noetic');rt='/workspace/main/runs/ffs_runtime'
    native_dirs=sorted((root/'runs/ffs_runtime/venv_ort/lib/python3.12/site-packages/nvidia').glob('*/lib'))
    library_path=':'.join(['/opt/rmua-driver/usr/lib/x86_64-linux-gnu']+['/workspace/main/'+str(p.relative_to(root)) for p in native_dirs])
    image=subprocess.check_output(['docker','inspect',container,'--format','{{.Image}}'],text=True).strip()
    run,manifest=prepare_run(root,root/'runs',None,image,{'onnx_probe':cfg});inside='/workspace/main/'+str(run.relative_to(root))
    manifest.update(stage='P2_ONNX_CONVERSION_DIAGNOSIS',run_state='PREPARED',acceptance_eligible=False,control_published=False)
    module='evaluation.metrics.onnx_layer_probe' if args.layers else 'evaluation.metrics.onnx_stereo_probe';code=1
    def save():(run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    save();print(run,flush=True)
    try:
        manifest['runtime_provenance']={'packages':subprocess.check_output(['docker','exec','-u','1000:1000',
            '-e','UV_CACHE_DIR='+rt+'/uv_cache',container,rt+'/uv','pip','freeze','--python',rt+'/venv_ort/bin/python'],text=True),
            'runtime_lock_sha256':file_hash(root/'configs/fast_foundation_stereo_ort_runtime.lock.txt'),
            'native_library_path':library_path}
        save()
        with (run/'worker.log').open('x') as log:
            completed=subprocess.run(['docker','exec','-u','1000:1000','-w','/workspace/main','-e','PYTHONNOUSERSITE=1',
                '-e','PYTHONPATH=/workspace/main','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1',
                '-e','LD_LIBRARY_PATH='+library_path,'-e','NVIDIA_TF32_OVERRIDE=0',container,
                rt+'/venv_ort/bin/python','-m',module,'--run-dir',inside],stdout=log,stderr=subprocess.STDOUT,timeout=600)
        code=completed.returncode;manifest['run_state']='COMPUTED' if code==0 else 'FAILED'
    finally:
        manifest['terminated_owned_worker_pids']=stop_bound_controller(container,module,inside)
        manifest['source_sha256_after']=source_hashes(root);manifest['source_unchanged']=manifest['source_sha256_before']==manifest['source_sha256_after']
        manifest['input_unchanged']=inputs=={name:file_hash(root/name) for name in inputs};manifest['result_exit_code']=code;save()
    return code if manifest['source_unchanged'] and manifest['input_unchanged'] else 2


if __name__=='__main__':raise SystemExit(main())
