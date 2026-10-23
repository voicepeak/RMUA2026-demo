"""Own an archived synthetic geometry-sampler diagnostic; no simulator scene."""
import json
import os
from pathlib import Path
import subprocess

from evaluation.seed_runner.controller_cleanup import stop_bound_controller
from evaluation.seed_runner.manifest import prepare_run,source_hashes,file_hash


def main():
    root=Path(__file__).resolve().parents[2];rt='/workspace/main/runs/ffs_runtime'
    container=os.environ.get('RMUA_CONTAINER','rmua_noetic')
    image=subprocess.check_output(['docker','inspect',container,'--format','{{.Image}}'],text=True).strip()
    run,manifest=prepare_run(root,root/'runs',None,image,{'sampler_probe':{'random_seed':4097,'global_tf32_override':'0',
        'geometry_shapes':[[n,c,1,w] for n in (1,11520) for c,w in ((28,48),(28,24),(1,120),(1,60))]}})
    inside='/workspace/main/'+str(run.relative_to(root));module='evaluation.metrics.stereo_sampler_probe'
    native=sorted((root/'runs/ffs_runtime/venv_ort/lib/python3.12/site-packages/nvidia').glob('*/lib'))
    libraries=':'.join(['/opt/rmua-driver/usr/lib/x86_64-linux-gnu']+['/workspace/main/'+str(p.relative_to(root)) for p in native])
    manifest.update(stage='P2_ISOLATED_GRID_SAMPLER_DIAGNOSIS',run_state='PREPARED',acceptance_eligible=False,control_published=False,
        runtime_lock_sha256=file_hash(root/'configs/fast_foundation_stereo_ort_runtime.lock.txt'),native_library_path=libraries)
    def save():(run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    save();print(run,flush=True);code=1
    try:
        with (run/'worker.log').open('x') as log:
            result=subprocess.run(['docker','exec','-u','1000:1000','-w','/workspace/main','-e','PYTHONPATH=/workspace/main',
                '-e','PYTHONNOUSERSITE=1','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1',
                '-e','NVIDIA_TF32_OVERRIDE=0','-e','LD_LIBRARY_PATH='+libraries,container,rt+'/venv_ort/bin/python',
                '-m',module,'--run-dir',inside],stdout=log,stderr=subprocess.STDOUT,timeout=300)
        code=result.returncode;manifest['run_state']='COMPUTED' if code==0 else 'FAILED'
    finally:
        manifest['terminated_owned_worker_pids']=stop_bound_controller(container,module,inside)
        manifest['source_sha256_after']=source_hashes(root);manifest['source_unchanged']=manifest['source_sha256_before']==manifest['source_sha256_after']
        manifest['result_exit_code']=code;save()
    return code if manifest['source_unchanged'] else 2


if __name__=='__main__':raise SystemExit(main())
