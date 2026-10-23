"""Freeze sources, image-only GPU inference, then independent sparse evaluation."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

from evaluation.seed_runner.controller_cleanup import stop_bound_controller
from evaluation.seed_runner.manifest import file_hash, prepare_run, source_hashes


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run',type=Path,action='append',required=True)
    args=parser.parse_args(); root=Path(__file__).resolve().parents[2]
    runtime=root/'runs/ffs_runtime';vendor=runtime/'Fast-FoundationStereo'
    checkpoint=runtime/'model/model_best_bp2_serialize.pth'
    profile_path=root/'configs/fast_foundation_stereo.json'
    profile=json.loads(profile_path.read_text())
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=vendor,text=True).strip()
    dirty=subprocess.check_output(['git','status','--porcelain'],cwd=vendor,text=True).strip()
    if commit!=profile['vendor']['commit'] or dirty:
        raise ValueError('vendor checkout must be clean and match pinned commit')
    if (file_hash(checkpoint)!=profile['checkpoint']['sha256']
            or checkpoint.stat().st_size!=profile['checkpoint']['bytes']):
        raise ValueError('checkpoint differs from pinned publisher digest')
    inputs={};sources=[]
    for item in args.source_run:
        source=item.resolve()
        if root/'runs' not in source.parents or source.is_symlink():
            raise ValueError('source must be an owned retained run')
        sources.append(str(source.relative_to(root)))
        names=['manifest.json','joint_summary.json','joint_audit.json']
        names += [str(p.relative_to(source)) for p in sorted((source/'joint').rglob('*')) if p.is_file()]
        inputs[str(source.relative_to(root))]={name:file_hash(source/name) for name in names}
    container=os.environ.get('RMUA_CONTAINER','rmua_noetic')
    digest=subprocess.check_output(['docker','inspect',container,'--format','{{.Image}}'],text=True).strip()
    config={'learned_stereo':{'source_runs':sources,'input_sha256':inputs,
        'profile_sha256':file_hash(profile_path),'historical_only':True}}
    run,manifest=prepare_run(root,root/'runs',None,digest,config)
    shutil.copyfile(profile_path,run/'profile.json')
    subprocess.run(['git','archive','--format=tar.gz','--output='+str(run/'vendor_source.tar.gz'),commit],cwd=vendor,check=True)
    manifest.update(stage='P2_OFFLINE_LEARNED_STEREO_COMPARISON',acceptance_eligible=False,
        control_published=False,run_state='PREPARED',model_hashes={checkpoint.name:file_hash(checkpoint)},
        vendor={'commit':commit,'dirty':dirty,'source_archive_sha256':file_hash(run/'vendor_source.tar.gz'),
                'license_sha256':file_hash(vendor/'LICENSE.txt')})
    inside='/workspace/main/'+str(run.relative_to(root)); rt='/workspace/main/runs/ffs_runtime'
    driver_dir=os.environ.get('RMUA_TRITON_LIBCUDA_PATH','/opt/rmua-driver/usr/lib/x86_64-linux-gnu')
    modules=('evaluation.metrics.learned_stereo','evaluation.metrics.learned_stereo_worker')
    python=rt+'/venv/bin/python'
    env=['-e','PYTHONNOUSERSITE=1','-e','PYTHONPATH=/workspace/main:'+rt+'/Fast-FoundationStereo',
         '-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','UV_CACHE_DIR='+rt+'/uv_cache',
         '-e','TORCH_HOME='+rt+'/cache/torch','-e','CUDA_CACHE_PATH='+rt+'/cache/cuda',
         '-e','TRITON_LIBCUDA_PATH='+driver_dir,
         '-e','TRITON_CACHE_DIR='+rt+'/cache/triton_driver_bound','-e','TORCHINDUCTOR_CACHE_DIR='+rt+'/cache/inductor']
    docker=['docker','exec','-u','1000:1000','-w','/workspace/main']+env+[container]
    def save():
        (run/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    save();print(run,flush=True);code=1
    try:
        probe='import hashlib,json,pathlib,sys; p=pathlib.Path(sys.argv[1])/"libcuda.so.1"; print(json.dumps({"python":sys.version,"libcuda_path":str(p.resolve()),"libcuda_sha256":hashlib.sha256(p.read_bytes()).hexdigest()}))'
        manifest['runtime_provenance']={
            'portable_python_sha256':file_hash(runtime/'python/cpython-3.12.11-linux-x86_64-gnu/bin/python3.12'),
            'uv_sha256':file_hash(runtime/'uv'),
            'uv_source':json.loads((runtime/'uv_source.json').read_text()),
            'checkpoint_source':json.loads((runtime/'checkpoint_source.json').read_text()),
            'packages':subprocess.check_output(docker+[rt+'/uv','pip','freeze','--python',python],text=True),
            'driver':json.loads(subprocess.check_output(docker+[python,'-c',probe,driver_dir],text=True)),
            'nvidia_smi':subprocess.check_output(docker+['nvidia-smi'],text=True)}
        save()
        commands=[('prepare',[str(root/'tools/enter_env.sh'),'env','OPENBLAS_NUM_THREADS=1','python3','-m',modules[0],
            '--run-dir',inside,'--stage','prepare'],120),
            ('candidate',docker+[python,'-m',modules[1],'--run-dir',inside,'--vendor',rt+'/Fast-FoundationStereo',
                '--checkpoint',rt+'/model/'+checkpoint.name],600),
            ('evaluate',[str(root/'tools/enter_env.sh'),'env','OPENBLAS_NUM_THREADS=1','python3','-m',modules[0],
                '--run-dir',inside,'--stage','evaluate'],120)]
        for stage,command,timeout in commands:
            with (run/(stage+'.log')).open('x') as output:
                subprocess.run(command,stdout=output,stderr=subprocess.STDOUT,check=True,timeout=timeout)
        code=0;manifest['run_state']='COMPUTED'
    except Exception as error:
        manifest.update(run_state='FAILED',error=str(error))
    finally:
        manifest['terminated_owned_worker_pids']={module:stop_bound_controller(container,module,inside) for module in modules}
        manifest['source_sha256_after']=source_hashes(root)
        manifest['source_unchanged']=manifest['source_sha256_before']==manifest['source_sha256_after']
        manifest['input_unchanged']=all(hashes=={name:file_hash(root/source/name) for name in hashes} for source,hashes in inputs.items())
        manifest['vendor_unchanged']=(not subprocess.check_output(['git','status','--porcelain'],cwd=vendor,text=True).strip()
            and subprocess.check_output(['git','rev-parse','HEAD'],cwd=vendor,text=True).strip()==commit)
        manifest['checkpoint_unchanged']=file_hash(checkpoint)==profile['checkpoint']['sha256']
        manifest['worker_exit_code']=code
        manifest['telemetry']=['inputs/index.json','predictions/runtime.json','learned_stereo_report.json']
        save()
    return code if all(manifest[name] for name in ('source_unchanged','input_unchanged','vendor_unchanged','checkpoint_unchanged')) else 2


if __name__=='__main__':
    raise SystemExit(main())
