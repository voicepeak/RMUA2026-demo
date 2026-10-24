"""Archive, export, build and independently compare the optional TensorRT candidate."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

from evaluation.seed_runner.controller_cleanup import stop_bound_controller
from evaluation.seed_runner.manifest import file_hash,prepare_run,source_hashes
from evaluation.metrics.trt_precision_policy import POLICIES


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source-run',type=Path,required=True)
    parser.add_argument('--precision',choices=('fp16','fp32'),default='fp16')
    parser.add_argument('--precision-policy',choices=POLICIES,default='default')
    parser.add_argument('--reuse-onnx-run',type=Path)
    parser.add_argument('--disable-tf32-globally',action='store_true')
    parser.add_argument('--freeze-fourier-inputs',action='store_true');parser.add_argument('--fp32-reference-run',type=Path)
    args=parser.parse_args()
    if args.precision_policy!='default' and args.precision!='fp16':parser.error('mixed precision policy requires fp16')
    root=Path(__file__).resolve().parents[2];source=args.source_run.resolve()
    if root/'runs' not in source.parents:parser.error('owned retained input run required')
    rt='/workspace/main/runs/ffs_runtime';runtime=root/'runs/ffs_runtime';vendor=runtime/'Fast-FoundationStereo'
    checkpoint=runtime/'model/model_best_bp2_serialize.pth';profile_path=root/'configs/fast_foundation_stereo.json'
    profile=json.loads(profile_path.read_text());commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=vendor,text=True).strip()
    if (commit!=profile['vendor']['commit'] or subprocess.check_output(['git','status','--porcelain'],cwd=vendor,text=True).strip()
            or file_hash(checkpoint)!=profile['checkpoint']['sha256']):raise ValueError('pinned model or vendor differs')
    names=['inputs/index.json','manifest.json','learned_stereo_report.json']
    names += [str(p.relative_to(source)) for folder in ('inputs','predictions','references') for p in sorted((source/folder).glob('*.npz'))]
    hashes={name:file_hash(source/name) for name in names};container=os.environ.get('RMUA_CONTAINER','rmua_noetic')
    reference=args.fp32_reference_run.resolve() if args.fp32_reference_run else None
    reference_hashes={}
    if args.freeze_fourier_inputs and not args.reuse_onnx_run:
        if reference is None or root/'runs' not in reference.parents:parser.error('owned FP32 reference required for Fourier parity')
        original=json.loads((reference/'manifest.json').read_text())['config']['precision_reference']
        if original['source_run']!=str(source.relative_to(root)):raise ValueError('FP32 reference uses different stereo inputs')
        reference_hashes={str(p.relative_to(reference)):file_hash(p) for p in reference.rglob('*') if p.is_file() and p.suffix=='.npz'}
    image=subprocess.check_output(['docker','inspect',container,'--format','{{.Image}}'],text=True).strip()
    config={'trt_experiment':{'source_run':str(source.relative_to(root)),'input_sha256':hashes,
        'precision':args.precision,'precision_policy':args.precision_policy,
        'global_tf32_override':'0' if args.disable_tf32_globally else None,
        'fourier_inputs_frozen':args.freeze_fourier_inputs,
        'fp32_reference':{'run':str(reference.relative_to(root)),'input_sha256':reference_hashes,
            'manifest_sha256':file_hash(reference/'manifest.json')} if reference else None,
        'profile_sha256':file_hash(profile_path),'historical_only':True}}
    run,manifest=prepare_run(root,root/'runs',None,image,config);inside='/workspace/main/'+str(run.relative_to(root))
    shutil.copyfile(profile_path,run/'profile.json')
    subprocess.run(['git','archive','--format=tar.gz','--output='+str(run/'vendor_source.tar.gz'),commit],cwd=vendor,check=True)
    manifest.update(stage='P2_OFFLINE_TENSORRT_COMPARISON',run_state='PREPARED',acceptance_eligible=False,
        control_published=False,vendor={'commit':commit,'source_archive_sha256':file_hash(run/'vendor_source.tar.gz'),
            'license_sha256':file_hash(vendor/'LICENSE.txt')},model_hashes={checkpoint.name:file_hash(checkpoint)})
    driver=os.environ.get('RMUA_TRITON_LIBCUDA_PATH','/opt/rmua-driver/usr/lib/x86_64-linux-gnu')
    docker=['docker','exec','-u','1000:1000','-w','/workspace/main','-e','PYTHONNOUSERSITE=1',
        '-e','PYTHONPATH=/workspace/main:'+rt+'/Fast-FoundationStereo','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1',
        '-e','UV_CACHE_DIR='+rt+'/uv_cache','-e','TRITON_LIBCUDA_PATH='+driver,
        '-e','TRITON_CACHE_DIR='+rt+'/cache/triton_driver_bound','-e','TORCH_HOME='+rt+'/cache/torch',
        '-e','TORCHINDUCTOR_CACHE_DIR='+rt+'/cache/inductor',container]
    python=rt+'/venv_trt/bin/python';module='evaluation.metrics.trt_stereo_build'
    if args.disable_tf32_globally:docker[-1:-1]=['-e','NVIDIA_TF32_OVERRIDE=0']
    modules=(module,'evaluation.metrics.trt_stereo_candidate','evaluation.metrics.trt_stereo_evaluate')
    def save():(run/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    save();print(run,flush=True);code=1
    try:
        manifest['runtime_provenance']={'packages':subprocess.check_output(docker+[rt+'/uv','pip','freeze','--python',python],text=True),
            'package_sources':json.loads((runtime/'trt_package_sources.json').read_text()),
            'nvidia_smi':subprocess.check_output(docker+['nvidia-smi'],text=True),
            'runtime_lock_sha256':file_hash(root/'configs/fast_foundation_stereo_trt_runtime.lock.txt')}
        save();commands=[]
        if args.reuse_onnx_run:
            reuse=args.reuse_onnx_run.resolve()
            if root/'runs' not in reuse.parents:raise ValueError('ONNX source must be an owned historical run')
            exported=json.loads((reuse/'export.json').read_text());parity=json.loads((reuse/'export_parity.json').read_text())
            if not parity['passed'] or exported['source_weights_sha256']!=profile['checkpoint']['sha256'] or exported['onnx_sha256']!=file_hash(reuse/'stereo.onnx'):
                raise ValueError('reused ONNX provenance differs')
            if exported.get('fourier_inputs_frozen',False)!=args.freeze_fourier_inputs:raise ValueError('reused graph has a different Fourier policy')
            for name in ('stereo.onnx','export.json','export_parity.json'):shutil.copyfile(reuse/name,run/name)
            if args.freeze_fourier_inputs:
                frozen_parity=json.loads((reuse/'fourier_parity.json').read_text())
                if not frozen_parity['passed']:raise ValueError('reused Fourier input freezing did not preserve FP32 reference')
                shutil.copyfile(reuse/'fourier_parity.json',run/'fourier_parity.json')
            manifest['onnx_reused_from']=str(reuse.relative_to(root));save()
        else:
            commands.append(('export',docker+['env','TORCH_COMPILE_DISABLE=1','TORCHDYNAMO_DISABLE=1',python,'-m',module,
                '--run-dir',inside,'--stage','export','--vendor',rt+'/Fast-FoundationStereo',
                '--checkpoint',rt+'/model/'+checkpoint.name,'--sample','/workspace/main/'+str(source.relative_to(root))+'/inputs/0000.npz'],300))
            if args.freeze_fourier_inputs:
                commands[-1][1].extend(['--freeze-fourier-inputs','--fp32-reference-run','/workspace/main/'+str(reference.relative_to(root))])
        commands += [('build',docker+[python,'-m',module,'--run-dir',inside,'--stage','build','--precision',args.precision,
                '--precision-policy',args.precision_policy],1800),
            ('candidate',docker+[python,'-m',modules[1],'--run-dir',inside],300),
            ('evaluate',docker+[python,'-m',modules[2],'--run-dir',inside],120)]
        for stage,command,timeout in commands:
            with (run/(stage+'.log')).open('x') as log:
                completed=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=timeout)
            if completed.returncode:
                manifest['failed_stage']=stage;code=completed.returncode
                raise RuntimeError(stage+' returned '+str(code))
        code=0;manifest['run_state']='COMPUTED'
    except Exception as error:manifest.update(run_state='FAILED',error=str(error))
    finally:
        manifest['terminated_owned_workers']={name:stop_bound_controller(container,name,inside) for name in modules}
        manifest['source_sha256_after']=source_hashes(root);manifest['source_unchanged']=manifest['source_sha256_before']==manifest['source_sha256_after']
        manifest['input_unchanged']=hashes=={name:file_hash(source/name) for name in hashes}
        manifest['fp32_reference_unchanged']=(reference_hashes=={name:file_hash(reference/name) for name in reference_hashes}
            and (reference is None or file_hash(reference/'manifest.json')==config['trt_experiment']['fp32_reference']['manifest_sha256']))
        manifest['vendor_unchanged']=(not subprocess.check_output(['git','status','--porcelain'],cwd=vendor,text=True).strip()
            and subprocess.check_output(['git','rev-parse','HEAD'],cwd=vendor,text=True).strip()==commit)
        manifest['checkpoint_unchanged']=file_hash(checkpoint)==profile['checkpoint']['sha256']
        manifest['result_exit_code']=code;save()
    return code if all(manifest[name] for name in ('source_unchanged','input_unchanged','vendor_unchanged','checkpoint_unchanged','fp32_reference_unchanged')) else 2


if __name__=='__main__':raise SystemExit(main())
