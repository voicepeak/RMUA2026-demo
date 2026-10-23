"""Own a fresh scene and asynchronous diagnostic GPU pipeline, without control."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time

from evaluation.seed_runner.controller_cleanup import stop_bound_controller
from evaluation.seed_runner.manifest import prepare_run,source_hashes,file_hash
from evaluation.seed_runner.simulator_preflight import verify_local_simulator


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed',type=int,required=True);parser.add_argument('--duration',type=float,default=10.)
    parser.add_argument('--image-age-budget-ms',type=int,choices=(150,200),default=150)
    args=parser.parse_args()
    if args.seed<0 or not math.isfinite(args.duration) or not 0<args.duration<=30:parser.error('invalid seed or duration')
    root=Path(__file__).resolve().parents[2];rt='/workspace/main/runs/ffs_runtime';runtime=root/'runs/ffs_runtime'
    vendor=runtime/'Fast-FoundationStereo';checkpoint=runtime/'model/model_best_bp2_serialize.pth'
    profile_path=root/'configs/fast_foundation_stereo.json';profile=json.loads(profile_path.read_text())
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=vendor,text=True).strip()
    if (commit!=profile['vendor']['commit'] or subprocess.check_output(['git','status','--porcelain'],cwd=vendor,text=True).strip()
            or file_hash(checkpoint)!=profile['checkpoint']['sha256']):raise ValueError('pinned candidate provenance differs')
    before=verify_local_simulator(root);container=os.environ.get('RMUA_CONTAINER','rmua_noetic')
    image=subprocess.check_output(['docker','inspect',container,'--format','{{.Image}}'],text=True).strip()
    config={'experiment':{'kind':'learned_stereo_read_only','duration_s':args.duration,
        'image_age_budget_ms':args.image_age_budget_ms,'profile_sha256':file_hash(profile_path),
        'latest_pending_capacity':1,'in_flight_limit':1,'warmup_before_scene':True}}
    run,manifest=prepare_run(root,root/'runs',args.seed,image,config);inside='/workspace/main/'+str(run.relative_to(root))
    shutil.copyfile(profile_path,run/'profile.json')
    subprocess.run(['git','archive','--format=tar.gz','--output='+str(run/'vendor_source.tar.gz'),commit],cwd=vendor,check=True)
    manifest.update(stage='P2_READ_ONLY_LEARNED_STEREO_LATENCY',simulator_preflight=before,
        acceptance_eligible=False,control_published=False,run_state='PREPARED',
        model_hashes={checkpoint.name:file_hash(checkpoint)},
        vendor={'commit':commit,'source_archive_sha256':file_hash(run/'vendor_source.tar.gz'),
                'license_sha256':file_hash(vendor/'LICENSE.txt')})
    def save():(run/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    save();print(run,flush=True)
    driver=os.environ.get('RMUA_TRITON_LIBCUDA_PATH','/opt/rmua-driver/usr/lib/x86_64-linux-gnu')
    docker=['docker','exec','-u','1000:1000','-w','/workspace/main',
        '-e','PYTHONNOUSERSITE=1','-e','PYTHONPATH=/workspace/main:'+rt+'/Fast-FoundationStereo',
        '-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','UV_CACHE_DIR='+rt+'/uv_cache',
        '-e','TORCH_HOME='+rt+'/cache/torch','-e','CUDA_CACHE_PATH='+rt+'/cache/cuda',
        '-e','TRITON_LIBCUDA_PATH='+driver,'-e','TRITON_CACHE_DIR='+rt+'/cache/triton_driver_bound',
        '-e','TORCHINDUCTOR_CACHE_DIR='+rt+'/cache/inductor',container]
    python=rt+'/venv/bin/python';enter=str(root/'tools/enter_env.sh')
    worker_module='perception.stereo_depth.live_worker';observer_module='adapters.ros_noetic.learned_stereo_observer'
    monitor_module='evaluation.metrics.learned_gpu_monitor';marker='rmua_learned_live_readiness'
    processes={};logs=[];code=1
    try:
        probe='import hashlib,json,pathlib,sys; p=pathlib.Path(sys.argv[1])/"libcuda.so.1"; print(json.dumps({"python":sys.version,"libcuda_path":str(p.resolve()),"libcuda_sha256":hashlib.sha256(p.read_bytes()).hexdigest()}))'
        manifest['runtime_provenance']={'packages':subprocess.check_output(docker+[rt+'/uv','pip','freeze','--python',python],text=True),
            'driver':json.loads(subprocess.check_output(docker+[python,'-c',probe,driver],text=True)),
            'portable_python_sha256':file_hash(runtime/'python/cpython-3.12.11-linux-x86_64-gnu/bin/python3.12'),
            'runtime_lock_sha256':file_hash(root/'configs/fast_foundation_stereo_runtime.lock.txt')}
        def start(name,command):
            log=(run/(name+'.log')).open('x');logs.append(log)
            processes[name]=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
            return processes[name]
        worker=start('gpu_worker',docker+[python,'-m',worker_module,'--run-dir',inside,
            '--vendor',rt+'/Fast-FoundationStereo','--checkpoint',rt+'/model/'+checkpoint.name])
        deadline=time.monotonic()+60
        while not (run/'gpu_ready.json').exists():
            if worker.poll() is not None:raise RuntimeError('GPU initialization failed; see gpu_worker.log')
            if time.monotonic()>deadline:raise RuntimeError('GPU initialization exceeded 60s')
            time.sleep(.1)
        simulator=start('simulator',[str(root/'tools/run_sim.sh'),str(args.seed),'offscreen'])
        deadline=time.monotonic()+90
        while time.monotonic()<deadline:
            if simulator.poll() is not None:raise RuntimeError('fresh simulator failed')
            command=[enter,'python3','-c','import socket,rospy; socket.setdefaulttimeout(2.); '
                'from sensor_msgs.msg import Imu; rospy.init_node("learned_ready",anonymous=True); '
                'rospy.wait_for_message("/airsim_node/drone_1/imu/imu",Imu,timeout=2)',marker,'--run-dir',inside]
            try:ready=subprocess.run(command,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=8)
            except subprocess.TimeoutExpired:
                stop_bound_controller(container,marker,inside);continue
            if ready.returncode==0:break
        else:raise RuntimeError('simulator formal-sensor readiness timeout')
        manifest['gpu_processes_during_scene']=subprocess.check_output(docker+['nvidia-smi'],text=True)
        monitor=start('usage_monitor',docker+[python,'-m',monitor_module,'--run-dir',inside])
        manifest['run_state']='OBSERVING';save()
        with (run/'observer.log').open('x') as log:
            observer=subprocess.run([enter,'env','OPENBLAS_NUM_THREADS=1','python3','-m',observer_module,
                '--run-dir',inside,'--duration',str(args.duration),'--image-age-budget-ms',str(args.image_age_budget_ms)],
                stdout=log,stderr=subprocess.STDOUT,timeout=args.duration+30)
        manifest['observer_exit_code']=observer.returncode
        worker_code=worker.wait(timeout=15);manifest['gpu_worker_exit_code']=worker_code
        code=observer.returncode or worker_code
        manifest['run_state']='OBSERVED' if code==0 else 'OBSERVER_FAILED'
    except Exception as error:manifest.update(run_state='FAILED',error=str(error));code=1
    finally:
        cleanup=[]
        for module in (observer_module,worker_module,monitor_module,marker):
            try:manifest['terminated_'+module]=stop_bound_controller(container,module,inside)
            except Exception as error:cleanup.append(str(error))
        if 'simulator' in processes:
            try:
                if processes['simulator'].poll() is None:
                    subprocess.run([str(root/'tools/stop_sim.sh')],stdout=subprocess.DEVNULL,check=True,timeout=25)
                processes['simulator'].wait(timeout=15)
            except Exception as error:cleanup.append('simulator:'+str(error))
        for name,process in processes.items():
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:cleanup.append(name+' did not stop')
        for log in logs:log.close()
        manifest['cleanup_errors']=cleanup
        if cleanup:code=1;manifest['run_state']='CLEANUP_FAILED'
        if (run/'online_summary.json').exists():
            with (run/'audit.log').open('x') as log:
                audit=subprocess.run([enter,'env','OPENBLAS_NUM_THREADS=1','python3','-m','evaluation.metrics.learned_live',
                    '--run-dir',inside],stdout=log,stderr=subprocess.STDOUT,timeout=30)
            manifest['audit_exit_code']=audit.returncode;code=code or audit.returncode
            if audit.returncode and manifest['run_state']=='OBSERVED':manifest['run_state']='AUDIT_FAILED'
        manifest['source_sha256_after']=source_hashes(root)
        manifest['source_unchanged']=manifest['source_sha256_before']==manifest['source_sha256_after']
        manifest['original_simulator_files_unchanged']=before==verify_local_simulator(root)
        manifest['vendor_unchanged']=(not subprocess.check_output(['git','status','--porcelain'],cwd=vendor,text=True).strip()
            and subprocess.check_output(['git','rev-parse','HEAD'],cwd=vendor,text=True).strip()==commit)
        manifest['checkpoint_unchanged']=file_hash(checkpoint)==profile['checkpoint']['sha256']
        manifest['result_exit_code']=code;save()
    return code if all(manifest[key] for key in ('source_unchanged','original_simulator_files_unchanged','vendor_unchanged','checkpoint_unchanged')) else 2


if __name__=='__main__':raise SystemExit(main())
