#!/usr/bin/env python3
"""One command: fresh simulator, immediate control, recording and stage runner."""
import argparse
import math
import json
from pathlib import Path
import subprocess
import sys
import time
import hashlib
import shutil
from system_health import snapshot as system_health_snapshot
from vision_runtime import probe_cuda

parser=argparse.ArgumentParser()
parser.add_argument('--mode',choices=('render','offscreen','background'),default='render')
parser.add_argument('--clock-speed',type=float,default=1.)
parser.add_argument('--clock-type',choices=('auto','ScalableClock','SteppableClock'),default='auto')
parser.add_argument('--lidar-point-rate',type=int,help='Optional lidar points/second; default preserves simulator settings')
parser.add_argument('--stall-seconds',type=float,default=20.)
parser.add_argument('--obstacle-backend',choices=('lidar_nav','legacy'),default='lidar_nav')
parser.add_argument('--planner-mode',choices=('legacy','spacetime'),default='legacy')
parser.add_argument('--spacetime-bridge-hold',type=float,help='Measured external bridge command expiry in seconds; absent means no certified motion')
parser.add_argument('--outbound-response',choices=('legacy','coupled'),default='coupled',help='First leg response policy; subsequent vehicle legs retain coupled response')
parser.add_argument('--out',type=Path)
parser.add_argument('--allow-cpu-vision',action='store_true',
                    help='Explicitly allow slower CPU gate inference if CUDA is unavailable')
args=parser.parse_args()
if args.planner_mode=='spacetime' and args.obstacle_backend!='lidar_nav':parser.error('spacetime requires lidar_nav')
if args.spacetime_bridge_hold is not None:
    interval=1./(20.*args.clock_speed) if 0.<args.clock_speed<=2. else 0.
    if not math.isfinite(args.spacetime_bridge_hold) or not 0.<args.spacetime_bridge_hold<=interval:
        parser.error('Measured bridge expiry must be positive and no longer than the control period')

if not 0.<args.clock_speed<=2.:parser.error('--clock-speed must be in (0, 2]')
if args.lidar_point_rate is not None and args.lidar_point_rate<=0:parser.error('--lidar-point-rate must be positive')
workspace=Path(__file__).resolve().parents[2]
out=(args.out or workspace/'frames'/time.strftime('race_%Y%m%d_%H%M%S')).resolve()
out.relative_to(workspace)  # The container mount must be able to see recordings.
out.mkdir(parents=True,exist_ok=True)
container_out=Path('/workspace')/out.relative_to(workspace)
settings=json.loads((workspace/'simulator/simulator_12.0.0.5/settings.json').read_text())
settings['ClockSpeed']=args.clock_speed
if args.clock_type!='auto':settings['ClockType']=args.clock_type
if args.lidar_point_rate is not None:settings['Vehicles']['drone_1']['Sensors']['lidar']['PointsPerSecond']=args.lidar_point_rate
(out/'settings.json').write_text(json.dumps(settings,indent=2)+'\n')
source=workspace/'rmua_ws/src/route_follower'
snapshot=out/'controller_sources'
if snapshot.exists():raise RuntimeError('Use a new experiment directory; controller snapshot already exists')
shutil.copytree(source,snapshot,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
native=workspace/'rmua_ws/devel/lib/libvelocity_response_native.so'
if native.exists():
    (snapshot/'native').mkdir()
    shutil.copy2(native,snapshot/'native'/native.name)
hashes={str(path.relative_to(out)):hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(snapshot.rglob('*')) if path.is_file()}
tool_snapshot=out/'tool_sources';tool_snapshot.mkdir(exist_ok=False)
for name in ('start_seed123_race.py','race_start_watch.py','race_runner.py','watch_debug_race.py',
             'race_monitor_policy.py','summarize_debug_race.py','control_trace.py','system_health.py','vision_runtime.py'):
    shutil.copy2(Path(__file__).with_name(name),tool_snapshot/name)
tool_hashes={str(path.relative_to(out)):hashlib.sha256(path.read_bytes()).hexdigest()
             for path in sorted(tool_snapshot.glob('*.py'))}
sim_snapshot=out/'sim_launch_sources';sim_snapshot.mkdir(exist_ok=False)
for path in (workspace/'run_sim.sh',workspace/'simulator/simulator_12.0.0.5/run_simulator.sh',
             workspace/'simulator/simulator_12.0.0.5/run_simulator_offscreen.sh'):
    shutil.copy2(path,sim_snapshot/path.name)
sim_hashes={str(path.relative_to(out)):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(sim_snapshot.glob('*'))}
(out/'controller_versions.json').write_text(json.dumps(dict(wall_time=time.time(),seed=123,
    requested_clock_speed=args.clock_speed,mode=args.mode,backend=args.obstacle_backend,
    requested_clock_type=args.clock_type,planner_mode=args.planner_mode,spacetime_bridge_hold=args.spacetime_bridge_hold,
    outbound_response=args.outbound_response,
    lidar_points_per_second=settings['Vehicles']['drone_1']['Sensors']['lidar']['PointsPerSecond'],
    car_model_enabled=args.obstacle_backend=='legacy',snapshot_kind='runtime package captured before launch',
    sha256=hashes,tools_sha256=tool_hashes,sim_launch_sha256=sim_hashes),indent=2)+'\n')
def inspect():
    result=subprocess.run(['docker','inspect','rmua_noetic'],capture_output=True,text=True,check=True)
    return json.loads(result.stdout)[0]['State']
old=inspect()
if not old['Running']:
    subprocess.run(['docker','start','rmua_noetic'],check=True,capture_output=True)
    old=inspect()
vision_health=probe_cuda()
(out/'vision_runtime.json').write_text(json.dumps(vision_health,indent=2)+'\n')
if not vision_health['available'] and not args.allow_cpu_vision:
    raise RuntimeError('CUDA vision unavailable before simulator start; see '+str(out/'vision_runtime.json')+
                       '. Repair the GPU compute device, or explicitly use --allow-cpu-vision.')
print('Vision device:',vision_health.get('device') or 'CPU (explicit override)',flush=True)
check=subprocess.run(['docker','exec','rmua_noetic','pgrep','-f','RMUA-Linux|rosmaster'],capture_output=True)
had_sim=check.returncode==0
sim=subprocess.Popen([str(workspace/'run_sim.sh'),'123',args.mode,str(container_out/'settings.json')],cwd=workspace,
                     stdout=(out/'simulator.log').open('w'),stderr=subprocess.STDOUT)
deadline=time.monotonic()+30.
while True:
    status=inspect()
    if status['Running'] and (not had_sim or status['StartedAt']!=old['StartedAt']):break
    if sim.poll() is not None or time.monotonic()>deadline:
        raise RuntimeError('Simulator restart failed; see '+str(out/'simulator.log'))
    time.sleep(.2)
command=['docker','exec','rmua_noetic','bash','-c',
         'source /opt/ros/noetic/setup.bash; source /workspace/rmua_ws/devel/setup.bash; '
         'exec python3 /workspace/repo/tools/race_start_watch.py --out "$1" --control-rate "$2" --obstacle-backend "$3" --outbound-response "$4" --planner-mode "$5" ${6:+--spacetime-bridge-hold "$6"}',
         'race-start',str(container_out),str(20.*args.clock_speed),args.obstacle_backend,args.outbound_response,args.planner_mode,'' if args.spacetime_bridge_hold is None else str(args.spacetime_bridge_hold)]
print('Recording:',out,flush=True)
monitor=None
health_stamp=None
if args.mode=='background':
    monitor=subprocess.Popen([sys.executable,str(workspace/'repo/tools/watch_debug_race.py'),
        '--out',str(out),'--display',':2','--stall-seconds',str(args.stall_seconds)],
        stdout=(out/'monitor.log').open('w'),stderr=subprocess.STDOUT)
try:
    watcher=subprocess.Popen(command,cwd=workspace)
    while watcher.poll() is None:
        if health_stamp is None or time.monotonic()-health_stamp>=5.:
            health_stamp=time.monotonic()
            with (out/'system_health.jsonl').open('a') as file:
                file.write(json.dumps(system_health_snapshot())+'\n')
        if monitor is not None and monitor.poll() is not None and not (out/'stop.json').exists():
            (out/'stop.json').write_text(json.dumps(dict(reason='DEBUG_MONITOR_EXITED',
                returncode=monitor.returncode,wall_time=time.time(),official_result='UNKNOWN'),indent=2)+'\n')
        time.sleep(.2)
    code=watcher.returncode
    if code!=0 and not (out/'stop.json').exists():
        log=(out/'simulator.log').read_text(errors='replace')
        (out/'stop.json').write_text(json.dumps(dict(
            reason=('SIMULATOR_CRASH' if 'SIGSEGV' in log or 'SIGABRT' in log else 'WATCHER_FAILED'),
            watcher_returncode=code,simulator_returncode=sim.poll(),
            wall_time=time.time(),official_result='UNKNOWN'),indent=2)+'\n')
finally:
    if monitor is not None:monitor.terminate();monitor.wait(timeout=5.)
subprocess.run([sys.executable,str(workspace/'repo/tools/summarize_debug_race.py'),
                '--run',str(out)],check=True)
raise SystemExit(code)
