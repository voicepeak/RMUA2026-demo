#!/usr/bin/env python3
"""One command: fresh simulator, immediate control, recording and stage runner."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import hashlib
import shutil

parser=argparse.ArgumentParser()
parser.add_argument('--mode',choices=('render','offscreen','background'),default='render')
parser.add_argument('--clock-speed',type=float,default=1.)
parser.add_argument('--stall-seconds',type=float,default=20.)
parser.add_argument('--obstacle-backend',choices=('lidar_nav','legacy'),default='lidar_nav')
parser.add_argument('--out',type=Path)
args=parser.parse_args()
if not 0.<args.clock_speed<=2.:parser.error('--clock-speed must be in (0, 2]')
workspace=Path(__file__).resolve().parents[2]
out=(args.out or workspace/'frames'/time.strftime('race_%Y%m%d_%H%M%S')).resolve()
out.relative_to(workspace)  # The container mount must be able to see recordings.
out.mkdir(parents=True,exist_ok=True)
container_out=Path('/workspace')/out.relative_to(workspace)
settings=json.loads((workspace/'simulator/simulator_12.0.0.5/settings.json').read_text())
settings['ClockSpeed']=args.clock_speed
(out/'settings.json').write_text(json.dumps(settings,indent=2)+'\n')
source=workspace/'rmua_ws/src/route_follower'
snapshot=out/'controller_sources'
if snapshot.exists():raise RuntimeError('Use a new experiment directory; controller snapshot already exists')
shutil.copytree(source,snapshot,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
hashes={str(path.relative_to(out)):hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(snapshot.rglob('*')) if path.is_file()}
(out/'controller_versions.json').write_text(json.dumps(dict(wall_time=time.time(),seed=123,
    requested_clock_speed=args.clock_speed,mode=args.mode,backend=args.obstacle_backend,
    car_model_enabled=args.obstacle_backend=='legacy',snapshot_kind='runtime package captured before launch',
    sha256=hashes),indent=2)+'\n')
def inspect():
    result=subprocess.run(['docker','inspect','rmua_noetic'],capture_output=True,text=True,check=True)
    return json.loads(result.stdout)[0]['State']
old=inspect()
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
         'exec python3 /workspace/repo/tools/race_start_watch.py --out "$1" --control-rate "$2" --obstacle-backend "$3"',
         'race-start',str(container_out),str(20.*args.clock_speed),args.obstacle_backend]
print('Recording:',out,flush=True)
monitor=None
if args.mode=='background':
    monitor=subprocess.Popen([sys.executable,str(workspace/'repo/tools/watch_debug_race.py'),
        '--out',str(out),'--display',':2','--stall-seconds',str(args.stall_seconds)],
        stdout=(out/'monitor.log').open('w'),stderr=subprocess.STDOUT)
try:
    watcher=subprocess.Popen(command,cwd=workspace)
    while watcher.poll() is None:
        if monitor is not None and monitor.poll() is not None and not (out/'stop.json').exists():
            (out/'stop.json').write_text(json.dumps(dict(reason='DEBUG_MONITOR_EXITED',
                returncode=monitor.returncode,wall_time=time.time(),official_result='UNKNOWN'),indent=2)+'\n')
        time.sleep(.2)
    code=watcher.returncode
finally:
    if monitor is not None:monitor.terminate();monitor.wait(timeout=5.)
subprocess.run([sys.executable,str(workspace/'repo/tools/summarize_debug_race.py'),
                '--run',str(out)],check=True)
raise SystemExit(code)
