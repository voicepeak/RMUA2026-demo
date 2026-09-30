#!/usr/bin/env python3
"""One command: fresh simulator, immediate control, recording and stage runner."""
import argparse
import json
from pathlib import Path
import subprocess
import time

parser=argparse.ArgumentParser()
parser.add_argument('--mode',choices=('render','offscreen'),default='render')
parser.add_argument('--out',type=Path)
args=parser.parse_args()
workspace=Path(__file__).resolve().parents[2]
out=(args.out or workspace/'frames'/time.strftime('race_%Y%m%d_%H%M%S')).resolve()
out.relative_to(workspace)  # The container mount must be able to see recordings.
out.mkdir(parents=True,exist_ok=True)
def inspect():
    result=subprocess.run(['docker','inspect','rmua_noetic'],capture_output=True,text=True,check=True)
    return json.loads(result.stdout)[0]['State']
old=inspect()
check=subprocess.run(['docker','exec','rmua_noetic','pgrep','-f','RMUA-Linux'],capture_output=True)
had_sim=check.returncode==0
sim=subprocess.Popen([str(workspace/'run_sim.sh'),'123',args.mode],cwd=workspace,
                     stdout=(out/'simulator.log').open('w'),stderr=subprocess.STDOUT)
deadline=time.monotonic()+30.
while True:
    status=inspect()
    if status['Running'] and (not had_sim or status['StartedAt']!=old['StartedAt']):break
    if sim.poll() is not None or time.monotonic()>deadline:
        raise RuntimeError('Simulator restart failed; see '+str(out/'simulator.log'))
    time.sleep(.2)
container_out=Path('/workspace')/out.relative_to(workspace)
command=['docker','exec','rmua_noetic','bash','-c',
         'source /opt/ros/noetic/setup.bash; source /workspace/rmua_ws/devel/setup.bash; '
         'exec python3 /workspace/repo/tools/race_start_watch.py --out "$1"',
         'race-start',str(container_out)]
print('Recording:',out,flush=True)
raise SystemExit(subprocess.call(command,cwd=workspace))
