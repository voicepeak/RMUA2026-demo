#!/usr/bin/env python3
"""Launch a fresh official simulator and the bounded motion probe."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import shutil
import hashlib


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--levels',default='2,4,6,8')
    parser.add_argument('--slew',type=float,default=0.)
    parser.add_argument('--drive-seconds',type=float,default=2.)
    parser.add_argument('--compensation',type=float,default=0.)
    parser.add_argument('--brake-feedback',type=float,default=0.)
    parser.add_argument('--start-x',type=float,default=8.)
    parser.add_argument('--brake-slew',type=float,default=0.)
    parser.add_argument('--clock-type',choices=('auto','ScalableClock','SteppableClock'),default='auto')
    parser.add_argument('--lidar-point-rate',type=int)
    parser.add_argument('--vertical-command-max',type=float,default=2.5)
    parser.add_argument('--vertical-step',type=float,default=0.)
    args=parser.parse_args()
    if args.lidar_point_rate is not None and args.lidar_point_rate<=0:parser.error('--lidar-point-rate must be positive')
    workspace=Path(__file__).resolve().parents[2]
    out=args.out.resolve();rel=out.relative_to(workspace);out.mkdir(parents=True,exist_ok=False)
    settings=json.loads((workspace/'simulator/simulator_12.0.0.5/settings.json').read_text())
    settings['ClockSpeed']=1.
    if args.clock_type!='auto':settings['ClockType']=args.clock_type
    if args.lidar_point_rate is not None:settings['Vehicles']['drone_1']['Sensors']['lidar']['PointsPerSecond']=args.lidar_point_rate
    (out/'settings.json').write_text(json.dumps(settings,indent=2)+'\n')
    (out/'sources').mkdir()
    for name in ('motion_probe.py','control_trace.py','start_motion_probe.py'):
        shutil.copy2(Path(__file__).with_name(name),out/'sources'/name)
    (out/'manifest.json').write_text(json.dumps(dict(arguments=vars(args)|{'out':str(args.out)},
        sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (out/'sources').iterdir()}),indent=2)+'\n')
    def inspect():
        return json.loads(subprocess.check_output(['docker','inspect','rmua_noetic']))[0]['State']
    old=inspect()
    simulator=subprocess.Popen([str(workspace/'run_sim.sh'),'123','background',str(Path('/workspace')/rel/'settings.json')],
        stdout=(out/'simulator.log').open('w'),stderr=subprocess.STDOUT)
    deadline=time.monotonic()+35
    while True:
        now=inspect()
        if now['Running'] and now['StartedAt']!=old['StartedAt']:break
        if simulator.poll() is not None or time.monotonic()>deadline:raise RuntimeError('simulator restart failed')
        time.sleep(.2)
    container_out=Path('/workspace')/rel
    def launch(script,arguments,log):
        return subprocess.Popen(['docker','exec','rmua_noetic','bash','-c',
            'source /opt/ros/noetic/setup.bash; source /workspace/rmua_ws/devel/setup.bash; exec python3 "$@"',
            'probe',str(Path('/workspace/repo/tools')/script),*arguments],
            stdout=(out/log).open('w'),stderr=subprocess.STDOUT)
    recorder=launch('control_trace.py',['--out',str(container_out/'trace'),'--save-clouds'],'trace.log')
    probe=launch('motion_probe.py',['--out',str(container_out),'--levels',args.levels,
                 '--slew',str(args.slew),'--drive-seconds',str(args.drive_seconds),
                 '--compensation',str(args.compensation),'--brake-feedback',str(args.brake_feedback),
                 '--start-x',str(args.start_x),'--brake-slew',str(args.brake_slew),
                 '--vertical-command-max',str(args.vertical_command_max),
                 '--vertical-step',str(args.vertical_step)],'probe.log')
    try:
        code=probe.wait(timeout=180.)
    finally:
        subprocess.run(['docker','exec','rmua_noetic','bash','-c',
            'source /opt/ros/noetic/setup.bash; rosnode kill $(rosnode list | sed -n "/control_trace/p")'],
            stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        recorder.wait(timeout=5.)
    print(json.dumps(dict(out=str(out),returncode=code)),flush=True)
    raise SystemExit(code)


if __name__=='__main__':main()
