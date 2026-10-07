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
    parser.add_argument('--height-gain',type=float,default=0.)
    parser.add_argument('--compensation-error-max',type=float,default=1e6)
    parser.add_argument('--compensation-attitude',action='store_true')
    parser.add_argument('--brake-feedback',type=float,default=0.)
    parser.add_argument('--start-x',type=float,default=8.)
    parser.add_argument('--brake-slew',type=float,default=0.)
    parser.add_argument('--clock-type',choices=('auto','ScalableClock','SteppableClock'),default='auto')
    parser.add_argument('--lidar-point-rate',type=int)
    parser.add_argument('--vertical-command-max',type=float,default=2.5)
    parser.add_argument('--vertical-step',type=float,default=0.)
    parser.add_argument('--curve-brake-radius',type=float,default=0.)
    parser.add_argument('--endpoint-velocity',action='store_true')
    parser.add_argument('--coupling-limited',action='store_true')
    parser.add_argument('--xy-error-max',type=float,default=float('inf'))
    parser.add_argument('--control-period',type=float,default=.02)
    args=parser.parse_args()
    if not .01<=args.control_period<=.35:parser.error('--control-period must be between 0.01 and 0.35 seconds')
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
    if args.curve_brake_radius or args.endpoint_velocity or args.height_gain or args.coupling_limited:
        shutil.copytree(workspace/'repo/ros_ws/src/route_follower',out/'controller_sources',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
        native=workspace/'rmua_ws/devel/lib/libvelocity_response_native.so'
        if native.exists():shutil.copy2(native,out/'sources'/native.name)
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
    probe_args=['--out',str(container_out),'--levels',args.levels,
                 '--slew',str(args.slew),'--drive-seconds',str(args.drive_seconds),
                 '--compensation',str(args.compensation),'--height-gain',str(args.height_gain),'--brake-feedback',str(args.brake_feedback),
                 '--compensation-error-max',str(args.compensation_error_max),
                 '--start-x',str(args.start_x),'--brake-slew',str(args.brake_slew),
                 '--vertical-command-max',str(args.vertical_command_max),
                 '--vertical-step',str(args.vertical_step),
                 '--curve-brake-radius',str(args.curve_brake_radius),
                 '--control-period',str(args.control_period),'--xy-error-max',str(args.xy_error_max)]
    if args.compensation_attitude:probe_args.append('--compensation-attitude')
    if args.endpoint_velocity:probe_args.append('--endpoint-velocity')
    if args.coupling_limited:probe_args.append('--coupling-limited')
    probe=launch('motion_probe.py',probe_args,'probe.log')
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
