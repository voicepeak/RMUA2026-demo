#!/usr/bin/env python3
"""Start control as soon as a fresh simulator publishes its initial pose.

Run inside the container after its restart. Vision and recording launch in
parallel with control so loading a neural network cannot consume departure time.
"""
import argparse
import math
import json
from pathlib import Path
import subprocess
import os
import signal
import time

import rospy
from geometry_msgs.msg import PoseStamped

parser=argparse.ArgumentParser()
parser.add_argument('--out',type=Path,required=True)
parser.add_argument('--control-rate',type=float,default=20.)
parser.add_argument('--obstacle-backend',choices=('lidar_nav','legacy'),default='lidar_nav')
parser.add_argument('--planner-mode',choices=('legacy','spacetime','longitudinal'),default='legacy')
parser.add_argument('--spacetime-bridge-hold',type=float,help='Measured external bridge command expiry in seconds; absent means no certified motion')
parser.add_argument('--outbound-response',choices=('legacy','coupled'),default='coupled')
args=parser.parse_args()
if not math.isfinite(args.control_rate) or args.control_rate<=0:parser.error("control-rate must be positive and finite")
if args.planner_mode in ('spacetime','longitudinal') and args.obstacle_backend!='lidar_nav':parser.error(args.planner_mode+' requires lidar_nav')
if args.spacetime_bridge_hold is not None:
    interval=1./args.control_rate
    if not math.isfinite(args.spacetime_bridge_hold) or not 0.<args.spacetime_bridge_hold<=interval:
        parser.error('Measured bridge expiry must be positive and no longer than the control period')

args.out.mkdir(parents=True,exist_ok=True)
root=Path(__file__).resolve().parents[1]
workspace=root.parent
config=workspace/'rmua_ws/src/route_follower/config'
from race_runner import outbound_response_arguments
rospy.init_node('race_start_watch')
pose=rospy.wait_for_message('/airsim_node/drone_1/debug/pose_gt',PoseStamped,timeout=20.)
position=pose.pose.position
if abs(position.x)>3. or abs(position.y)>3.:
    raise RuntimeError('Fresh seed123 start position required')
command=['roslaunch','route_follower','route_follower.launch',
         'control_rate:='+str(args.control_rate),
         'obstacle_backend:='+args.obstacle_backend,
         'gates_file:='+str(config/'gates_seed123_recorded.yaml'),
         'guides_file:='+str(config/'guides_seed123_recorded.yaml'),
         'gate_center_pull_max:=0','static_correction_max:=2',
         'cruise_speed:=40','max_speed:=40','adaptive_speed:=true','lidar_braking:=8',
         'curve_preview_max:=100','curve_preview_step:=1','z_response_time:=0.15',
         'terminal_hover_height:=1.5','lidar_height_gain:=1.0','lidar_local_replan:=false','lidar_path_options:=false',
         'lidar_lift_gain:=0.11','lidar_coupling_gain_min:=0.09','lidar_envelope_margin:=1.15',
         'sensor_reaction:=0.35',
         'lidar_anticipation_distance:=0','debug_cloud_dir:='+str(args.out/'clouds'),
         'slope_eta:=0.95','vz_down_limit:=4.5','a_up:=8','a_down:=8',
         'z_rate_max:=5','vz_capability_file:='+str(config/'vz_capability_seed123_fast.yaml')]
command+=outbound_response_arguments(args.outbound_response)
command+=['planner_mode:='+args.planner_mode]
if args.spacetime_bridge_hold is not None:
    command+=['spacetime_bridge_verified:=true','spacetime_bridge_hold:='+str(args.spacetime_bridge_hold)]
trace=subprocess.Popen(['python3',str(root/'tools/control_trace.py'),
                        '--out',str(args.out/'control_trace'),'--save-clouds'],
                       stdout=(args.out/'control_trace.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
child=subprocess.Popen(command,stdout=(args.out/'controller.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
(args.out/'startup.json').write_text(json.dumps(dict(wall_time=time.time(),
    first_pose_stamp=pose.header.stamp.to_sec(),command=command),indent=2))
recorder=subprocess.Popen(['python3',str(workspace/'frames/flight_probe.py'),
                  '--out',str(args.out/'flight'),'--seconds','1200',
                  '--image-interval','0.5','--save-right'],
                 stdout=(args.out/'probe.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
vision=subprocess.Popen(['/opt/conda/envs/xal/bin/python',
                  str(workspace/'rmua_ws/src/rmua_gate_vision/scripts/gate_yolo_node.py'),
                  '_model:='+str(root/'yolo/weights/best.pt'),
                  '_car_model:='+(str(root/'yolo/weights/car_score91_best.pt') if args.obstacle_backend=='legacy' else ''),
                  '_route_file:='+str(config/'route_1_3.yaml'),'_imgsz:=960','_conf:=0.35'],
                 stdout=(args.out/'vision.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
runner=subprocess.Popen(['python3',str(root/'tools/race_runner.py'),
                         '--out',str(args.out/'mission'),'--cruise','40',
                         '--fast-descent','--adaptive-speed','--control-rate',str(args.control_rate),
                         '--obstacle-backend',args.obstacle_backend,'--planner-mode',args.planner_mode,
                         *([] if args.spacetime_bridge_hold is None else ['--spacetime-bridge-hold',str(args.spacetime_bridge_hold)]),
                         '--height-trace',str(args.out/'flight/streams.jsonl')],
                        stdout=(args.out/'runner.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
last_pose_arrival=time.monotonic()
def received_pose(_):
    global last_pose_arrival
    last_pose_arrival=time.monotonic()
rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt',PoseStamped,received_pose,queue_size=1)
try:
    while not rospy.is_shutdown():
        if (args.out/'stop.json').exists():break
        if time.monotonic()-last_pose_arrival>3.:
            (args.out/'simulator_lost.json').write_text(json.dumps(dict(reason='POSE_STREAM_LOST',wall_time=time.time())))
            raise SystemExit('Simulator pose stream disappeared; see simulator.log')
        if child.poll() is not None and child.returncode:raise SystemExit(child.returncode)
        if runner.poll() is not None:raise SystemExit(runner.returncode)
        time.sleep(.2)
finally:
    for process in (child,runner,recorder,vision,trace):
        try:os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:pass
    for process in (child,runner,recorder,vision,trace):
        try:process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            try:os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:pass
    # A stage switch in progress can launch descendants after the first TERM.
    # All group leaders have now exited; remove any remaining descendants.
    for process in (child,runner,recorder,vision,trace):
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
    # roslaunch may give a switched controller its own process group. Remove
    # its registered nodes too, after the runner can no longer launch a leg.
    try:
        subprocess.run(['rosnode','kill','/route_follower','/gate_yolo'],
                       stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=5.)
    except subprocess.TimeoutExpired:pass
