#!/usr/bin/env python3
"""Start control as soon as a fresh simulator publishes its initial pose.

Run inside the container after its restart. Vision and recording launch in
parallel with control so loading a neural network cannot consume departure time.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

import rospy
from geometry_msgs.msg import PoseStamped

parser=argparse.ArgumentParser()
parser.add_argument('--out',type=Path,required=True)
args=parser.parse_args()
args.out.mkdir(parents=True,exist_ok=True)
root=Path(__file__).resolve().parents[1]
workspace=root.parent
config=workspace/'rmua_ws/src/route_follower/config'
rospy.init_node('race_start_watch')
pose=rospy.wait_for_message('/airsim_node/drone_1/debug/pose_gt',PoseStamped,timeout=20.)
position=pose.pose.position
if abs(position.x)>3. or abs(position.y)>3.:
    raise RuntimeError('Fresh seed123 start position required')
command=['roslaunch','route_follower','route_follower.launch',
         'gates_file:='+str(config/'gates_seed123_recorded.yaml'),
         'guides_file:='+str(config/'guides_seed123_recorded.yaml'),
         'gate_center_pull_max:=0','static_correction_max:=2',
         'cruise_speed:=15','max_speed:=15','slope_eta:=0.95','vz_down_limit:=4.5',
         'z_rate_max:=5','vz_capability_file:='+str(config/'vz_capability_seed123_fast.yaml')]
child=subprocess.Popen(command,stdout=(args.out/'controller.log').open('w'),stderr=subprocess.STDOUT)
(args.out/'startup.json').write_text(json.dumps(dict(wall_time=time.time(),
    first_pose_stamp=pose.header.stamp.to_sec(),command=command),indent=2))
subprocess.Popen(['python3',str(workspace/'frames/flight_probe.py'),
                  '--out',str(args.out/'flight'),'--seconds','1200',
                  '--image-interval','0.5','--save-right'],
                 stdout=(args.out/'probe.log').open('w'),stderr=subprocess.STDOUT)
subprocess.Popen(['/opt/conda/envs/xal/bin/python',
                  str(workspace/'rmua_ws/src/rmua_gate_vision/scripts/gate_yolo_node.py'),
                  '_model:='+str(root/'yolo/weights/best.pt'),
                  '_route_file:='+str(config/'route_1_3.yaml'),'_imgsz:=960','_conf:=0.35'],
                 stdout=(args.out/'vision.log').open('w'),stderr=subprocess.STDOUT)
runner=subprocess.Popen(['python3',str(root/'tools/race_runner.py'),
                         '--out',str(args.out/'mission'),'--cruise','15',
                         '--fast-descent','--height-trace',str(args.out/'flight/streams.jsonl')],
                        stdout=(args.out/'runner.log').open('w'),stderr=subprocess.STDOUT)
code=child.wait()
if code:raise SystemExit(code)
raise SystemExit(runner.wait())
