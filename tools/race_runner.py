#!/usr/bin/env python3
"""Advance local controllers only when the simulator changes its end_goal.

Run alongside the first route controller. Never infer a segment completion from
controller PASS or route progress. Factory legs require a separate inspection
implementation, and are stopped explicitly rather than reporting a guessed meter.
"""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros_ws/src/route_follower/scripts'))
from splines_to_yaml import load_splines,connect_routes
from reference_planner import RouteGeometry

SEQUENCE=(1,3,5,8,12,10,7)

def build_leg(splines,start,finish,goal,measured):
    incoming=splines[start-1]
    outgoing=list(reversed(splines[finish-1]))
    if math.hypot(outgoing[-1][0]-goal[0],outgoing[-1][1]-goal[1])>35.:
        raise ValueError('Published endpoint does not match supplied spline')
    if min(len(incoming),len(outgoing))<20:
        raise ValueError('Incomplete road spline')
    route=RouteGeometry(connect_routes(incoming,outgoing))
    gates=[]
    for raw in measured:
        i,t,d,c=route.project((raw['x'],raw['y'],raw['z']))
        if d>25.:continue
        g=dict(raw);g['s']=route.seg_s[i]+t*route.seg_len[i]
        g['nx'],g['ny'],g['nz']=route.tangent(g['s'])
        gates.append(g)
    gates.sort(key=lambda g:g['s'])
    for i,g in enumerate(gates):g['id']=i
    return route.points,gates

def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    ap=argparse.ArgumentParser()
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--cruise',type=float,default=10.)
    ap.add_argument('--stage',type=int,default=0)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[1]
    config=root/'ros_ws/src/route_follower/config'
    splines=load_splines(config/'splines.txt')
    measured=yaml.safe_load((config/'gates_seed123_recorded.yaml').read_text())['gates']
    state={};rospy.init_node('race_runner')
    def pose(m):
        p=m.pose.position;state['pose']=(p.x,p.y,p.z)
    def goal(m):
        p=m.pose.position;state['goal']=(p.x,p.y,p.z)
    def on_gates(m):
        try:state['gates']=json.loads(m.data)
        except ValueError:pass
    rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt',PoseStamped,pose)
    rospy.Subscriber('/airsim_node/end_goal',PoseStamped,goal)
    rospy.Subscriber('/rmua/gate_map',String,on_gates)
    stage=a.stage;previous=None;child=None
    log=(a.out/'mission.jsonl').open('a',buffering=1)
    def event(kind,**fields):
        payload=dict(kind=kind,stamp=time.time(),stage=stage,**fields)
        log.write(json.dumps(payload)+'\n');print(json.dumps(payload),flush=True)
    while not rospy.is_shutdown():
        current=state.get('goal')
        if current is not None and previous is None:
            previous=current;event('OBSERVE',goal=current)
        elif current is not None and math.dist(current,previous)>20.:
            position=state.get('pose')
            if position is None or math.dist(position,previous)>20.:
                event('UNVERIFIED_GOAL_CHANGE',goal=current,pose=position)
                break
            stage+=1
            event('OFFICIAL_ENDPOINT_CHANGED',old_goal=previous,goal=current)
            subprocess.run(['rosnode','kill','/route_follower'],check=False)
            if stage>=len(SEQUENCE)-1:
                event('FINAL_ENDPOINT_CHANGED_REQUIRES_REVIEW');break
            if stage>=3:
                event('FACTORY_INSPECTION_REQUIRED',leg=[SEQUENCE[stage],SEQUENCE[stage+1]])
                break
            try:
                reusable=list(measured)
                for g in state.get('gates',[]):
                    if not g.get('hard_anchor') or g.get('support',0)<8:continue
                    if any(math.hypot(g['x']-p['x'],g['y']-p['y'])<8. for p in reusable):continue
                    reusable.append(g)
                points,gates=build_leg(splines,SEQUENCE[stage],SEQUENCE[stage+1],current,reusable)
            except ValueError as error:
                event('ROUTE_UNAVAILABLE',reason=str(error));break
            folder=a.out/('leg_%d_%d'%(SEQUENCE[stage],SEQUENCE[stage+1]));folder.mkdir(exist_ok=True)
            route_file=folder/'route.yaml';gates_file=folder/'gates.yaml'
            route_file.write_text(yaml.safe_dump({'routes':{'race_leg':points}}))
            gates_file.write_text(yaml.safe_dump({'gates':gates}))
            rospy.ServiceProxy('/gate_yolo/clear',Trigger)()
            command=['roslaunch','route_follower','route_follower.launch',
                     'route_file:='+str(route_file),'route_name:=race_leg',
                     'gates_file:='+str(gates_file),'gate_center_pull_max:=0',
                     'cruise_speed:='+str(a.cruise)]
            child=subprocess.Popen(command,stdout=(folder/'controller.log').open('w'),stderr=subprocess.STDOUT)
            event('CONTROLLER_STARTED',leg=[SEQUENCE[stage],SEQUENCE[stage+1]],command=command)
            previous=current
        time.sleep(.05)
    log.close()

if __name__=='__main__':main()
