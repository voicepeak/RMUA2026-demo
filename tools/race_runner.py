#!/usr/bin/env python3
"""Run course legs with explicit, separately logged simulator/local progress.

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

def goal_matches_road(splines,road,goal):
    point=splines[road-1][0]
    return math.hypot(point[0]-goal[0],point[1]-goal[1])<35.

def controller_command(route_file,gates_file,guides_file,cruise,fast_descent=False,adaptive_speed=False,control_rate=20.,obstacle_backend='lidar_nav',planner_mode='legacy',spacetime_bridge_hold=None):
    command=['roslaunch','route_follower','route_follower.launch',
            'route_file:='+str(route_file),'route:=race_leg',
            'gates_file:='+str(gates_file),'guides_file:='+str(guides_file),
            'gate_center_pull_max:=0','cruise_speed:='+str(cruise),'control_rate:='+str(control_rate),
            'obstacle_backend:='+obstacle_backend,
            'max_speed:='+str(max(12.,cruise))]
    command+=['planner_mode:='+planner_mode]
    if spacetime_bridge_hold is not None:
        command+=['spacetime_bridge_verified:=true','spacetime_bridge_hold:='+str(spacetime_bridge_hold)]
    if adaptive_speed:command+=['adaptive_speed:=true','lidar_braking:=8','curve_preview_max:=100',
                               'terminal_hover_height:=1.5','debug_cloud_dir:='+str(Path(route_file).parent/'clouds'),
                               'curve_preview_step:=1','z_response_time:=0.15']
    if fast_descent:
        config=Path(__file__).resolve().parents[1]/'ros_ws/src/route_follower/config'
        command+=['slope_eta:=0.95','vz_down_limit:=4.5','z_rate_max:=5',
                  'vz_capability_file:='+str(config/'vz_capability_seed123_fast.yaml')]
    return command

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

def recorded_return_guides(route,incoming,height_trace):
    guides=[]
    for p in height_trace:
        _,_,distance,_=incoming.project(p)
        if distance>10.:continue
        s=route.project_gate(p[0],p[1])
        # The previous controller lowered to the marker to trigger the stage.
        # Replaying that maneuver in reverse hits the departure window sill.
        # Let the current pose and the first gate anchor define departure.
        if s<40.:continue
        guides.append(dict(s=s,z=p[2]))
    guides.sort(key=lambda g:g['s'])
    sampled=[]
    for g in guides:
        if not sampled or g['s']-sampled[-1]['s']>=8.:sampled.append(g)
    return sampled

def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    ap=argparse.ArgumentParser()
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--cruise',type=float,default=10.)
    ap.add_argument('--stage',type=int,default=0)
    ap.add_argument('--start-current-leg',action='store_true',
                    help='Resume the selected leg at its starting marker')
    ap.add_argument('--continue-on-route-end',action='store_true',
                    help='Continue local flight without claiming official completion')
    ap.add_argument('--measured-map',type=Path)
    ap.add_argument('--height-trace',type=Path,
                    help='JSONL flight telemetry used only as return-road height guides')
    ap.add_argument('--fast-descent',action='store_true')
    ap.add_argument('--adaptive-speed',action='store_true')
    ap.add_argument('--control-rate',type=float,default=20.)
    ap.add_argument('--obstacle-backend',choices=('lidar_nav','legacy'),default='lidar_nav')
    ap.add_argument('--planner-mode',choices=('legacy','spacetime'),default='legacy')
    ap.add_argument('--spacetime-bridge-hold',type=float,help='Measured external bridge command expiry in seconds; absent means no certified motion')
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    if not math.isfinite(a.control_rate) or a.control_rate<=0:ap.error("control-rate must be positive and finite")
    if a.planner_mode=='spacetime' and a.obstacle_backend!='lidar_nav':ap.error('spacetime requires lidar_nav')
    if a.spacetime_bridge_hold is not None and (not math.isfinite(a.spacetime_bridge_hold) or not 0.<a.spacetime_bridge_hold<=1./a.control_rate):
        ap.error('Measured bridge expiry must be positive and no longer than the control period')
    if not 0<=a.stage<3:ap.error('--stage must be 0, 1 or 2 for implemented racing legs')
    root=Path(__file__).resolve().parents[1]
    config=root/'ros_ws/src/route_follower/config'
    splines=load_splines(config/'splines.txt')
    measured=yaml.safe_load((config/'gates_seed123_recorded.yaml').read_text())['gates']
    if a.measured_map:
        measured=yaml.safe_load(a.measured_map.read_text())['gates']
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
    def on_event(m):
        try:
            data=json.loads(m.data)
            if data.get('reason')=='ROUTE_END':state['route_end']=True
        except ValueError:pass
    rospy.Subscriber('/rmua/controller/events',String,on_event)
    stage=a.stage;previous=None;child=None
    log=(a.out/'mission.jsonl').open('a',buffering=1)
    def event(kind,**fields):
        payload=dict(kind=kind,stamp=time.time(),stage=stage,**fields)
        log.write(json.dumps(payload)+'\n');print(json.dumps(payload),flush=True)
    def start_leg(goal,cause):
        nonlocal child
        if not 0<=stage<3:
            event('FACTORY_INSPECTION_REQUIRED',leg=[SEQUENCE[stage],SEQUENCE[stage+1]])
            return False
        reusable=list(measured)
        for g in state.get('gates',[]):
            if not g.get('hard_anchor') or g.get('support',0)<8:continue
            if any(math.hypot(g['x']-p['x'],g['y']-p['y'])<8. for p in reusable):continue
            reusable.append(g)
        try:
            points,gates=build_leg(splines,SEQUENCE[stage],SEQUENCE[stage+1],goal,reusable)
        except ValueError as error:
            event('ROUTE_UNAVAILABLE',reason=str(error));return False
        position=state.get('pose')
        if position is None or math.hypot(position[0]-points[0][0],position[1]-points[0][1])>30.:
            event('START_MARKER_MISMATCH',pose=position);return False
        # Begin the return from the actual trigger position, without snapping
        # sideways to the exported spline before the forward cameras turn.
        points.insert(0,position)
        route=RouteGeometry(points)
        for g in gates:g['s']=route.project_gate(g['x'],g['y'])
        guides=[]
        incoming=RouteGeometry(splines[SEQUENCE[stage]-1])
        height_trace=[]
        if a.height_trace:
            for line in a.height_trace.open():
                try:row=json.loads(line)
                except ValueError:continue  # Recorder may be writing its final line.
                if row.get('topic')=='telemetry':
                    data=row['data']
                    height_trace.append((*data['path_xy'],data.get('z_ref_raw',data['z_ref'])))
        sampled=recorded_return_guides(route,incoming,height_trace)
        folder=a.out/('leg_%d_%d'%(SEQUENCE[stage],SEQUENCE[stage+1]));folder.mkdir(exist_ok=True)
        route_file=folder/'route.yaml';gates_file=folder/'gates.yaml';guides_file=folder/'guides.yaml'
        route_file.write_text(yaml.safe_dump({'routes':{'race_leg':points}}))
        gates_file.write_text(yaml.safe_dump({'gates':gates}))
        guides_file.write_text(yaml.safe_dump({'altitude_guides':sampled}))
        subprocess.run(['rosnode','kill','/route_follower'],check=False)
        state.pop('route_end',None)
        # A fresh vision process reads the new route; clearing the old map alone
        # leaves its projection/pruning tied to the previous road direction.
        subprocess.run(['rosnode','kill','/gate_yolo'],check=False)
        subprocess.Popen(['/opt/conda/envs/xal/bin/python',
                          str(root/'ros_ws/src/rmua_gate_vision/scripts/gate_yolo_node.py'),
                          '_model:='+str(root/'yolo/weights/best.pt'),
                          '_car_model:='+(str(root/'yolo/weights/car_score91_best.pt') if a.obstacle_backend=='legacy' else ''),
                          '_route_file:='+str(route_file),'_route_name:=race_leg',
                          '_imgsz:=960','_conf:=0.35'],
                         stdout=(folder/'vision.log').open('w'),stderr=subprocess.STDOUT)
        command=controller_command(route_file,gates_file,guides_file,a.cruise,a.fast_descent,a.adaptive_speed,a.control_rate,a.obstacle_backend,a.planner_mode,a.spacetime_bridge_hold)
        if cause=='OFFICIAL_ENDPOINT_CHANGED' and previous is not None:
            command+=['departure_hover_z:='+str(previous[2]-1.5)]
        child=subprocess.Popen(command,stdout=(folder/'controller.log').open('w'),stderr=subprocess.STDOUT)
        event('CONTROLLER_STARTED',leg=[SEQUENCE[stage],SEQUENCE[stage+1]],
              cause=cause,official_completion='UNKNOWN',command=command)
        return True
    while not rospy.is_shutdown():
        current=state.get('goal')
        if current is not None and previous is None:
            if not (goal_matches_road(splines,SEQUENCE[stage+1],current) or
                    (a.start_current_leg and goal_matches_road(splines,SEQUENCE[stage],current))):
                # During UE startup the goal topic briefly publishes (0,0,0).
                # This is initialization, not completion of the first leg.
                time.sleep(.05)
                continue
            previous=current;event('OBSERVE',goal=current)
            if a.start_current_leg:
                if not start_leg(splines[SEQUENCE[stage+1]-1][0], 'EXPLICIT_RESUME'):break
        elif current is not None and math.dist(current,previous)>20.:
            expected=splines[SEQUENCE[stage+1]-1][0]
            if child is not None and math.hypot(current[0]-expected[0],current[1]-expected[1])<35.:
                previous=current;event('ACTIVE_LEG_GOAL_CONFIRMED',goal=current)
                continue
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
            if not start_leg(current,'OFFICIAL_ENDPOINT_CHANGED'):break
            previous=current
        if a.continue_on_route_end and state.pop('route_end',False):
            position=state.get('pose');expected=splines[SEQUENCE[stage+1]-1][0]
            if position is None or math.hypot(position[0]-expected[0],position[1]-expected[1])>20.:
                event('UNVERIFIED_ROUTE_END',pose=position);break
            event('LOCAL_ROUTE_END',official_completion='UNKNOWN')
            stage+=1
            if stage>=3:
                event('FACTORY_INSPECTION_REQUIRED');break
            if not start_leg(splines[SEQUENCE[stage+1]-1][0],'LOCAL_CONTINUATION'):break
        if child is not None and child.poll() is not None:
            event('CONTROLLER_EXIT',code=child.returncode);break
        time.sleep(.05)
    log.close()

if __name__=='__main__':main()
