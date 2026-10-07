#!/usr/bin/env python3
"""Replay an archived stopped scene with the planner unavailable.

Reconstruct the last certified braking profile from telemetry. The forced
planner-unavailable state is a counterfactual, since archived reference curves
are sampled at one metre and cannot reproduce every online search result.
No ROS publication or simulator mutation occurs.
"""
import argparse
import bisect
import json
from pathlib import Path
import sys
import numpy as np


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--stations',default='320.56389594535716,568.4448958615092')
    parser.add_argument('--controller-snapshot',type=Path)
    args=parser.parse_args()
    scripts=(args.controller_snapshot/'scripts' if args.controller_snapshot else
             Path(__file__).resolve().parents[1]/'ros_ws/src/route_follower/scripts')
    sys.path.insert(0,str(scripts))
    from execution_guard import ExecutionGuard
    from lidar_navigation import LidarNavigator
    from velocity_response import VelocityResponse
    cutoff=json.loads((args.run/'result.json').read_text())['last_valid_hud']['wall_time']
    telemetry=[]
    for line in (args.run/'flight/streams.jsonl').open():
        row=json.loads(line)
        if row['topic']=='telemetry' and row['received']<=cutoff:telemetry.append(row['data'])
    telemetry.sort(key=lambda r:r['pose_stamp']);times=[r['pose_stamp'] for r in telemetry]
    scenes=[]
    for p in sorted((args.run/'mission/leg_3_5/clouds').glob('*.json')):
        meta=json.loads(p.read_text())
        if meta['clearance']['command_reason']=='COMMAND_BRAKING' and not meta['clearance'].get('path'):
            scenes.append((p,meta))
    results=[]
    for station in map(float,args.stations.split(',')):
        p,meta=min(scenes,key=lambda pair:abs(pair[1]['s']-station))
        stamp=meta['pose_stamp'];i=bisect.bisect_right(times,stamp)-1
        if i<1:continue
        previous=telemetry[i-1];braking=None
        for row in telemetry[:i+1]:
            if (row['clearance']['command_reason'] in ('LIDAR_TRACK','LIDAR_BRAKE') and
                    row['clearance'].get('path')):
                braking=LidarNavigator._command_profile(np.array(row['clearance']['path']))
        if braking is None:continue
        with np.load(p.with_suffix('.npz')) as data:
            position=data['position'];points=data['points'];reference=data['reference']
        s=meta['s'];ss=s+np.arange(len(reference))
        xy=lambda t:tuple(np.interp(t,ss,reference[:,axis]) for axis in (0,1))
        center=lambda t:float(np.interp(t,ss,reference[:,2]))
        velocity=np.asarray(meta['measured_velocity_world'])
        old_target=position[2]+LidarNavigator._stop_profile(braking)(position[None,:],np.zeros((1,3)))[0]
        guard=ExecutionGuard();guard.response_model=VelocityResponse(lift_gain=.110,coupling_gains=(.09,.110,.13))
        guard.response_model.commit(np.r_[previous['velocity_world'],-previous['vz_command']],
                                    np.asarray(previous['measured_velocity_world']),previous['pose_stamp'])
        nav=LidarNavigator(guard);nav.braking_path=braking.copy()
        nav.path=lambda *args,**kwargs:(None,{})
        age=max(0.,telemetry[i]['lidar_age'] or 0.)
        command,info=nav.select(position,velocity,np.zeros(3),s,xy,center,points,
                                stamp-age,stamp,meta['departure_floor_offset'])
        results.append(dict(scene=str(p),s=s,position=position.tolist(),velocity=velocity.tolist(),
                            previous_stop_target=float(old_target),road_reference=center(s),
                            command=command.tolist(),result=info))
    output=dict(scope=__doc__,controller_scripts=str(scripts),results=results)
    args.out.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps([dict(s=r['s'],old=r['previous_stop_target'],road=r['road_reference'],
                          command=r['command'],reason=r['result']['command_reason']) for r in results]))


if __name__=='__main__':main()
