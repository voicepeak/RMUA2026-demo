#!/usr/bin/env python3
"""Compare lidar navigation on archived scenes using measured XYZ motion."""
import argparse
import bisect
import json
from pathlib import Path
import sys
import time
import numpy as np

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--stations',default='0,3,10,40,70,120,190,250')
    parser.add_argument('--speed',type=float,default=6.)
    parser.add_argument('--before-received',type=float,required=True)
    args=parser.parse_args()
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros_ws/src/route_follower/scripts'))
    from execution_guard import ExecutionGuard
    from lidar_navigation import LidarNavigator
    telemetry=[]
    for line in (args.run/'flight/streams.jsonl').open():
        row=json.loads(line)
        if row.get('topic')=='telemetry' and row['received']<=args.before_received:telemetry.append(row)
    telemetry.sort(key=lambda row:row['data']['pose_stamp'])
    stamps=[row['data']['pose_stamp'] for row in telemetry]
    if not stamps:parser.error('No telemetry within the supplied HUD cutoff')
    valid=[]
    for path in sorted((args.run/'mission/leg_3_5/clouds').glob('*.json')):
        meta=json.loads(path.read_text());stamp=meta['pose_stamp']
        i=min(len(stamps)-1,bisect.bisect_left(stamps,stamp))
        i=min((i,max(0,i-1)),key=lambda j:abs(stamps[j]-stamp))
        if abs(stamps[i]-stamp)<=.2:valid.append((path,meta,telemetry[i]['data']))
    if not valid:parser.error('No return-leg clouds matched valid telemetry within 0.2 s')
    selected=[]
    for station in map(float,args.stations.split(',')):
        item=min(valid,key=lambda item:abs(item[1]['s']-station))
        if item not in selected:selected.append(item)
    results=[]
    for path,meta,row in selected:
        with np.load(path.with_suffix('.npz')) as data:
            points=data['points'];position=data['position'];reference=data['reference']
        s=meta['s'];ss=s+np.arange(len(reference))
        xy=lambda t:tuple(np.interp(t,ss,reference[:,axis]) for axis in (0,1))
        center=lambda t:float(np.interp(t,ss,reference[:,2]))
        velocity=np.asarray(meta.get('measured_velocity_world',row['measured_velocity_world']))
        f=np.asarray(xy(s+1.))-np.asarray(xy(s));f/=max(1e-6,np.linalg.norm(f))
        desired=np.r_[args.speed*f,np.clip((center(s+1.)-center(s))*args.speed,-4.,4.5)]
        guard=ExecutionGuard();nav=LidarNavigator(guard)
        age=max(0.,min(.49,row.get('lidar_age') or 0.))
        started=time.monotonic()
        command,info=nav.select(position,velocity,desired,s,xy,center,points,
                                meta['pose_stamp']-age,meta['pose_stamp'],meta.get('departure_floor_offset'))
        results.append(dict(sample=path.name,s=s,measured_velocity=velocity.tolist(),
            hypothetical_desired=desired.tolist(),command=command.tolist(),
            elapsed_ms=1000.*(time.monotonic()-started),result=info))
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(dict(scope='Frozen recorded scenes; measured motion and hypothetical nominal speed. Not flight evidence.',results=results),indent=2)+'\n')
    print(json.dumps(dict(samples=len(results),feasible=sum(row['result']['feasible'] for row in results),
        reasons=[row['result']['command_reason'] for row in results],
        elapsed_ms=[round(row['elapsed_ms'],2) for row in results])))

if __name__=='__main__':main()
