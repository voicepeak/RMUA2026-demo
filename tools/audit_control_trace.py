#!/usr/bin/env python3
"""Audit every archived lidar frame against its emitting aircraft center.

This is a sampled surface-distance audit, not a collision or full-volume proof.
SensorLocalFrame distances are rotation invariant, so no controller world
transform or predicted trajectory is required.
"""
import argparse
import bisect
import hashlib
import json
from pathlib import Path
import numpy as np


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    result=json.loads((args.run/'result.json').read_text())
    cutoff=result['last_valid_hud']['wall_time']
    settings=json.loads((args.run/'settings.json').read_text())
    sensor=settings['Vehicles']['drone_1']['Sensors']['lidar']
    if sensor['DataFrame']!='SensorLocalFrame' or any(sensor.get(k,0)!=0 for k in ('Roll','Pitch','Yaw')):
        parser.error('This audit requires the archived zero-rotation SensorLocalFrame configuration')
    offset=np.array([sensor.get(k,0.) for k in ('X','Y','Z')])
    telemetry=[];stage=0;last_s=None
    for line in (args.run/'flight/streams.jsonl').open():
        row=json.loads(line)
        if row['topic']!='telemetry' or row['received']>cutoff:continue
        data=row['data']
        if last_s is not None and last_s>100. and data['s']<5.:stage+=1
        last_s=data['s']
        telemetry.append((data['pose_stamp'],stage,data['s'],data['mode']))
    telemetry.sort();stamps=[row[0] for row in telemetry]
    frames=[];trace=args.run/'control_trace/control.jsonl'
    for line in trace.open():
        row=json.loads(line)
        if row['topic']!='lidar' or row['received_wall']>cutoff or 'file' not in row['data']:continue
        stamp=row['sensor_stamp'];i=bisect.bisect_left(stamps,stamp)
        ids=[j for j in (i-1,i) if 0<=j<len(stamps)]
        if not ids:continue
        matched=telemetry[min(ids,key=lambda j:abs(stamps[j]-stamp))]
        if abs(matched[0]-stamp)>.15 or matched[3]!='TRACK':continue
        with np.load(trace.parent/row['data']['file']) as data:points=data['points']
        points=points[np.all(np.isfinite(points),axis=1)&(np.sum(points**2,axis=1)>1e-8)]
        if not len(points):continue
        distances=np.linalg.norm(points+offset,axis=1)
        frames.append(dict(stamp=stamp,received_wall=row['received_wall'],stage=matched[1],
            s=matched[2],nearest=float(distances.min()),file=row['data']['file']))
    summary={}
    for stage in sorted(set(row['stage'] for row in frames)):
        selected=[row for row in frames if row['stage']==stage]
        distance=np.array([row['nearest'] for row in selected]);times=np.sort([row['stamp'] for row in selected])
        summary[str(stage)]=dict(frames=len(selected),minimum=float(distance.min()),
            below_1_25m=int(np.sum(distance<1.25)),below_1m=int(np.sum(distance<1.)),
            max_sample_gap=float(np.max(np.diff(times))) if len(times)>1 else None,
            closest=sorted(selected,key=lambda row:row['nearest'])[:10])
    output=dict(scope='All matched archived lidar frames during TRACK, before last valid HUD. Measured visible surfaces only; gaps, hidden volume and collisions are not certified.',
        cutoff_wall=cutoff,sensor_offset=offset.tolist(),by_stage=summary,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),frames=frames)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(dict(by_stage=summary),indent=2))


if __name__=='__main__':main()
