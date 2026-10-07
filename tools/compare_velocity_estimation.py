#!/usr/bin/env python3
"""Compare archived controller velocity with a causal terminal quadratic fit.

Reference is an offline smoothed position derivative, not a velocity sensor or
flight acceptance. This tool does not alter the active controller.
"""
import argparse
import bisect
import json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    cutoff=json.loads((args.run/'result.json').read_text())['last_valid_hud']['wall_time']
    poses=[]
    for line in (args.run/'control_trace/control.jsonl').open():
        row=json.loads(line)
        if row['topic']=='pose' and row['received_wall']<=cutoff:poses.append(row)
    pt=np.array([r['sensor_stamp'] for r in poses]);position=np.array([r['data']['position'] for r in poses])
    unique=np.r_[True,np.diff(pt)>0.];pt=pt[unique];position=position[unique]
    times=np.arange(pt[0],pt[-1],.01)
    p=np.column_stack([np.interp(times,pt,position[:,i]) for i in range(3)])
    reference=savgol_filter(p,11,3,deriv=1,delta=.01,axis=0)
    groups={};stage=0;last_s=None
    for line in (args.run/'flight/streams.jsonl').open():
        row=json.loads(line)
        if row['topic']!='telemetry' or row['received']>cutoff:continue
        d=row['data'];stamp=d['pose_stamp']
        if last_s is not None and last_s>100. and d['s']<5.:stage+=1
        last_s=d['s']
        if d['mode']!='TRACK' or stamp<pt[0]+.2 or stamp>pt[-1]-.1:continue
        high=bisect.bisect_right(pt,stamp);low=bisect.bisect_left(pt,stamp-.18)
        if high-low<6:continue
        x=(pt[low:high]-pt[high-1])/.18
        design=np.column_stack([np.ones(len(x)),x,x*x])
        coefficient=np.linalg.lstsq(design,position[low:high],rcond=None)[0]
        estimated=coefficient[1]/.18
        expected=np.array([np.interp(stamp,times,reference[:,i]) for i in range(3)])
        groups.setdefault(str(stage),[]).append((np.asarray(d['measured_velocity_world'])-expected,estimated-expected))
    result={}
    for stage,values in groups.items():
        values=np.array(values)
        result[stage]={'samples':len(values),'methods':{}}
        for i,name in enumerate(['controller_filtered','causal_quadratic_0_18s']):
            error=values[:,i,:];norm=np.linalg.norm(error,axis=1)
            result[stage]['methods'][name]=dict(rmse_xyz=np.sqrt(np.mean(error**2,axis=0)).tolist(),
                norm_p50=float(np.percentile(norm,50)),norm_p95=float(np.percentile(norm,95)),norm_max=float(norm.max()))
    output=dict(scope=__doc__,run=str(args.run),by_stage=result)
    args.out.write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output,indent=2))


if __name__=='__main__':main()
