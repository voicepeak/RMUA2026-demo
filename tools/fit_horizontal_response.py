#!/usr/bin/env python3
"""Identify XY response from actual published commands and independent poses.

Derivative fits diagnose model mismatch; they do not certify stopping bounds.
Command receipt is mapped to the sensor clock using adjacent pose receipts.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter


def samples(run):
    cutoff=json.loads((run/'result.json').read_text())['last_valid_hud']['wall_time']
    poses=[];commands=[]
    for line in (run/'control_trace/control.jsonl').open():
        row=json.loads(line)
        if row['received_wall']>cutoff:continue
        if row['topic']=='pose':poses.append(row)
        elif row['topic']=='command_trace' and row['data']['source_pose_stamp'] is not None:commands.append(row)
    wall=np.array([r['received_wall'] for r in poses]);stamp=np.array([r['sensor_stamp'] for r in poses])
    position=np.array([r['data']['position'][:2] for r in poses])
    quaternion=np.array([r['data']['quaternion'] for r in poses])
    x,y,z,w=quaternion.T
    yaw=np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    command_stamp=np.interp([r['received_wall'] for r in commands],wall,stamp)
    command_yaw=np.interp(command_stamp,stamp,yaw)
    body=np.array([r['data']['body_velocity'][:2] for r in commands])
    c=np.cos(command_yaw);s=np.sin(command_yaw)
    command=np.c_[c*body[:,0]-s*body[:,1],s*body[:,0]+c*body[:,1]]
    times=np.arange(command_stamp[0]+1.,min(command_stamp[-1],stamp[-1])-.5,.02)
    p=np.column_stack([np.interp(times,stamp,position[:,i]) for i in range(2)])
    v=savgol_filter(p,21,3,deriv=1,delta=.02,axis=0)
    a=savgol_filter(p,21,3,deriv=2,delta=.02,axis=0)
    j=savgol_filter(p,21,3,deriv=3,delta=.02,axis=0)
    return times,v,a,j,command_stamp,command


def features(data,delay,order):
    times,v,a,j,stamps,commands=data
    ids=np.clip(np.searchsorted(stamps,times-delay,side='right')-1,0,len(commands)-1)
    error=commands[ids]-v
    # Exclude clipping/contacts from this local linear identification.
    valid=(np.linalg.norm(a,axis=1)<5.5)&(np.linalg.norm(error,axis=1)<5.)
    target=(a if order==1 else j)[valid].ravel()
    columns=[error[valid].ravel()]
    if order==2:columns.append(-a[valid].ravel())
    return np.column_stack(columns),target


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--train',type=Path,required=True)
    parser.add_argument('--validate',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();train=samples(args.train);validation=samples(args.validate);results=[]
    for order in [1,2]:
        fits=[]
        for delay in np.arange(0.,.401,.02):
            x,y=features(train,delay,order);coefficient=np.linalg.lstsq(x,y,rcond=None)[0]
            if np.any(coefficient<=0):continue
            xv,yv=features(validation,delay,order)
            fits.append(dict(order=order,delay=float(delay),coefficients=coefficient.tolist(),
                train_rmse=float(np.sqrt(np.mean((x@coefficient-y)**2))),
                validation_rmse=float(np.sqrt(np.mean((xv@coefficient-yv)**2))),train_count=len(y),validation_count=len(yv)))
        results.append(min(fits,key=lambda r:r['train_rmse']))
    output=dict(scope='Local unsaturated derivative fit; different target derivatives across orders cannot be compared as the same RMSE. Not a safety envelope or flight acceptance.',train=str(args.train),validate=str(args.validate),fits=results)
    args.out.write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output,indent=2))


if __name__=='__main__':main()
