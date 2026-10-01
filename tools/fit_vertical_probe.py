#!/usr/bin/env python3
"""Fit pure vertical command response using subscribed commands and poses.

This identifies a first-order approximation in a bounded calibration only.
Command wall time is mapped through the pose receive clock, rather than
subtracting wall time from the simulator's drifting sensor timestamps.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.signal import lfilter, savgol_filter


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();run=args.run
    phases=[json.loads(line) for line in (run/'phases.jsonl').open()]
    vertical=[r for r in phases if r['phase'].startswith('vertical_')]
    if not vertical:raise RuntimeError('No pure vertical phases')
    records=[json.loads(line) for line in (run/'trace/control.jsonl').open()]
    poses=[r for r in records if r['topic']=='pose' and abs(sum(q*q for q in r['data']['quaternion'])-1.)<.01]
    commands=[r for r in records if r['topic']=='command']
    pt=np.array([r['sensor_stamp'] for r in poses])
    wall=np.array([r['received_wall'] for r in poses])
    height=np.array([r['data']['position'][2] for r in poses])
    ct=np.interp([r['received_wall'] for r in commands],wall,pt)
    cz=-np.array([r['data']['body_velocity'][2] for r in commands])
    start=vertical[0]['pose_stamp']
    end=next((r['pose_stamp'] for r in phases if r['phase']=='complete'),pt[-1])
    dt=.02;t=np.arange(start,end,dt);z=np.interp(t,pt,height)
    # Derivative uses a 0.18 s symmetric smoothing window. Its effect belongs
    # to the approximation error and is not treated as controller delay.
    v=savgol_filter(z,9,2,deriv=1,delta=dt)
    mask=np.arange(len(t))>15
    best=None;table=[]
    for delay in np.arange(0.,.221,.02):
        u=cz[np.clip(np.searchsorted(ct,t-delay,side='right')-1,0,len(cz)-1)]
        for tau in np.arange(.06,2.001,.02):
            decay=np.exp(-dt/tau)
            filtered=lfilter([1.-decay],[1.,-decay],u,zi=[v[0]*decay])[0]
            design=np.column_stack([filtered[mask],np.ones(mask.sum())])
            scale,bias=np.linalg.lstsq(design,v[mask],rcond=None)[0]
            prediction=filtered*scale+bias
            rmse=float(np.sqrt(np.mean((prediction[mask]-v[mask])**2)))
            row=dict(tau=float(tau),delay=float(delay),scale=float(scale),bias=float(bias),velocity_rmse=rmse)
            if best is None or rmse<best['velocity_rmse']:best=row
            if delay==0. and abs(tau-.16)<1e-6:table.append(row)
    result=dict(scope='Pure vertical, first-order approximation; not a complete coupled model or race acceptance.',
        run=str(run),complete=(run/'complete.json').exists(),samples=len(t),best=best,
        previous_tau_comparison=table,maximum_vertical_speed=float(np.max(abs(v))),
        trace_sha256=hashlib.sha256((run/'trace/control.jsonl').read_bytes()).hexdigest())
    (run/'vertical_fit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
