#!/usr/bin/env python3
"""Replay archived cloud snapshots and audit stop reasons without ROS/UE4."""
import argparse
import bisect
import json
import sys
import time
from pathlib import Path
import numpy as np
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--leg',default='leg_3_5')
    parser.add_argument('--min-s',type=float,default=100.)
    parser.add_argument('--max-s',type=float,default=132.5)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--scripts-dir',type=Path)
    args=parser.parse_args()
    scripts=Path(__file__).resolve().parents[1]/'ros_ws/src/route_follower/scripts'
    sys.path.insert(0,str(scripts))
    if args.scripts_dir:sys.path.insert(0,str(args.scripts_dir.resolve()))
    from predictive_avoidance import PredictiveAvoidance
    from reference_planner import RouteGeometry
    from execution_guard import ExecutionGuard
    leg=args.run/'mission'/args.leg
    route=RouteGeometry(yaml.safe_load((leg/'route.yaml').read_text())['routes']['race_leg'])
    gates=yaml.safe_load((leg/'gates.yaml').read_text())['gates']
    events=[json.loads(line) for line in (args.run/'mission/mission.jsonl').open()]
    start=next(r['stamp'] for r in events if r.get('kind')=='CONTROLLER_STARTED' and r.get('leg')==[3,5])
    telemetry=[]
    for line in (args.run/'flight/streams.jsonl').open():
        row=json.loads(line)
        if row['topic']=='telemetry' and row['received']>=start:telemetry.append(row)
    # This is an observed progress window, NOT an inferred official stage
    # timeout. HUD State was not recorded, and clocks diverge significantly.
    window=[]
    for row in telemetry:
        if row['data']['s']>=args.max_s:break
        window.append(row)
    stamps=[r['data']['pose_stamp'] for r in telemetry]
    tasks={r['data']['clearance'].get('planned_stamp'):r['data']['clearance'] for r in window}
    durations=[c['planning_ms'] for c in tasks.values() if c.get('planned_stamp') is not None]
    audit=dict(window_basis='observed route progress; official HUD validity unavailable',
               samples=len(window),distinct_tasks=len(durations),
               planning_ms_percentiles=dict(zip(('p50','p95','max'),np.percentile(durations,[50,95,100]).tolist())) if durations else {})
    audit['zero_command_samples']=sum(r['data']['speed']<=.05 for r in window)
    audit['feasible_stale_zero_samples']=sum(r['data']['speed']<=.05 and bool(r['data']['clearance'].get('feasible'))
            and r['data']['clearance'].get('stale',False) for r in window)
    results=[]
    for meta in sorted((leg/'clouds').glob('*.json')):
        saved=json.loads(meta.read_text());s=saved['s']
        if not args.min_s<=s<args.max_s:continue
        data=np.load(meta.with_suffix('.npz'));points=data['points'];position=data['position'];reference=data['reference']
        ss=s+np.arange(len(reference))
        xy=lambda station:tuple(np.interp(station,ss,reference[:,axis]) for axis in (0,1))
        center=lambda station:float(np.interp(station,ss,reference[:,2]))
        stamp=saved['pose_stamp']
        idx=min(len(telemetry)-1,bisect.bisect_left(stamps,stamp))
        row=telemetry[idx]['data']
        velocity=np.array([*row['velocity_world'], -row['vz_command']])
        cars=[dict(c,world=np.array(c['world'])) for c in saved['cars']]
        planner=PredictiveAvoidance(margin=1.15,braking=8.)
        started=time.monotonic()
        info=planner.evaluate(s,position,velocity,xy,center,route,points,cars,stamp,max(.5,row['speed']),gates)
        elapsed=1000.*(time.monotonic()-started)
        guard=ExecutionGuard()
        verified=guard.evaluate(s,position,velocity,xy,center,planner.plan,points,
                                stamp-(row.get('lidar_age') or 0.),stamp)
        results.append(dict(sample=meta.name,s=s,points=len(points),planning_ms=elapsed,
                            planner=info,guard=verified,
                            velocity_basis='nearest telemetry XY command and vertical command; measured 3D velocity not archived',
                            terminal_offset=planner.plan.offset(planner.plan.end).tolist() if planner.plan is not None else None,
                            terminal_slope=planner.plan.slope(planner.plan.end).tolist() if planner.plan is not None else None))
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(dict(audit=audit,replays=results),ensure_ascii=False,indent=2))
    print(json.dumps(dict(audit=audit,replays=len(results),feasible=sum(r['planner'].get('feasible',False) for r in results),
                         recertified=sum(r['guard']['execution_verified'] for r in results),
                         planning_ms=[round(r['planning_ms'],2) for r in results],
                         validation_ms=[round(r['guard'].get('validation_ms',0.),2) for r in results]),ensure_ascii=False))


if __name__=='__main__':main()
