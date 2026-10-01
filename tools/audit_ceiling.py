#!/usr/bin/env python3
"""Audit archived ceiling reconstruction with an explicitly selected source copy.

This compares geometry at fixed archived poses. It does not simulate flight or
infer an official race result. Each invocation loads exactly one source copy.
"""
import argparse
import bisect
import hashlib
import json
import sys
from pathlib import Path

import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cloud-dir',type=Path,required=True)
    parser.add_argument('--scripts-dir',type=Path,required=True)
    parser.add_argument('--streams',type=Path)
    parser.add_argument('--before-received',type=float)
    parser.add_argument('--min-s',type=float,default=-float('inf'))
    parser.add_argument('--max-s',type=float,default=float('inf'))
    parser.add_argument('--limit',type=int,default=0,help='Evenly select this many snapshots; zero selects all')
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.before_received is not None and args.streams is None:
        parser.error('--before-received requires --streams')
    scripts=args.scripts_dir.resolve()
    sys.path.insert(0,str(scripts))
    from point_index import PointIndex
    from path_sampling import swept_samples

    telemetry=[]
    if args.streams:
        with args.streams.open() as stream:
            for line in stream:
                row=json.loads(line)
                if row['topic']=='telemetry':telemetry.append(row)
        telemetry.sort(key=lambda row:row['data']['pose_stamp'])
    stamps=[row['data']['pose_stamp'] for row in telemetry]
    selected=[];excluded=0
    for path in sorted(args.cloud_dir.glob('*.json')):
        saved=json.loads(path.read_text())
        if not args.min_s<=saved['s']<=args.max_s:continue
        row=None
        if telemetry:
            i=bisect.bisect_left(stamps,saved['pose_stamp'])
            neighbors=[j for j in (i-1,i) if 0<=j<len(stamps)]
            row=telemetry[min(neighbors,key=lambda j:abs(stamps[j]-saved['pose_stamp']))]
        if args.before_received is not None:
            if row is None or abs(row['data']['pose_stamp']-saved['pose_stamp'])>.2 or row['received']>args.before_received:
                excluded+=1;continue
        selected.append((path,saved,row))
    if args.limit>0 and len(selected)>args.limit:
        selected=[selected[i] for i in np.linspace(0,len(selected)-1,args.limit,dtype=int)]
    results=[]
    for path,saved,row in selected:
        with np.load(path.with_suffix('.npz')) as archive:
            points=archive['points'];position=archive['position'];reference=archive['reference']
        index=PointIndex(points,origin=position,road_height=5.)
        raw=PointIndex(points)
        route=reference[:31].copy();route[0]=position
        queries,_,_=swept_samples(route)
        raw_distance=raw.distance(queries);augmented=index.distance(queries)
        normal=None;roof_clearance=None;floor_clearance=None
        if index.roof is not None:
            origin,coeff,_,_=index.roof
            normal=float(np.sqrt(1.+np.sum(coeff[:2]**2)))
            height=float(position[2]-origin[2]-(position[:2]-origin[:2])@coeff[:2]-coeff[2])
            roof_clearance=height/normal;floor_clearance=(5.-height)/normal
        results.append(dict(sample=path.name,s=saved['s'],pose_stamp=saved['pose_stamp'],
                            received=None if row is None else row['received'],
                            roof_fitted=index.roof is not None,
                            roof_coeff=None if index.roof is None else index.roof[1].tolist(),
                            position_raw_clearance=float(raw.distance([position])[0]),
                            position_augmented_clearance=float(index.distance([position])[0]),
                            position_surface_clearance=(float(index.surface_distance([position])[0])
                                if np.isfinite(index.surface_distance([position])[0]) else None),
                            roof_signed_clearance=roof_clearance,floor_signed_clearance=floor_clearance,
                            nominal_30m_raw_min=float(np.min(raw_distance)),
                            nominal_30m_augmented_min=float(np.min(augmented)),
                            nominal_30m_blocked_queries=int(np.sum(augmented<1.25))))
    summary=dict(samples=len(results),excluded_by_received_cutoff=excluded,
                 fitted_roofs=sum(r['roof_fitted'] for r in results),
                 positions_below_buffer=sum(r['position_augmented_clearance']<1.25 for r in results),
                 raw_positions_below_buffer=sum(r['position_raw_clearance']<1.25 for r in results),
                 nominal_30m_blocked_snapshots=sum(r['nominal_30m_blocked_queries']>0 for r in results))
    report=dict(validation='Fixed archived geometry only; no closed-loop flight or official race acceptance',
                scripts_dir=str(scripts),cloud_dir=str(args.cloud_dir.resolve()),
                point_index_sha256=hashlib.sha256((scripts/'point_index.py').read_bytes()).hexdigest(),
                before_received=args.before_received,summary=summary,snapshots=results)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(summary))


if __name__=='__main__':main()
