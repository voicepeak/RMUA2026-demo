"""Offline LiDAR cluster and dynamic association replay from recorded flight."""
import argparse
from dataclasses import asdict
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from adapters.ros_noetic.lidar_cloud import LidarCloud
from evaluation.pipeline.replay_gate_tracks import interpolate_pose
from interfaces.contracts import Metadata
from perception.object_motion.lidar_clusters import cluster_endpoints
from perception.stereo_depth.geometry import load_nominal_rig
from world_model.dynamic_tracks.tracker import DynamicTracker
from world_model.occupancy.rolling_endpoints import world_endpoints


def replay(source,settings,limit=24):
    source=Path(source);directory=source/'scene_trace';index_path=directory/'index.json'
    index=json.loads(index_path.read_text());rig=load_nominal_rig(settings)
    manifest=json.loads((source/'manifest.json').read_text())
    if index['fault'] is not None or manifest['simulator_preflight']['settings_sha256']!=rig.settings_sha256:
        raise ValueError('capture or simulator settings mismatch')
    log=source/('pwm_telemetry.jsonl' if (source/'pwm_telemetry.jsonl').exists() else 'vel_telemetry.jsonl')
    states=[json.loads(s) for s in log.read_text().splitlines()]
    if any(a['raw']['stamp']>=b['raw']['stamp'] for a,b in zip(states,states[1:])):
        raise ValueError('nonmonotonic state recording')
    scans=sorted((r for r in index['frames'] if r['sensor']=='lidar'),key=lambda r:r['stamp'])
    tracker=DynamicTracker();rows=[];processed=0
    for scan in scans:
        if processed>=limit:break
        stamp=scan['stamp']; pose,reason=interpolate_pose(states,stamp)
        if pose is None:
            rows.append({'file':scan['file'],'status':reason});continue
        path=directory/scan['file']
        if path.parent!=directory or path.suffix!='.npz':raise ValueError('invalid point file')
        with np.load(path,allow_pickle=False) as payload:points=np.asarray(payload['points'],dtype=float)
        cloud=LidarCloud(Metadata(stamp,stamp+.2,0,'recorded_lidar','lidar'),points,len(points),0)
        now=pose.bracket_stamps[1];started=time.monotonic()
        try:
            world=world_endpoints(cloud,pose,rig.lidar_pose);clusters=cluster_endpoints(world,now,0)
            association=tracker.ingest(clusters,now,0);tracks=tracker.snapshot(now,0)
            rows.append({'file':scan['file'],'stamp':stamp,'status':'PROCESSED','input_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                         'now':now,'duration_s':time.monotonic()-started,'clusters':len(clusters.clusters),
                         'occupied_voxels':clusters.occupied_voxels,'rejected_components':clusters.rejected_components,
                         'association':association,'tracks':[asdict(t) for t in tracks]})
        except (ValueError,ArithmeticError) as error:
            rows.append({'file':scan['file'],'stamp':stamp,'status':'REJECTED','reason':str(error)})
        processed+=1
    return {'source':str(source),'settings_sha256':rig.settings_sha256,
            'scene_index_sha256':hashlib.sha256(index_path.read_bytes()).hexdigest(),
            'state_input_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),'statuses':dict(Counter(x['status'] for x in rows)),
            'uses_truth':False,'now_is_earliest_bracket_not_processing_completion':True,
            'control_eligible':False,'results':rows}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source');p.add_argument('output')
    p.add_argument('--settings',default='configs/simulator_settings.json');p.add_argument('--limit',type=int,default=24)
    args=p.parse_args()
    if not 1<=args.limit<=100:p.error('limit must be 1..100')
    result=replay(args.source,args.settings,args.limit)
    with Path(args.output).open('x') as out:json.dump(result,out,indent=2,allow_nan=False);out.write('\n')
    print(json.dumps({'output':args.output,'statuses':result['statuses']}))


if __name__ == '__main__':main()
