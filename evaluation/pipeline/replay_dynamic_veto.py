"""Probe candidate-dynamic veto against retained LiDAR associations."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

from evaluation.pipeline.replay_gate_tracks import interpolate_pose
from interfaces.contracts import Metadata, Trajectory, TrajectorySample
from planning.trajectory_validator.dynamic_candidates import DynamicBounds, DynamicScene, check_dynamic_candidates
from planning.trajectory_validator.static_sweep import SweepBounds
from world_model.dynamic_tracks.tracker import ObjectTrack


def replay(source):
    source=Path(source);data=json.loads(source.read_text());run=Path(data['source'])
    log=run/('pwm_telemetry.jsonl' if (run/'pwm_telemetry.jsonl').exists() else 'vel_telemetry.jsonl')
    if hashlib.sha256(log.read_bytes()).hexdigest()!=data['state_input_sha256']:
        raise ValueError('recorded state input changed')
    states=[json.loads(s) for s in log.read_text().splitlines()]
    body=SweepBounds((.2,.2,.1),.05,0.)
    # Explicit assumptions for a diagnostic model, not calibrated bounds.
    bounds=DynamicBounds(.2,2.,4.,20.,max_horizon_s=.3)
    rows=[]
    for frame in data['results']:
        if frame['status']!='PROCESSED':continue
        pose,reason=interpolate_pose(states,frame['stamp'])
        if pose is None:
            rows.append({'file':frame['file'],'reason':reason});continue
        now=frame['now']
        sample=lambda t:TrajectorySample(t,pose.position_ned,(0.,0.,0.),(0.,0.,0.),0.)
        path=Trajectory(Metadata(now,now+.2,0,'stationary_geometry_probe'),(sample(now),sample(now+.2)))
        tracks=[]
        for raw in frame['tracks']:
            allowed={name:raw[name] for name in ('track_id','epoch','center_ned','half_extents_m','velocity_ned_m_s','observations','stamp','valid_until')}
            for key in ('center_ned','half_extents_m','velocity_ned_m_s'):allowed[key]=tuple(allowed[key])
            tracks.append(ObjectTrack(**allowed))
        scene=DynamicScene(Metadata(frame['stamp'],frame['stamp']+.2,0,'recorded_scan'),tuple(tracks))
        result=check_dynamic_candidates(path,scene,now,0,body,bounds)
        rows.append({'file':frame['file'],'track_count':len(tracks),'reason':result.reason,
                     'allowed':result.allowed,'safety_certified':result.safety_certified,
                     'track_id':result.track_id,'pair_tests':result.pair_tests})
    return {'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'state_sha256':data['state_input_sha256'],'role':'stationary_synthetic_geometry_probe',
            'assumptions':asdict(bounds),'body_bounds':asdict(body),
            'source_time_is_idealized_not_processing_completion':True,'control_eligible':False,
            'counts':dict(Counter(r['reason'] for r in rows)),'results':rows}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source');p.add_argument('output');a=p.parse_args()
    result=replay(a.source)
    with Path(a.output).open('x') as out:json.dump(result,out,indent=2,allow_nan=False);out.write('\n')
    print(json.dumps({'output':a.output,'counts':result['counts']}))


if __name__=='__main__':main()
