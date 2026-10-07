#!/usr/bin/env python3
"""Generate a reproducible continuous CV scene, replay it and score known truth.

This validates perception under known geometry/motion, not vehicle-course
completion. Includes two opposing crossing targets, a static target, and a
moving observer. Ground truth is analytic and remains in world NED.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from replay_dynamic_tracker import replay,SCRIPTS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--noise-std',type=float,default=.03)
    args=parser.parse_args()
    if not np.isfinite(args.noise_std) or args.noise_std<0:parser.error('noise-std must be nonnegative')
    if args.out.exists():parser.error('Use a new output directory')
    clouds=args.out/'clouds';clouds.mkdir(parents=True)
    actors=[dict(name='cross_left',position=np.array([8.,-3.,0.]),velocity=np.array([0.,1.,0.])),
            dict(name='cross_right',position=np.array([14.,3.,0.]),velocity=np.array([0.,-.8,0.])),
            dict(name='static',position=np.array([20.,.5,0.]),velocity=np.zeros(3))]
    shape=np.array([(x,y,z) for x in np.linspace(-.5,.5,6)
                    for y in np.linspace(-.5,.5,6) for z in (-.5,.5)])
    rng=np.random.default_rng(20261007)
    for i in range(61):
        t=i*.1;position=np.array([t,0.,0.])
        centers=[a['position']+a['velocity']*t for a in actors]
        visible=[(j,center) for j,center in enumerate(centers)
                 if not (j==0 and 18<=i<=20) and not (j==1 and 35<=i<=37)]
        points=np.concatenate([shape+center+rng.normal(0.,args.noise_std,shape.shape) for j,center in visible])
        reference=np.array([[t+d,0.,0.] for d in range(66)])
        name=clouds/('%04d'%i)
        np.savez_compressed(str(name)+'.npz',points=points,position=position,reference=reference)
        Path(str(name)+'.json').write_text(json.dumps(dict(s=t,cloud_stamp=100.+t,pose_stamp=100.+t,
            truth=[dict(name=a['name'],position=center.tolist(),velocity=a['velocity'].tolist())
                   for a,center in zip(actors,centers)]))+'\n')
    summary=replay(clouds,args.out/'replay',SCRIPTS.parent/'config/dynamic_tracker.yaml')
    rows=[json.loads(line) for line in (args.out/'replay/tracks.jsonl').read_text().splitlines()]
    errors={1.:[],2.:[]};velocity_error=[];ids={a['name']:set() for a in actors};confirmed=0
    for row in rows:
        t=row['stamp']-100.
        centers=[a['position']+a['velocity']*t for a in actors]
        for track in row['tracks']:
            if track['age']!=0.:continue
            distances=[np.linalg.norm(np.asarray(track['observed_position'])-center) for center in centers]
            index=int(np.argmin(distances))
            if distances[index]>max(.12,4.*args.noise_std):raise RuntimeError('Track no longer matches synthetic observation')
            actor=actors[index];ids[actor['name']].add((track['epoch'],track['track_id']))
            if t<1. or track['status']!='CONFIRMED':continue
            confirmed+=1;velocity_error.append(float(np.linalg.norm(track['velocity']-actor['velocity'])))
            for prediction in track['predictions']:
                dt=float(prediction['dt']);truth=actor['position']+actor['velocity']*(t+dt)
                errors[dt].append(float(np.linalg.norm(prediction['position']-truth)))
    result=dict(scope='Analytic constant-velocity synthetic scene only; not flight evidence.',
                point_noise_std=args.noise_std,occlusions='cross_left frames 18-20, cross_right frames 35-37',
                evaluated_confirmed_observations=confirmed,
                ids_per_actor={name:sorted(list(identity) for identity in identities) for name,identities in ids.items()},
                velocity_error_mps_p95=float(np.percentile(velocity_error,95)),
                prediction_error_m={str(dt):dict(samples=len(values),p95=float(np.percentile(values,95)),
                                                maximum=float(np.max(values))) for dt,values in errors.items()},
                frames=summary['frames'],tracking_ms=summary['tracking_ms'])
    (args.out/'truth_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
    if any(len(identity)!=1 for identity in ids.values()):raise SystemExit('Unexpected identity changes')
    if result['prediction_error_m']['2.0']['p95']>.15:raise SystemExit('Unexpected CV prediction error')


if __name__=='__main__':main()
