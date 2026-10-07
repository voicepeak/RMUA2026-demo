#!/usr/bin/env python3
"""Simulate nearest-hit LiDAR rays in a bounded scene and replay perception.

Enclosure: front wall x=12, side walls y=+-3.5, floor/ceiling z=+-2.5.
A visible vehicle box crosses laterally in front of the moving observer.
This is geometry/ray evidence validation, not an AirSim flight experiment.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from replay_local_occupancy import replay


def sample_scene(origin,vehicle_center,directions):
    inverse=np.zeros_like(directions)
    np.divide(1.,directions,out=inverse,where=abs(directions)>1e-12)
    ranges=np.full(len(directions),np.inf)
    for axis,plane in ((0,12.),(1,-3.5),(1,3.5),(2,-2.5),(2,2.5)):
        candidate=(plane-origin[axis])*inverse[:,axis]
        candidate[(abs(directions[:,axis])<=1e-12)|(candidate<=0.)]=np.inf
        ranges=np.minimum(ranges,candidate)
    low=vehicle_center-np.array([.5,.5,.5]);high=vehicle_center+np.array([.5,.5,.5])
    a=(low-origin)*inverse;b=(high-origin)*inverse
    near=np.minimum(a,b);far=np.maximum(a,b)
    parallel=abs(directions)<=1e-12
    near[parallel]=-np.inf;far[parallel]=np.inf
    outside=np.any(parallel&((origin<low)|(origin>high)),axis=1)
    enter=near.max(axis=1);leave=far.min(axis=1)
    hit=(enter>=0.)&(leave>=enter)&~outside
    ranges[hit]=np.minimum(ranges[hit],enter[hit])
    if not np.all(np.isfinite(ranges)):raise ValueError('Unbounded synthetic sensor rays')
    return origin+directions*ranges[:,None]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--frames',type=int,default=51)
    args=parser.parse_args()
    if args.frames<3 or args.frames>1000:parser.error('frames must be 3..1000')
    if args.out.exists():parser.error('Use a new output directory')
    clouds=args.out/'clouds';clouds.mkdir(parents=True)
    azimuth,elevation=np.meshgrid(np.deg2rad(np.linspace(-88.,88.,177)),
                                  np.deg2rad(np.linspace(-35.,35.,31)),indexing='ij')
    directions=np.column_stack((np.cos(elevation.ravel())*np.cos(azimuth.ravel()),
                                np.cos(elevation.ravel())*np.sin(azimuth.ravel()),
                                np.sin(elevation.ravel())))
    for i in range(args.frames):
        t=i*.1;position=np.array([.1+.2*t,.1,.1]);origin=position+[0.,0.,-.05]
        vehicle=np.array([7.,-1.5+.6*t,0.])
        points=sample_scene(origin,vehicle,directions)
        reference=np.array([[position[0]+d,.1,.1] for d in range(66)])
        path=clouds/('%04d'%i)
        np.savez_compressed(str(path)+'.npz',points=points,position=position,reference=reference,
                            sensor_origin=origin,cloud_stamp=100.+t,lidar_epoch=0)
        Path(str(path)+'.json').write_text(json.dumps(dict(s=position[0],cloud_stamp=100.+t,
            pose_stamp=100.+t,sensor_origin=origin.tolist(),lidar_epoch=0,
            truth=dict(vehicle_center=vehicle.tolist(),vehicle_velocity=[0.,.6,0.])))+'\n')
    summary=replay(clouds,args.out/'replay')
    rows=[json.loads(line) for line in (args.out/'replay/occupancy.jsonl').read_text().splitlines()]
    result=dict(scope='Synthetic nearest-hit ray scene; no flight or real vehicle performance claim.',
                frames=args.frames,returns_per_frame=len(directions),
                map_valid_frames=sum(r['map']['valid'] for r in rows),
                dynamic_frames=summary['dynamic_frames'],mapping_ms=summary['mapping_ms'],
                max_voxels=summary['max_voxels'],rays_skipped=summary['rays_skipped'],
                sample_static_voxels=rows[-1]['map']['static_occupied'],
                sample_dynamic_voxels=rows[-1]['map']['dynamic_voxels'],
                sample_free_voxels=rows[-1]['map']['free'],sample_unknown_voxels=rows[-1]['map']['unknown'])
    (args.out/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
    if result['map_valid_frames']!=args.frames:raise SystemExit('Invalid synthetic exposure frame')
    if args.frames>=20 and not result['dynamic_frames']:raise SystemExit('No credible dynamic return ownership')


if __name__=='__main__':main()
