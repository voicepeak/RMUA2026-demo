#!/usr/bin/env python3
"""Lossless local planning-snapshot archive; never infer missing command history."""
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np
from trajectory_types import PlanningSnapshot,ResponseState,readonly
from response_rollout import ResponseParameters
from route_coordinates import RouteCoordinates
from local_occupancy import OccupancyConfig,OccupancySnapshot
from dynamic_tracker import DynamicObstacle,TrackerConfig


def write_snapshot(snapshot,parameters,path,provenance):
    path=Path(path)
    if path.exists() or path.with_suffix('.npz').exists():raise ValueError('Snapshot path already exists')
    if snapshot.response_state.model_key!=parameters.model_key:raise ValueError('Snapshot/model mismatch')
    path.parent.mkdir(parents=True,exist_ok=True)
    state=snapshot.response_state;mapping=snapshot.occupancy;route=snapshot.route
    arrays={key:getattr(state,key) for key in ('position','velocity','previous','applied','last_control','next_control','effective')}
    arrays.update(stations=route.stations,route_points=route.points,floor_limits=route.floor_limits,
                  states=mapping.states,static_states=mapping.static_states,dynamic_owners=mapping.dynamic_owners,
                  map_center=mapping.center,map_forward=mapping.forward,cell_origin=mapping.cell_origin)
    response=asdict(parameters)
    if np.isposinf(response['xy_error_max']):response['xy_error_max']=None
    tracks=[]
    for obstacle in snapshot.obstacles:
        values={k:v.tolist() if isinstance(v,np.ndarray) else asdict(v) if isinstance(v,TrackerConfig) else v for k,v in vars(obstacle).items()}
        tracks.append(values)
    meta=dict(format='rmua_spacetime_snapshot_v2',provenance=provenance,epoch=snapshot.epoch,scene_version=snapshot.scene_version,
        pose_stamp=snapshot.pose_stamp,reaction_delay=snapshot.reaction_delay,evaluation_stamp=snapshot.evaluation_stamp,response_parameters=response,
        sensor_origin=None if snapshot.sensor_origin is None else snapshot.sensor_origin.tolist(),
        model_key=state.model_key,has_applied=state.has_applied,next_physics=state.next_physics,
        route=dict(lateral_limit=route.lateral_limit,vertical_limit=route.vertical_limit),
        occupancy=dict(stamp=mapping.stamp,evaluated_at=mapping.evaluated_at,epoch=mapping.epoch,version=mapping.version,
                       valid=mapping.valid,config=asdict(mapping.config)),obstacles=tracks)
    serialized=json.dumps(meta,indent=2,allow_nan=False)+'\n'
    np.savez_compressed(path.with_suffix('.npz'),**arrays);path.write_text(serialized)


def read_snapshot(path):
    path=Path(path);meta=json.loads(path.read_text())
    if meta.get('format')!='rmua_spacetime_snapshot_v2':raise ValueError('Not a frozen planning snapshot; response history is required')
    configuration=dict(meta['response_parameters'])
    if configuration['xy_error_max'] is None:configuration['xy_error_max']=float('inf')
    parameters=ResponseParameters(**configuration)
    if parameters.model_key!=meta['model_key']:raise ValueError('Frozen model hash mismatch')
    with np.load(path.with_suffix('.npz'),allow_pickle=False) as data:
        state=ResponseState(meta['pose_stamp'],meta['model_key'],*[data[key] for key in
               ('position','velocity','previous','applied','last_control','next_control')],meta['has_applied'],data['effective'],meta['next_physics'])
        if len(state.position)!=len(parameters.scenarios):raise ValueError('Frozen response scenario count mismatch')
        route=RouteCoordinates(data['stations'],data['route_points'],floor_limits=data['floor_limits'],**meta['route'])
        config=OccupancyConfig(**meta['occupancy']['config']);shape=data['states'].shape
        if (len(shape)!=3 or np.prod(shape)>config.max_voxels or any(n<1 for n in shape) or
                data['static_states'].shape!=shape or data['dynamic_owners'].shape!=shape or
                not np.issubdtype(data['dynamic_owners'].dtype,np.signedinteger) or
                np.any(data['dynamic_owners']<-2) or np.any(data['dynamic_owners']==0) or
                not np.all(np.isin(data['states'],[0,1,2])) or not np.all(np.isin(data['static_states'],[0,1,2])) or
                data['cell_origin'].shape!=(3,) or not np.issubdtype(data['cell_origin'].dtype,np.integer) or
                data['map_center'].shape!=(3,) or data['map_forward'].shape!=(2,) or
                not np.all(np.isfinite(np.r_[data['map_center'],data['map_forward']])) or
                abs(np.linalg.norm(data['map_forward'])-1.)>1e-8):raise ValueError('Invalid frozen occupancy arrays')
        mapping=OccupancySnapshot(**{k:v for k,v in meta['occupancy'].items() if k!='config'},config=config,
             center=readonly(data['map_center']),forward=readonly(data['map_forward']),cell_origin=readonly(data['cell_origin'],dtype=np.int64),
             states=readonly(data['states'],dtype=np.uint8),static_states=readonly(data['static_states'],dtype=np.uint8),
             dynamic_owners=readonly(data['dynamic_owners'],dtype=np.int64))
    obstacles=[]
    for values in meta['obstacles']:
        values=dict(values);values['config']=TrackerConfig(**values['config'])
        for key in ('position','observed_position','velocity','acceleration','bbox_size','covariance','point_indices'):
            values[key]=readonly(values[key],dtype=int if key=='point_indices' else float)
        obstacles.append(DynamicObstacle(**values))
    snapshot=PlanningSnapshot(meta['epoch'],meta['scene_version'],meta['pose_stamp'],state,route,mapping,tuple(obstacles),meta['reaction_delay'],meta.get('evaluation_stamp'),meta.get('sensor_origin'))
    return snapshot,parameters,meta['provenance']
