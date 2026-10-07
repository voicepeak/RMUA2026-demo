#!/usr/bin/env python3
"""Explicit oracle-known-space fixtures, NOT reconstructed LiDAR evidence.

Used to isolate search/response/time checking before replaying actual occupancy.
Vehicle predictions are controlled CV tracks; no flight performance claim.
"""
from dataclasses import replace
from pathlib import Path
import sys
import numpy as np
SCRIPTS=Path(__file__).resolve().parents[1]/'ros_ws/src/route_follower/scripts'
sys.path.insert(0,str(SCRIPTS))
from trajectory_types import PlanningSnapshot,readonly
from route_coordinates import RouteCoordinates
from dynamic_tracker import DynamicObstacle,TrackerConfig
from local_occupancy import OccupancyConfig,OccupancySnapshot,FREE
from velocity_response import VelocityResponse
from response_rollout import rollout_primitive


def obstacle(position,velocity=(0.,0.,0.),size=(1.,1.,1.),track_id=1,stamp=100.,config=None,status='CONFIRMED'):
    config=TrackerConfig(acceleration_std=.03,base_margin=0.,uncertainty_sigma=0.) if config is None else config
    return DynamicObstacle(track_id,0,stamp,stamp,stamp-1.,readonly(position),readonly(position),readonly(velocity),
        readonly([0.,0.,0.]),readonly(size),readonly(np.eye(6)*.001),0.,10,.9,status,False,True,
        readonly([],dtype=int),config)


def known_snapshot(obstacles=(),velocity=(0.,0.,0.),stamp=100.,vertical_limit=1.25,lateral_limit=2.25):
    model=VelocityResponse(native=False,lift_gain=.095,coupling_gains=(.075,.11,.13),
                           coupling_limited=True,xy_error_max=4.5)
    parameters=model.freeze_parameters()
    p=np.array([.1,0.,0.]);v=np.asarray(velocity,dtype=float)
    model.commit(v,v,stamp-.08)
    state=model.snapshot_state(p,v,stamp,parameters)
    stations=np.arange(-4.,31.)
    route=RouteCoordinates(stations,np.column_stack([stations,np.zeros((len(stations),2))]),lateral_limit,vertical_limit)
    config=OccupancyConfig(resolution=.5,forward=25.,backward=5.,lateral=4.,vertical_min=-3.,vertical_max=3.)
    shape=(61,17,13);states=np.full(shape,FREE,dtype=np.uint8)
    mapping=OccupancySnapshot(stamp,stamp,0,1,True,config,readonly([0.,0.,0.]),readonly([1.,0.]),
        readonly([-10,-8,-6],dtype=np.int64),readonly(states,dtype=np.uint8),readonly(states,dtype=np.uint8),
        readonly(np.full(shape,-1),dtype=np.int64))
    return PlanningSnapshot(0,1,stamp,state,route,mapping,tuple(obstacles)),parameters


def set_box(snapshot,low,high,state):
    mapping=snapshot.occupancy;values=mapping.static_states.copy()
    first=np.maximum(0,np.floor(np.asarray(low)/mapping.config.resolution).astype(int)-mapping.cell_origin)
    last=np.minimum(values.shape,np.floor(np.asarray(high)/mapping.config.resolution).astype(int)-mapping.cell_origin+1)
    values[tuple(slice(int(a),int(b)) for a,b in zip(first,last))]=state
    mapping=replace(mapping,states=readonly(values,dtype=np.uint8),static_states=readonly(values,dtype=np.uint8))
    return replace(snapshot,occupancy=mapping)


def cases():
    return {
        'A_static_vehicle':known_snapshot([obstacle([6.,0.,0.])]),
        'B_crossing_vehicle':known_snapshot([obstacle([5.,-2.,0.],[0.,1.5,0.])],velocity=(2.,0.,0.),vertical_limit=.75),
        'C_opposing_vehicles':known_snapshot([obstacle([5.,-2.,0.],[0.,1.5,0.]),
            obstacle([8.,3.,0.],[0.,-1.2,0.],track_id=2)],velocity=(2.,0.,0.),vertical_limit=.75),
        'D_temporary_block':known_snapshot([obstacle([3.,0.,0.],[0.,.8,0.],size=(1.,8.,2.))],vertical_limit=.75),
        'F_narrow_corridor':known_snapshot(velocity=(2.,0.,0.),lateral_limit=1.,vertical_limit=.75),
    }


def advance_oracle(snapshot,parameters,trajectory):
    """Execute ONLY the certified prefix in the middle empirical scenario.

    Refresh perfect observations explicitly; this oracle does not model real
    sensor occlusion, scheduling, publishing, bridge behavior or flight error.
    """
    if snapshot.reaction_delay:raise ValueError('Oracle stepping expects zero reaction delay')
    prefix=rollout_primitive(parameters,snapshot.response_state,trajectory.primitives[0],trajectory.profiles[0])
    state=prefix.end_state;truth=len(parameters.scenarios)//2
    duplicate=lambda a:np.tile(a[truth],(len(parameters.scenarios),1))
    state=replace(state,position=duplicate(state.position),velocity=duplicate(state.velocity),previous=duplicate(state.previous),
                  applied=duplicate(state.applied),last_control=np.full(len(parameters.scenarios),state.last_control[truth]),
                  next_control=np.full(len(parameters.scenarios),state.next_control[truth]),
                  effective=duplicate(state.applied),next_physics=state.stamp)
    stamp=state.stamp;obstacles=[]
    for old in snapshot.obstacles:
        p=readonly(old.position+(stamp-old.timestamp)*old.velocity)
        obstacles.append(replace(old,position=p,observed_position=p,timestamp=stamp,observed_stamp=stamp,age=0.))
    mapping=replace(snapshot.occupancy,stamp=stamp,evaluated_at=stamp,version=snapshot.occupancy.version+1)
    return replace(snapshot,pose_stamp=stamp,response_state=state,scene_version=mapping.version,
                   occupancy=mapping,obstacles=tuple(obstacles)),prefix
