#!/usr/bin/env python3
"""Bounded, read-only tracker diagnostics; ROS imports are optional.

Preview uses an explicit ENU frame: world NED (x,y,z) -> (y,x,-z).
This transform is for display only; tracking and prediction stay in NED.
"""
import numpy as np


def track_marker_specs(obstacles,horizons=(1.,2.),max_tracks=128):
    if max_tracks<1 or any(not np.isfinite(t) or t<0 for t in horizons):
        raise ValueError('Invalid marker horizons/budget')
    result=[]
    for obstacle in sorted(obstacles,key=lambda o:o.track_id)[:max_tracks]:
        point=lambda p:np.array([p[1],p[0],-p[2]]).tolist()
        scale=lambda p:np.maximum(.05,np.array([p[1],p[0],p[2]])).tolist()
        identity=dict(track_id=obstacle.track_id,epoch=obstacle.epoch,
                      status=obstacle.status,source_stamp=obstacle.timestamp)
        result.append(dict(identity,kind='box',dt=0.,position=point(obstacle.position),
                           size=scale(obstacle.bbox_size),color=[.15,.45,1.,.65]))
        result.append(dict(identity,kind='text',dt=0.,position=point(obstacle.position),
                           text='%d:%d %s v=(%.2f,%.2f,%.2f) age=%.2fs' %
                           (obstacle.epoch,obstacle.track_id,obstacle.status,*obstacle.velocity,obstacle.age),
                           color=[.3,.6,1.,1.]))
        for dt in horizons:
            prediction=obstacle.predict(dt)
            result.append(dict(identity,kind='box',dt=float(dt),position=point(prediction.position),
                               size=scale(prediction.bbox_size+2.*prediction.uncertainty),
                               color=[1.,.5,.05,.18],
                               text='track %d future +%.2fs' % (obstacle.track_id,dt)))
    return result


def build_track_markers(specs,stamp,frame='rmua_tracker_preview_enu',lifetime=.5):
    return _build_preview_markers(specs,stamp,frame,lifetime,'dynamic_tracker_preview')


def occupancy_marker_specs(snapshot,max_count=1024,show_unknown=False):
    from local_occupancy import FREE,OCCUPIED,UNKNOWN
    result=[]
    layers=[(FREE,'observed',[.15,1.,.3,.4]),(OCCUPIED,'static',[1.,.15,.1,.6])]
    if show_unknown:layers.append((UNKNOWN,'observed',[.6,.6,.6,.12]))
    for state,layer,color in layers:
        points=snapshot.voxel_centers(state,layer,max_count)
        enu=points[:,[1,0,2]].copy();enu[:,2]*=-1.
        result.append(dict(kind='voxels',positions=enu.tolist(),
                           size=[snapshot.config.resolution]*3,color=color,state=state,layer=layer))
    return result


def build_occupancy_markers(specs,stamp,frame='rmua_tracker_preview_enu',lifetime=.5):
    return _build_preview_markers(specs,stamp,frame,lifetime,'local_occupancy_preview')


def trajectory_marker_specs(plan,executed,backup,max_points=200):
    if max_points<2:raise ValueError('Trajectory marker budget must be at least two')
    result=[]
    for trace,name,color in ((plan,'search_suffix_preview',[.7,.2,1.,.65]),
                              (executed,'certified_command_and_stop',[1.,.85,.1,1.]),
                              (backup,'certified_backup',[.5,.5,.5,.8])):
        if trace is None:continue
        ids=np.unique(np.linspace(0,len(trace.times)-1,min(max_points,len(trace.times)),dtype=int))
        positions=trace.positions[ids,len(trace.end_state.position)//2][:,[1,0,2]].copy();positions[:,2]*=-1.
        result.append(dict(kind='line',positions=positions.tolist(),size=[.04,0.,0.],color=color,text=name))
    return result


def build_trajectory_markers(specs,stamp,frame='rmua_tracker_preview_enu',lifetime=.5):
    return _build_preview_markers(specs,stamp,frame,lifetime,'spacetime_trajectory_preview')


def _build_preview_markers(specs,stamp,frame,lifetime,namespace):
    """Construct ROS messages only when explicitly invoked in a ROS process."""
    import rospy
    from visualization_msgs.msg import Marker,MarkerArray
    from geometry_msgs.msg import Point
    result=MarkerArray()
    clear=Marker();clear.action=Marker.DELETEALL
    clear.header.frame_id=frame;clear.header.stamp=stamp
    result.markers.append(clear)
    for i,spec in enumerate(specs):
        marker=Marker();marker.header.frame_id=frame;marker.header.stamp=stamp
        marker.ns=namespace;marker.id=i
        marker.action=Marker.ADD
        marker.type=(Marker.TEXT_VIEW_FACING if spec['kind']=='text' else
                     Marker.LINE_STRIP if spec['kind']=='line' else
                     Marker.CUBE_LIST if spec['kind']=='voxels' else Marker.CUBE)
        if spec['kind'] in ('voxels','line'):
            marker.points=[Point(x=p[0],y=p[1],z=p[2]) for p in spec['positions']]
        else:marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=spec['position']
        marker.pose.orientation.w=1.
        marker.color.r,marker.color.g,marker.color.b,marker.color.a=spec['color']
        if spec['kind']=='text':
            marker.text=spec['text'];marker.scale.z=.25;marker.pose.position.z+=.75
        else:marker.scale.x,marker.scale.y,marker.scale.z=spec['size']
        marker.lifetime=rospy.Duration(lifetime)
        result.markers.append(marker)
    return result
