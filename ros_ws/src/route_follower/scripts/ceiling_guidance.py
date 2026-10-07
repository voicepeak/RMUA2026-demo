#!/usr/bin/env python3
"""Local height guidance inside an already supported measured ceiling hull.

This supplies road height evidence when gates disappear. It creates no gates
and does not certify obstacle clearance; final execution checks remain active.
The five-metre road-height prior is the same one used by ExecutionGuard.
"""
import numpy as np


def ceiling_guidance(index,cloud_stamp,pose_stamp,station,xy,base_center):
    if (index is None or index.roof is None or index.road_height is None or
            cloud_stamp is None or not 0.<=pose_stamp-cloud_stamp<.3):return None
    stations=station+np.arange(-3.,24.01,.5)
    coordinates=np.array([xy(s)[:2] for s in stations])
    tangent=np.gradient(coordinates,axis=0)
    tangent/=np.maximum(1e-9,np.linalg.norm(tangent,axis=1))[:,None]
    side=np.column_stack((-tangent[:,1],tangent[:,0]))
    # A narrow roof return is not enough: require supported ceiling over
    # a one-metre transverse strip, including the road reference itself.
    queries=np.array([np.column_stack((coordinates+offset*side,np.zeros(len(stations))))
                      for offset in (-.5,0.,.5)])
    ceilings=index.floor_limit(queries.reshape(-1,3),0.).reshape(3,-1)-index.road_height
    supported=np.all(np.isfinite(ceilings),axis=0)
    here=6
    starts=np.flatnonzero(supported[here:here+7])+here
    if not len(starts):return None
    start=int(starts[0])
    # The upward-looking lidar hull may taper near the aircraft. Its center
    # must still be observed through that short connector; the forward strip
    # must have full width. This never fills an unobserved gap in the hull.
    if not np.all(np.isfinite(ceilings[1,here:start+1])):return None
    end=start
    while end+1<len(stations) and supported[end+1]:end+=1
    if stations[end]-stations[start]<4.:return None
    gap=base_center(station)-ceilings[1,here]
    if not 1.4<=gap<=3.8:return None
    begin=here
    while begin>0 and supported[begin-1]:begin-=1
    height=ceilings[1,begin:end+1]+.5*index.road_height
    return stations[begin:end+1],height
