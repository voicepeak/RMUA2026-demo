"""Bounded LiDAR point clusters; candidates only, not object certificates."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from interfaces.contracts import Metadata


@dataclass(frozen=True)
class Cluster:
    center_ned: tuple
    half_extents_m: tuple
    points: int


@dataclass(frozen=True)
class ClusterFrame:
    metadata: Metadata
    clusters: tuple
    occupied_voxels: int
    rejected_components: int
    control_eligible: bool = False


def cluster_endpoints(observation, now, epoch, voxel_m=.5, min_points=8, max_voxels=50000, max_clusters=128, center_mode="bbox"):
    """Connected voxel clusters, preserving source lease and unknown space.

    Large connected components and sparse points are omitted from the object
    candidate list; they remain obstacles in the separate occupancy model.
    No candidate is a complete object boundary, dynamic class, or free space.
    """
    if (not observation.metadata.current(now,epoch) or observation.metadata.frame_id!='world_ned'
            or observation.pose_bracket_stamps[1]>now or not math.isfinite(voxel_m)
            or not .1<=voxel_m<=2. or isinstance(min_points,bool) or not isinstance(min_points,int) or min_points<3
            or isinstance(max_voxels,bool) or not isinstance(max_voxels,int) or not 1<=max_voxels<=100000
            or isinstance(max_clusters,bool) or not isinstance(max_clusters,int) or not 1<=max_clusters<=256
            or center_mode not in ('bbox','centroid')):
        raise ValueError('invalid cluster input or limits')
    points=observation.world_points
    with np.errstate(over='ignore',invalid='ignore'):
        coords=points/voxel_m
    if not np.isfinite(coords).all() or np.any(np.abs(coords)>2**45):raise ValueError('cluster coordinates exceed range')
    if not len(coords):return ClusterFrame(observation.metadata,(),0,0)
    keys,inverse,counts=np.unique(np.floor(coords).astype(np.int64),axis=0,return_inverse=True,return_counts=True)
    sums=np.column_stack([np.bincount(inverse,weights=points[:,axis],minlength=len(keys)) for axis in range(3)]) if center_mode=='centroid' else None
    if len(keys)>max_voxels:raise ValueError('cluster voxel budget exceeded')
    # Every selected cluster encloses its contributing occupied voxels, rather
    # than only centroids, and is therefore no smaller than the sampled points.
    mapping={tuple(int(v) for v in key):i for i,key in enumerate(keys)}
    unvisited=set(mapping);components=[];rejected=0
    while unvisited:
        first=unvisited.pop();queue=deque([first]); members=[];total=0;indices=[]
        while queue:
            key=queue.popleft();idx=mapping[key];members.append(key);indices.append(idx);total+=int(counts[idx])
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    for dz in (-1,0,1):
                        if dx==dy==dz==0:continue
                        neighbor=(key[0]+dx,key[1]+dy,key[2]+dz)
                        if neighbor in unvisited:unvisited.remove(neighbor);queue.append(neighbor)
        values=np.asarray(members,dtype=float);lower=values.min(axis=0)*voxel_m;upper=(values.max(axis=0)+1)*voxel_m
        extents=(upper-lower)/2
        if total<min_points or np.any(extents*2>np.array((12.,12.,8.))):
            rejected+=1;continue
        center=(lower+upper)/2 if center_mode=='bbox' else sums[indices].sum(axis=0)/total
        # Ensure the box still encloses every selected voxel when its center
        # moves from the midpoint to the measured centroid.
        extents=np.maximum(center-lower,upper-center)
        components.append(Cluster(tuple(center),tuple(extents),total))
        if len(components)>max_clusters:raise ValueError('cluster count budget exceeded')
    components.sort(key=lambda c:(-c.points,c.center_ned))
    return ClusterFrame(Metadata(observation.metadata.stamp,observation.metadata.valid_until,epoch,
                                 'lidar_voxel_object_candidates','world_ned'),tuple(components),len(keys),rejected)
