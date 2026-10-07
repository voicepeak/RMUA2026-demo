#!/usr/bin/env python3
"""Local, nonsemantic obstacle association and future occupied boxes.

Only interior clusters with consistent translating bounds acquire velocity.
Static points/surface patches remain authoritative for every raw return.
"""
import numpy as np
from scipy.ndimage import label, binary_dilation
from dataclasses import dataclass


@dataclass(frozen=True)
class LidarCluster:
    """Observed bounds and original-return ownership, not a semantic car box."""
    center: np.ndarray
    low: np.ndarray
    high: np.ndarray
    point_indices: np.ndarray
    truncated: bool = False

    @property
    def size(self):
        return self.high-self.low


def extract_clusters(points, position, forward, center, s, resolution=.45,
                     min_points=20, max_extent=9., along_limit=25.,
                     lateral_limit=3.8, vertical_limit=1.8, ground_height=None,
                     ground_margin=.15, with_count=False):
    """Pure world-grid clustering shared with the new tracker.

    Optional ground_height is a *measured* world-NED floor, not a guessed
    centerline height. Legacy callers retain their existing ROI/thresholds.
    """
    points=np.asarray(points)
    position=np.asarray(position,dtype=float)
    forward=np.asarray(forward,dtype=float)[:2]
    parameters=(resolution,max_extent,along_limit,lateral_limit,vertical_limit)
    if (points.ndim!=2 or points.shape[1]!=3 or position.shape!=(3,) or
            forward.shape!=(2,) or not np.all(np.isfinite(position)) or
            not np.all(np.isfinite(forward)) or np.linalg.norm(forward)<1e-9 or
            any(not np.isfinite(v) or v<=0 for v in parameters) or min_points<1 or
            not np.isfinite(s) or not np.isfinite(ground_margin) or ground_margin<0):
        raise ValueError('Invalid clustering geometry or parameters')
    if not len(points):return ([],0) if with_count else []
    forward=forward/np.linalg.norm(forward)
    side=np.array([-forward[1],forward[0]])
    relative=points-position
    along=relative[:,:2]@forward
    cross=relative[:,:2]@side
    stations=s+np.arange(-max(30.,along_limit),max(30.,along_limit)+.1,.5)
    heights=np.array([center(t) for t in stations])
    if not np.all(np.isfinite(heights)):raise ValueError('Nonfinite route heights')
    dz=points[:,2]-np.interp(s+along,stations,heights)
    eligible=(np.all(np.isfinite(points),axis=1)&(abs(along)<along_limit)&
              (abs(cross)<lateral_limit)&(abs(dz)<vertical_limit))
    if ground_height is not None:
        floor=np.asarray(ground_height(s+along),dtype=float)
        eligible&=np.isfinite(floor)&(points[:,2]<floor-ground_margin)
    selected_ids=np.flatnonzero(eligible)
    if len(selected_ids)<min_points:return ([],len(selected_ids)) if with_count else []
    selected=points[selected_ids]
    cells=np.floor(selected/resolution).astype(np.int64)
    origin=cells.min(axis=0);shape=cells.max(axis=0)-origin+3
    # Bound allocation even for a misconfigured enormous ROI/resolution.
    if np.prod(shape.astype(float))>8_000_000:
        raise ValueError('Clustering ROI exceeds voxel allocation budget')
    occupancy=np.zeros(tuple(shape),dtype=bool);local=cells-origin+1
    occupancy[tuple(local.T)]=True
    labels,_=label(binary_dilation(occupancy))
    ids=labels[tuple(local.T)]
    result=[]
    for key in np.unique(ids):
        mask=ids==key;cloud=selected[mask]
        if len(cloud)<min_points:continue
        low=cloud.min(axis=0);high=cloud.max(axis=0)
        if np.max(high-low)>max_extent:continue
        indices=selected_ids[mask]
        truncated=bool(np.any((along_limit-abs(along[indices])<resolution)|
                            (lateral_limit-abs(cross[indices])<resolution)|
                            (vertical_limit-abs(dz[indices])<resolution)))
        result.append(LidarCluster((low+high)/2.,low,high,indices,truncated))
    return (result,len(selected_ids)) if with_count else result


class LidarScene:
    def __init__(self):
        self.stamp=None
        self.tracks=[]
        self.pose_stamp=None

    def reset(self):
        self.stamp=None;self.tracks=[];self.pose_stamp=None

    def update(self,points,position,stamp,pose_stamp,forward,center,s):
        self.pose_stamp=pose_stamp
        if stamp==self.stamp:return
        if self.stamp is not None and stamp<self.stamp:self.reset()
        observations,selected_count=extract_clusters(points,position,forward,center,s,with_count=True)
        if selected_count<8:
            self.tracks=[t for t in self.tracks if stamp-t['stamp']<.4]
            self.stamp=stamp;return
        clusters=[]
        for observation in observations:
            low,high,center_point=observation.low,observation.high,observation.center
            previous=None
            if self.tracks:
                distances=[np.linalg.norm(center_point-t['center']-t['velocity']*max(0.,stamp-t['stamp'])) for t in self.tracks]
                k=int(np.argmin(distances))
                if distances[k]<2.:previous=self.tracks[k]
            velocity=np.zeros(3);support=1
            if previous is not None and .025<stamp-previous['stamp']<.35:
                dt=stamp-previous['stamp']
                low_shift=low-previous['low'];high_shift=high-previous['high']
                consistent=np.linalg.norm(low_shift-high_shift)<.35
                if consistent:
                    measured=(low_shift+high_shift)/(2.*dt)
                    if np.linalg.norm(measured)<15.:
                        velocity=.65*previous['velocity']+.35*measured
                        support=previous['support']+1
                else:
                    velocity=.5*previous['velocity']
            clusters.append(dict(center=center_point,low=low,high=high,
                half=(high-low)/2.,velocity=velocity,support=support,stamp=stamp))
        # Preserve an occluded moving object briefly, with growing uncertainty.
        for old in self.tracks:
            if (stamp-old['stamp']<.4 and np.linalg.norm(old['velocity'])>1. and
                all(np.linalg.norm(old['center']-c['center'])>2. for c in clusters)):
                clusters.append(old)
        self.tracks=clusters;self.stamp=stamp

    def distance(self,queries,times):
        queries=np.asarray(queries);times=np.asarray(times)
        result=np.full(len(queries),np.inf)
        for track in self.tracks:
            if track['support']<3 or np.linalg.norm(track['velocity'])<1.:continue
            age=max(0.,self.pose_stamp-track['stamp'])
            elapsed=times+age
            center=track['center']+elapsed[:,None]*track['velocity']
            uncertainty=.15+.2*elapsed
            separation=abs(queries-center)-track['half']-uncertainty[:,None]
            result=np.minimum(result,np.linalg.norm(np.maximum(0.,separation),axis=1))
        return result

    def summary(self):
        moving=[t for t in self.tracks if t['support']>=3 and np.linalg.norm(t['velocity'])>=1.]
        return dict(interior_clusters=len(self.tracks),moving_obstacles=len(moving),
                    obstacle_velocities=[t['velocity'].tolist() for t in moving])
