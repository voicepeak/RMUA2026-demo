#!/usr/bin/env python3
"""Local, nonsemantic obstacle association and future occupied boxes.

Only interior clusters with consistent translating bounds acquire velocity.
Static points/surface patches remain authoritative for every raw return.
"""
import numpy as np
from scipy.ndimage import label, binary_dilation


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
        forward=np.asarray(forward);side=np.array([-forward[1],forward[0]])
        relative=np.asarray(points)-position
        along=relative[:,:2]@forward
        cross=relative[:,:2]@side
        stations=s+np.arange(-30.,30.1,.5)
        heights=np.array([center(t) for t in stations])
        dz=points[:,2]-np.interp(s+along,stations,heights)
        eligible=(abs(along)<25.)&(abs(cross)<3.8)&(abs(dz)<1.8)
        selected=np.asarray(points)[eligible]
        if len(selected)<8:
            self.tracks=[t for t in self.tracks if stamp-t['stamp']<.4]
            self.stamp=stamp;return
        # World-aligned cells keep translation distinct from drone motion.
        cells=np.floor(selected/.45).astype(int)
        origin=cells.min(axis=0);shape=cells.max(axis=0)-origin+3
        occupancy=np.zeros(shape,dtype=bool);local=cells-origin+1
        occupancy[tuple(local.T)]=True
        labels,_=label(binary_dilation(occupancy))
        ids=labels[tuple(local.T)]
        clusters=[]
        for key in np.unique(ids):
            cloud=selected[ids==key]
            if len(cloud)<20:continue
            low=cloud.min(axis=0);high=cloud.max(axis=0)
            if np.max(high-low)>9.:continue
            center_point=(low+high)/2.
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
