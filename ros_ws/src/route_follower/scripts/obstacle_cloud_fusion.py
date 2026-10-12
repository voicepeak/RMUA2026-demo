#!/usr/bin/env python3
"""Cache occupied camera hits alongside the fresh authoritative LiDAR frame."""
import numpy as np


class ObstacleCloudFusion:
    def __init__(self, max_age=.6):
        self.max_age=float(max_age)
        self.stamp=None
        self.points=np.empty((0,3))
        self._key=None
        self._combined=None

    def reset(self):
        self.stamp=None;self.points=np.empty((0,3));self._key=None;self._combined=None

    def update(self,stamp,points):
        points=np.asarray(points,dtype=float)
        if not np.isfinite(stamp) or points.ndim!=2 or points.shape[1]!=3 or not np.all(np.isfinite(points)):
            raise ValueError('Invalid stereo obstacle observation')
        if self.stamp is not None and stamp<=self.stamp:return False
        if not len(points):return False  # No hits are not proof that an old object disappeared.
        self.stamp=float(stamp);self.points=points.copy();self.points.setflags(write=False)
        self._key=None
        return True

    def combine(self,lidar,cloud_stamp,pose_stamp):
        if lidar is None or not len(lidar):return lidar
        active=(self.stamp is not None and 0.<=pose_stamp-self.stamp<=self.max_age and len(self.points)>0)
        if not active:return lidar
        key=(id(lidar),cloud_stamp,self.stamp)
        if key!=self._key:
            self._combined=np.concatenate((lidar,self.points));self._combined.setflags(write=False);self._key=key
        return self._combined
