#!/usr/bin/env python3
"""Discard nonfinite/zero-range samples before any sensor-frame transform."""
import numpy as np
from dataclasses import dataclass


def valid_points(points):
    points=np.asarray(points)
    # A return at the emitting origin carries no obstacle position. Do not
    # reject a radius around the aircraft: genuine near surfaces still matter.
    return points[np.all(np.isfinite(points),axis=1)&(np.sum(points*points,axis=1)>1e-8)]


def decode_point_cloud(message):
    """Decode float32 XYZ including organized clouds' row padding."""
    fields={field.name:field for field in message.fields}
    if (message.width<0 or message.height<0 or message.point_step<=0 or
            message.row_step<message.width*message.point_step or
            any(key not in fields or fields[key].datatype!=7 for key in ('x','y','z'))):
        raise ValueError('Invalid PointCloud2 XYZ layout')
    endian='>' if message.is_bigendian else '<'
    dtype=np.dtype(dict(names=['x','y','z'],formats=[endian+'f4']*3,
                        offsets=[fields[key].offset for key in ('x','y','z')],itemsize=message.point_step))
    raw=np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                   strides=(message.row_step,message.point_step))
    return valid_points(np.column_stack([raw[key].reshape(-1) for key in ('x','y','z')]))


@dataclass(frozen=True)
class LidarFrame:
    """One raw world frame paired atomically with its exposure-time origin."""
    stamp: float
    points: np.ndarray
    sensor_origin: np.ndarray
    epoch: int = 0

    def __post_init__(self):
        points=np.asarray(self.points);origin=np.asarray(self.sensor_origin,dtype=float)
        if (not np.isfinite(self.stamp) or points.ndim!=2 or points.shape[1]!=3 or
                not np.all(np.isfinite(points)) or origin.shape!=(3,) or not np.all(np.isfinite(origin)) or
                isinstance(self.epoch,bool) or not isinstance(self.epoch,(int,np.integer)) or self.epoch<0):
            raise ValueError('Invalid synchronized LiDAR frame')
        points=points.copy();origin=origin.copy()
        points.setflags(write=False);origin.setflags(write=False)
        object.__setattr__(self,'points',points);object.__setattr__(self,'sensor_origin',origin)
