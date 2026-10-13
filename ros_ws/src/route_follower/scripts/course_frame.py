#!/usr/bin/env python3
"""Bounded rigid correction of exported UP course points into world NED XY.

Tilt comes from this run's validated height calibration, never a seed table.
Keep raw spline heights and station labels; only change horizontal geometry.
"""
import numpy as np


def mapped_course_points(points, tilt):
    points=np.asarray(points,dtype=float)
    tilt=np.asarray(tilt,dtype=float)
    if (points.ndim!=2 or points.shape[1]!=3 or tilt.shape!=(2,) or
            not np.all(np.isfinite(points)) or not np.all(np.isfinite(tilt)) or
            np.linalg.norm(tilt)>.12):
        raise ValueError('Invalid course frame calibration')
    rotation=course_rotation(tilt)
    ned=points.copy();ned[:,2]*=-1.
    mapped=ned@rotation.T
    mapped[:,2]=points[:,2]  # raw UP is not a flight altitude command
    return mapped


def course_rotation(tilt):
    tilt=np.asarray(tilt,dtype=float)
    if tilt.shape!=(2,) or not np.all(np.isfinite(tilt)) or np.linalg.norm(tilt)>.12:
        raise ValueError('Invalid course frame calibration')
    # R[2,0:2] approximately equals the fitted vertical-datum tilt.
    rotation_vector=np.array([tilt[1],-tilt[0],0.])
    angle=np.linalg.norm(rotation_vector)
    skew=np.array([[0.,0.,rotation_vector[1]],
                   [0.,0.,-rotation_vector[0]],
                   [-rotation_vector[1],rotation_vector[0],0.]])
    rotation=(np.eye(3) if angle<1e-12 else
              np.eye(3)+np.sin(angle)/angle*skew+(1.-np.cos(angle))/angle**2*(skew@skew))
    return rotation
