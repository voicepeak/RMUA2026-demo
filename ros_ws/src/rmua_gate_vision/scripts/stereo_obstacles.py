#!/usr/bin/env python3
"""Measured stereo hits below the MID360 downward field of view.

These are occupied returns, never evidence that other pixels are free. Keeping
them separate from the gate detector also retains cars which cover an opening.
"""
import numpy as np
import cv2
from gate_stereo import FX, FY, CX, CY, BASELINE, camera_to_body, body_to_world


def downward_hits(disparity, position, quaternion, stride=4, max_depth=20., scale=1.):
    disparity=np.asarray(disparity)
    if disparity.ndim!=2 or stride<1 or not 0.<scale<=1.:raise ValueError('Invalid disparity sampling')
    fx,fy,cx,cy=FX*scale,FY*scale,CX*scale,CY*scale
    clean=np.where(np.isfinite(disparity),disparity,0.).astype(np.float32)
    support=cv2.boxFilter((clean>0).astype(np.float32),-1,(3,3),normalize=False)
    median=cv2.medianBlur(clean,3)
    consistent=(support>=5)&(abs(clean-median)<=scale)
    v,u=np.mgrid[0:disparity.shape[0]:stride,0:disparity.shape[1]:stride]
    d=clean[::stride,::stride]
    # Ignore the invalid left matching strip and the image boundary. The
    # 7-degree boundary is in the body/camera frame, not world height.
    eligible=(consistent[::stride,::stride]&(d>fx*BASELINE/max_depth)&(d<fx*BASELINE/.8)&
              (u>132*scale)&(u<disparity.shape[1]-8*scale)&(v<disparity.shape[0]-8*scale)&
              (v>cy+fy*np.tan(np.deg2rad(7.))))
    if not np.any(eligible):return np.empty((0,3))
    z=fx*BASELINE/d[eligible]
    optical=np.column_stack(((u[eligible]-cx)*z/fx,(v[eligible]-cy)*z/fy,z))
    return body_to_world(camera_to_body(optical),position,quaternion)
