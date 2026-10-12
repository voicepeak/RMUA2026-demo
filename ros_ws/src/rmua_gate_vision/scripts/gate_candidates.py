#!/usr/bin/env python3
"""Supplement merged detector boxes with individually measured colored frames."""
import numpy as np
import cv2
from gate_detector_opencv import detect_gates


def gate_frame_support(image,bbox):
    x,y,w,h=(int(v) for v in bbox)
    crop=image[max(0,y):min(image.shape[0],y+h),max(0,x):min(image.shape[1],x+w)]
    if not crop.size:return False
    hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    mask=(hsv[:,:,0]>=3)&(hsv[:,:,0]<=22)&(hsv[:,:,1]>110)&(hsv[:,:,2]>150)&(crop[:,:,1]>65)
    v,u=np.where(mask)
    # A car's red roof is a horizontal patch, not the gate's vertical trim.
    return len(v)>=30 and np.ptp(v)>=.5*h and np.ptp(u)>=.3*w


def gate_opening_support(image,bbox):
    """Require two tall white posts, rather than one post or a car panel."""
    x,y,w,h=(int(v) for v in bbox)
    if w<24 or h<24 or x<0 or y<0 or x+w>image.shape[1] or y+h>image.shape[0]:return False
    crop=image[y:y+h,x:x+w]
    hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    white=(hsv[:,:,1]<70)&(hsv[:,:,2]>90)
    border=max(2,int(.22*w))
    top,bottom=int(.1*h),int(.9*h)
    for strip in (white[top:bottom,:border],white[top:bottom,-border:]):
        rows=np.count_nonzero(strip,axis=1)>=max(2,int(.12*border))
        if np.mean(rows)<.65:return False
    return gate_frame_support(image,bbox)


def passable_gate_geometry(geometry, body_radius=.5):
    """A sub-body opening is not a navigation gate, even if it is planar."""
    uncertainty=float(geometry.get('position_sigma') or 0.)
    minimum=2.*(body_radius+uncertainty)+.5
    return geometry['width']>=minimum and geometry['height']>=minimum


def frame_candidates(image, learned, limit=8):
    candidates=[g for g in learned if gate_opening_support(image,g['bbox'])]
    height,width=image.shape[:2]
    added=0
    for g in detect_gates(image):
        x,y,w,h=g['bbox']
        if (g['orange_ratio']<.015 or g['confidence']<.5 or
                min(w,h)<24 or not .4<=w/h<=2.5 or
                x<4 or y<4 or x+w>width-4 or y+h>height-4):continue
        if not gate_opening_support(image,g['bbox']):continue
        duplicate=False
        for old in candidates:
            a,b,c,d=old['bbox']
            intersection=max(0,min(x+w,a+c)-max(x,a))*max(0,min(y+h,b+d)-max(y,b))
            if intersection/max(1,w*h+c*d-intersection)>.6:duplicate=True;break
        if duplicate:continue
        corners=np.asarray(g['corners'],dtype=np.float32)
        candidates.append(dict(bbox=g['bbox'],center=corners.mean(axis=0),
            corners=corners,conf=g['confidence'],area=float(w*h),keypoints=False,
            source='colored_frame',geometry_only=True))
        added+=1
        if added>=limit:break
    return candidates
