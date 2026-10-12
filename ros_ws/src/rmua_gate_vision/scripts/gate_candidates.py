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


def frame_candidates(image, learned, limit=8):
    candidates=[g for g in learned if gate_frame_support(image,g['bbox'])]
    height,width=image.shape[:2]
    added=0
    for g in detect_gates(image):
        x,y,w,h=g['bbox']
        if (g['orange_ratio']<.015 or g['confidence']<.5 or
                min(w,h)<24 or not .4<=w/h<=2.5 or
                x<4 or y<4 or x+w>width-4 or y+h>height-4):continue
        if not gate_frame_support(image,g['bbox']):continue
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
