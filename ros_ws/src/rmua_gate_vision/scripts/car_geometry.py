#!/usr/bin/env python3
"""Car-face observations; lidar remains responsible for whole-body clearance."""
import numpy as np
import gate_stereo as gs


def car_boxes(result,confidence=.5):
    rows=[]
    for box in result.boxes:
        if result.names[int(box.cls.item())]!='car':continue
        conf=float(box.conf.item())
        if conf<confidence:continue
        rows.append(dict(bbox=[float(v) for v in box.xyxy[0].cpu().tolist()],confidence=conf))
    return rows


def overlaps_car(gate,cars):
    x,y,w,h=gate['bbox'];area=w*h
    for car in cars:
        a,b,c,d=car['bbox']
        intersection=max(0.,min(x+w,c)-max(x,a))*max(0.,min(y+h,d)-max(y,b))
        union=area+(c-a)*(d-b)-intersection
        if union>0. and intersection/union>.55:return True
    return False


def locate_car(car,disparity,position,quaternion):
    observation=dict(car,geometry_valid=False)
    a,b,c,d=car['bbox'];w=c-a;h=d-b
    x0=max(0,int(a+.2*w));x1=min(disparity.shape[1],int(c-.2*w))
    y0=max(0,int(b+.2*h));y1=min(disparity.shape[0],int(d-.2*h))
    values=disparity[y0:y1,x0:x1].ravel()
    values=values[np.isfinite(values)&(values>1.)]
    if len(values)<16:return observation
    median=float(np.median(values));q25,q75=np.percentile(values,[25,75])
    spread=float(q75-q25)
    if spread>max(2.,median*.3):return observation
    depth=gs.FX*gs.BASELINE/median
    if not 1.<depth<100.:return observation
    optical=np.array([((a+c)/2-gs.CX)*depth/gs.FX,
                      ((b+d)/2-gs.CY)*depth/gs.FY,depth])
    body=gs.camera_to_body(optical)
    observation.update(geometry_valid=True,world=gs.body_to_world(body,position,quaternion).tolist(),
        depth=depth,depth_near=gs.FX*gs.BASELINE/(median+max(1.,spread)),
        face_half_width=.5*w*depth/gs.FX,face_half_height=.5*h*depth/gs.FY,
        disparity_pixels=median)
    return observation
