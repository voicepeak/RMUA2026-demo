#!/usr/bin/env python3
"""Measured two-post trim depth; white buildings and inner gates are excluded."""
import numpy as np


def trim_center(cand,disparity,trim,fx,fy,cx,cy,baseline,gravity=None):
    x,y,w,h=(int(v) for v in cand['bbox']);height,width=disparity.shape
    x0,y0=max(0,x),max(0,y);x1,y1=min(width,x+w),min(height,y+h)
    if x0>=x1 or y0>=y1:return None
    v,u=np.nonzero(trim[y0:y1,x0:x1]);u=u+x0;v=v+y0
    band_x=max(1,int(w*.22));band_y=max(1,int(h*.22))
    left=u<x+band_x;right=u>=x+w-band_x
    outside=left|right|(v<y+band_y)|(v>=y+h-band_y)
    dv=disparity[v,u]
    valid=outside&np.isfinite(dv)&(dv>0)
    u,v,dv,left,right=(a[valid] for a in (u,v,dv,left,right))
    if min(np.count_nonzero(left),np.count_nonzero(right))<12:return None
    if len(u)>512:
        take=np.linspace(0,len(u)-1,512,dtype=int)
        u,v,dv,left,right=(a[take] for a in (u,v,dv,left,right))
    depth=fx*baseline/dv
    points=np.column_stack(((u-cx)*depth/fx,(v-cy)*depth/fy,depth))
    rng=np.random.default_rng(0);best=None
    for _ in range(48):
        sample=points[rng.choice(len(points),3,replace=False)]
        n=np.cross(sample[1]-sample[0],sample[2]-sample[0]);length=np.linalg.norm(n)
        if length<1e-8:continue
        n/=length
        if abs(n[2])<.25:continue
        mask=abs((points-sample[0])@n)<.2
        if min(np.count_nonzero(mask&left),np.count_nonzero(mask&right))<12:continue
        if best is None or np.count_nonzero(mask)>np.count_nonzero(best):best=mask
    if best is None or np.mean(best)<.65:
        # The two recessed C-shaped posts need not share one front plane.
        # Keep independent measured post centers as non-geometric depth
        # evidence; never manufacture four-corner/plane validity for them.
        vertical=np.asarray([0.,1.,0.] if gravity is None else gravity,dtype=float)
        vertical/=max(1e-9,np.linalg.norm(vertical));centers=[]
        for post in (points[left],points[right]):
            median=float(np.median(post[:,2]));mad=float(np.median(abs(post[:,2]-median)))
            post=post[abs(post[:,2]-median)<=max(.6,3.*mad)]
            if len(post)<12:return None
            lo,hi=np.percentile(post@vertical,[5,95])
            if hi-lo<.5:return None
            center=np.median(post,axis=0);center+=vertical*(.5*(lo+hi)-center@vertical)
            centers.append(center)
        if np.linalg.norm(centers[1]-centers[0])>18.:return None
        center=.5*(centers[0]+centers[1])
        return center,float(center[2]),int(len(points))
    measured=points[best];origin=measured.mean(axis=0)
    _,sv,vt=np.linalg.svd(measured-origin,full_matrices=False);normal=vt[-1]
    if sv[1]<.01 or abs(normal[2])<.25:return None
    vertical=np.asarray([0.,1.,0.] if gravity is None else gravity,dtype=float)
    vertical=vertical-normal*(vertical@normal);length=np.linalg.norm(vertical)
    if length<.7:return None
    vertical/=length
    centers=[]
    for post in (measured[left[best]],measured[right[best]]):
        lo,hi=np.percentile(post@vertical,[5,95]);center=np.median(post,axis=0)
        center+=vertical*(.5*(lo+hi)-center@vertical)
        center-=normal*((center-origin)@normal);centers.append(center)
    center=.5*(centers[0]+centers[1])
    if not np.all(np.isfinite(center)) or center[2]<=0:return None
    return center,float(center[2]),int(len(measured))
