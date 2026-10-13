#!/usr/bin/env python3
"""Two measured trim components, excluding buildings and nested far gates.

A position/height observation does not certify a four-corner opening plane.
A textureless post may yield depth only at its cap. A fully measured opposite
post then supplies the common vertical center, after complete image support.
"""
import numpy as np


def trim_center(cand,disparity,trim,fx,fy,cx,cy,baseline,gravity=None):
    x,y,w,h=(int(v) for v in cand['bbox']);height,width=disparity.shape
    x0,y0=max(0,x),max(0,y);x1,y1=min(width,x+w),min(height,y+h)
    if x0>=x1 or y0>=y1:return None
    v,u=np.nonzero(trim[y0:y1,x0:x1]);u=u+x0;v=v+y0
    band=max(1,int(w*.35));left=u<x+band;right=u>=x+w-band
    dv=disparity[v,u];valid=np.isfinite(dv)&(dv>0)
    u,v,dv,left,right=(a[valid] for a in (u,v,dv,left,right))
    vertical=np.asarray([0.,1.,0.] if gravity is None else gravity,dtype=float).copy()
    vertical/=max(1e-9,np.linalg.norm(vertical))
    posts=[];complete=[];centers=[];midpoints=[]
    for side in (left,right):
        component=None
        for level in np.unique(np.floor(dv[side]*2.)/2.)[::-1]:
            candidate=side&(abs(dv-level)<=max(.35,.05*level))
            if np.count_nonzero(candidate)<12:continue
            # A cap is a usable range measurement; vertical completeness is
            # checked jointly below, not fabricated from a single cap.
            component=candidate;break
        if component is None:return None
        uu,vv,dd=u[component],v[component],dv[component]
        if len(uu)>512:
            sample=np.linspace(0,len(uu)-1,512,dtype=int);uu,vv,dd=(a[sample] for a in (uu,vv,dd))
        depth=fx*baseline/dd
        post=np.column_stack(((uu-cx)*depth/fx,(vv-cy)*depth/fy,depth))
        lo,hi=np.percentile(post@vertical,[5,95]);vlo,vhi=np.percentile(vv,[5,95])
        complete.append(hi-lo>=.5 and vhi-vlo>=.35*h)
        centers.append(np.median(post,axis=0));midpoints.append(.5*(lo+hi));posts.append(post)
    if not any(complete):return None
    common=float(np.mean([mid for mid,full in zip(midpoints,complete) if full]))
    for i,center in enumerate(centers):
        target=midpoints[i] if complete[i] else common
        centers[i]=center+vertical*(target-center@vertical)
    if np.linalg.norm(centers[1]-centers[0])>18.:return None
    center=.5*(centers[0]+centers[1])
    if not np.all(np.isfinite(center)) or center[2]<=0:return None
    return center,float(center[2]),sum(len(p) for p in posts)
