#!/usr/bin/env python3
"""Local point-cloud clearance around the continuous flight path.

Select the smallest safe path displacement; never pull toward a doorway center.
All vectors are world NED relative to the aircraft. This is local clearance,
not a replacement for the course topology or a claim that unobserved space is free.
"""
import math
import numpy as np

class LidarClearance:
    def __init__(self, margin=.8):
        self.margin=margin
        self.previous=np.zeros(2)

    def evaluate(self,points,endpoint):
        points=np.asarray(points,dtype=float)
        end=np.asarray(endpoint,dtype=float)
        distance=math.hypot(end[0],end[1])
        empty=dict(lateral=0.,vertical=0.,cap=None,clearance=None,active=False)
        if distance<1. or len(points)<20:return empty
        f=end[:2]/distance
        side=np.array([-f[1],f[0],0.])
        longitudinal=points[:,:2]@f
        lateral=points@side
        mask=(longitudinal>.5)&(longitudinal<distance+1.)&(abs(lateral)<7.)&(abs(points[:,2])<abs(end[2])+6.)
        points=points[mask]
        if len(points)<10:return empty
        # Spatial subsampling bounds CPU cost without deleting narrow frames.
        _,indices=np.unique(np.floor(points/.2).astype(np.int32),axis=0,return_index=True)
        points=points[indices]
        choices=np.array([(y,z) for y in np.arange(-5.,5.01,.5)
                          for z in (-3.,-2.5,-2.,-1.5,-1.,-.5,0.,.5,1.,1.5,2.)])
        vectors=end[None,:]+choices[:,0,None]*side[None,:]
        vectors[:,2]+=choices[:,1]
        length2=np.sum(vectors*vectors,axis=1)
        dot=points@vectors.T
        t=np.clip(dot/length2,0.,1.)
        squared=np.sum(points*points,axis=1)[:,None]+t*t*length2-2*t*dot
        clearance=np.sqrt(np.maximum(0.,np.min(squared,axis=0)))
        zero=int(np.flatnonzero(np.all(choices==0.,axis=1))[0])
        if clearance[zero]>=self.margin:
            self.previous*=.5
            return dict(empty,clearance=float(clearance[zero]))
        feasible=clearance>=self.margin
        effort=np.sum(choices*choices,axis=1)+.25*np.sum((choices-self.previous)**2,axis=1)
        if np.any(feasible):
            index=int(np.argmin(np.where(feasible,effort,np.inf)))
        else:
            index=int(np.argmax(clearance-.015*effort))
        self.previous=choices[index]
        # Allow time for the offset to develop before reaching the blocking frame.
        close=np.sqrt(np.maximum(0.,squared[:,zero]))<self.margin
        ahead=np.maximum(.0,points[:,:2]@f)
        obstacle=float(np.min(ahead[close])) if np.any(close) else distance
        cap=max(1.,obstacle/1.5)
        if clearance[index]<.35:cap=0.
        return dict(lateral=float(choices[index,0]),vertical=float(choices[index,1]),
                    cap=cap,clearance=float(clearance[zero]),selected_clearance=float(clearance[index]),active=True)
