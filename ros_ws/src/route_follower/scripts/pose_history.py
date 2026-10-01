#!/usr/bin/env python3
"""Bounded pose interpolation for lidar timestamps; no stale extrapolation."""
from collections import deque
import numpy as np


class PoseHistory:
    def __init__(self):self.rows=deque()
    def add(self,stamp,position,quaternion):
        if self.rows and stamp<self.rows[-1][0]:self.rows.clear()
        if self.rows and stamp==self.rows[-1][0]:return
        self.rows.append((stamp,np.array(position),np.array(quaternion)))
        while self.rows and stamp-self.rows[0][0]>2.:self.rows.popleft()
    def sample(self,stamp):
        if not self.rows or stamp<self.rows[0][0] or stamp>self.rows[-1][0]:return None
        for t,p,q in self.rows:
            if t==stamp:return p.copy(),q.copy()
        for (a,p,q),(b,r,u) in zip(self.rows,list(self.rows)[1:]):
            if a<stamp<b:
                t=(stamp-a)/(b-a);dot=float(q@u)
                if dot<0.:u=-u;dot=-dot
                if dot>.9995:v=q+t*(u-q)
                else:
                    theta=np.arccos(np.clip(dot,-1.,1.))
                    v=(np.sin((1-t)*theta)*q+np.sin(t*theta)*u)/np.sin(theta)
                return p+t*(r-p),v/max(1e-9,np.linalg.norm(v))
        return None
