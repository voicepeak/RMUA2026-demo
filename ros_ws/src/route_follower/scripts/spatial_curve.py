#!/usr/bin/env python3
"""Shape-preserving C1 Hermite interpolation, with analytic spatial derivative."""
from bisect import bisect_right
import math
import numpy as np

class SpatialCurve:
    def __init__(self, anchors):
        points = sorted({float(s): float(v) for s, v in anchors
                         if math.isfinite(s) and math.isfinite(v)}.items())
        if len(points) < 2:
            raise ValueError("At least two distinct finite anchors required")
        self.x, self.y = map(list, zip(*points))
        h = [b-a for a,b in zip(self.x, self.x[1:])]
        d = [(b-a)/w for a,b,w in zip(self.y, self.y[1:], h)]
        self.m = [d[0]] + [0.]*(len(points)-2) + [d[-1]]
        for i in range(1, len(points)-1):
            if d[i-1]*d[i] > 0:
                w1, w2 = 2*h[i]+h[i-1], h[i]+2*h[i-1]
                self.m[i] = (w1+w2)/(w1/d[i-1]+w2/d[i])
        if len(points) > 2:
            for idx, a,b,u,v in [(0,d[0],d[1],h[0],h[1]),
                                  (-1,d[-1],d[-2],h[-1],h[-2])]:
                m = ((2*u+v)*a-u*b)/(u+v)
                self.m[idx] = 0. if m*a <= 0 else math.copysign(min(abs(m),3*abs(a)),a)

    def sample(self, s):
        if s < self.x[0]: return self.y[0], 0.
        if s > self.x[-1]: return self.y[-1], 0.
        i = min(len(self.x)-2, max(0,bisect_right(self.x,s)-1))
        h = self.x[i+1]-self.x[i]
        t = (s-self.x[i])/h
        a,b,u,v = self.y[i], self.y[i+1], self.m[i], self.m[i+1]
        value = (2*t**3-3*t*t+1)*a+(t**3-2*t*t+t)*h*u+(-2*t**3+3*t*t)*b+(t**3-t*t)*h*v
        slope = (6*t*t-6*t)*(a-b)/h+(3*t*t-4*t+1)*u+(3*t*t-2*t)*v
        return value, slope

    def center(self, s): return self.sample(s)[0]
    def center_many(self, stations):
        stations=np.asarray(stations,dtype=float)
        x=np.asarray(self.x);y=np.asarray(self.y);m=np.asarray(self.m)
        i=np.clip(np.searchsorted(x,stations,side='right')-1,0,len(x)-2)
        h=x[i+1]-x[i];t=(stations-x[i])/h
        a,b,u,v=y[i],y[i+1],m[i],m[i+1]
        value=(2*t**3-3*t*t+1)*a+(t**3-2*t*t+t)*h*u+(-2*t**3+3*t*t)*b+(t**3-t*t)*h*v
        return np.where(stations<x[0],y[0],np.where(stations>x[-1],y[-1],value))
    def dz_ds(self, s, h=None): return self.sample(s)[1]
