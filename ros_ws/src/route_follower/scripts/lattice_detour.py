#!/usr/bin/env python3
"""Multi-obstacle route lattice plus certified smooth offset interpolation."""
import numpy as np
import time
from spatial_curve import SpatialCurve


class LatticeDetour:
    def __init__(self,stations,offsets,start_slope=(0.,0.)):
        self.s=float(stations[0]);self.end=float(stations[-1]);self.length=self.end-self.s
        self.target=offsets[int(np.argmax(np.linalg.norm(offsets,axis=1)))].copy()
        self.y=SpatialCurve(zip(stations,offsets[:,0]));self.z=SpatialCurve(zip(stations,offsets[:,1]))
        self.y.m[0],self.z.m[0]=start_slope
        self.y.m[-1]=self.z.m[-1]=0.
        self.recentered=bool(np.linalg.norm(offsets[-1])<1e-6)
    def offset(self,s):return np.array([self.y.center(s),self.z.center(s)])
    def offsets(self,ss):return np.array([self.offset(s) for s in np.atleast_1d(ss)])
    def slope(self,s):return np.array([self.y.dz_ds(s),self.z.dz_ds(s)])


def search(stations,nominal,sides,start,index,margin,cars,old_plan=None,start_slope=(0.,0.),deadline=None,vertical_limit=1.5):
    if deadline is not None and time.monotonic()>deadline:return None
    offsets=np.array([(y,z) for y in np.arange(-3.,3.01,.5)
                      for z in np.arange(-vertical_limit,vertical_limit+.01,.5)])
    width=len(offsets);count=len(stations)
    paths=nominal[:,None,:]+offsets[None,:,0,None]*sides[:,None,:]
    paths[:,:,2]+=offsets[None,:,1]
    clearance=index.distance(paths.reshape(-1,3)).reshape(count,width)
    valid=clearance>=margin+.15
    car_penalty=np.zeros_like(clearance)
    for car in cars:
        delta=paths-car['world'];f=car['forward'];side=car['side']
        inside=(abs(delta[:,:,:2]@f)<car['extent'][0])&(abs(delta[:,:,:2]@side)<car['extent'][1])&(abs(delta[:,:,2])<car['extent'][2])
        valid&=~inside
    cost=.035*np.sum(offsets*offsets,axis=1)[None,:]+.03/np.maximum(.2,clearance)
    cost=np.broadcast_to(cost,(count,width)).copy()
    if old_plan is not None:
        previous=old_plan.offsets(stations)
        cost+=.1*np.sum((offsets[None,:,:]-previous[:,None,:])**2,axis=2)
    # Adjacent lattice cells only. Every diagonal is checked with midpoint
    # samples; the final smoothed curve undergoes denser full certification.
    delta=offsets[:,None,:]-offsets[None,:,:]
    neighbors=np.all(abs(delta)<=.501,axis=2)
    rows,cols=np.nonzero(neighbors)
    step_cost=.7*np.sum(delta*delta,axis=2)
    previous=np.full(width,np.inf)
    parent=np.full((count,width),-1,dtype=np.int32)
    diff=offsets-start
    reachable=np.linalg.norm(diff,axis=1)<1.01
    for j in np.flatnonzero(reachable & valid[1]):
        a=nominal[0]+start[0]*sides[0];a[2]+=start[1]
        b=paths[1,j];samples=a+np.linspace(0.,1.,8)[:,None]*(b-a)
        if np.min(index.distance(samples))>=margin+.1:
            previous[j]=cost[1,j]+2.*float(np.sum(diff[j]**2))
    if not np.any(np.isfinite(previous)):return None
    for i in range(2,count):
        if deadline is not None and time.monotonic()>deadline:return None
        transitions=previous[None,:]+step_cost
        transitions[~neighbors]=np.inf
        transitions[~valid[i],:]=np.inf
        # Favor continuing the last direction over alternating cells.
        grand=parent[i-1]
        prev_delta=np.zeros_like(offsets)
        known=grand>=0;prev_delta[known]=offsets[known]-offsets[grand[known]]
        transitions+=2.*np.sum((delta-prev_delta[None,:,:])**2,axis=2)
        eligible=np.isfinite(transitions[rows,cols])
        rr,cc=rows[eligible],cols[eligible]
        if len(rr):
            a=paths[i-1,cc];b=paths[i,rr]
            mid=np.stack([a+(b-a)*t for t in (.2,.4,.6,.8)],axis=1)
            safe=np.min(index.distance(mid.reshape(-1,3)).reshape(len(rr),4),axis=1)>=margin+.15
            transitions[rr[~safe],cc[~safe]]=np.inf
        pred=np.argmin(transitions,axis=1)
        previous=transitions[np.arange(width),pred]+cost[i]
        parent[i]=pred
        if not np.any(np.isfinite(previous)):return None
    center=int(np.argmin(np.sum(offsets*offsets,axis=1)))
    # Return to zero when reachable. Otherwise keep the nonzero terminal
    # position with zero derivative for a continuous subsequent replan.
    end=center if np.isfinite(previous[center]) else int(np.argmin(previous+.3*np.sum(offsets*offsets,axis=1)))
    selected=[end]
    for i in range(count-1,1,-1):selected.append(int(parent[i,selected[-1]]))
    result=np.vstack([start,offsets[selected[::-1]]])
    # Prefer smooth curves. Keep endpoints fixed, and let the caller reject
    # any smoothing that reduces the complete swept-path clearance.
    candidates=[]
    for passes in (12,6,3,0):
        values=result.copy()
        for _ in range(passes):values[1:-1]=.25*values[:-2]+.5*values[1:-1]+.25*values[2:]
        candidates.append(LatticeDetour(stations,values,start_slope))
    return candidates
