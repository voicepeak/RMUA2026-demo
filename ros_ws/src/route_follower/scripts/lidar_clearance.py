#!/usr/bin/env python3
"""Local point-cloud clearance around the continuous flight path.

Select the smallest safe path displacement; never pull toward a doorway center.
All vectors are world NED relative to the aircraft. This is local clearance,
not a replacement for the course topology or a claim that unobserved space is free.
"""
import math
import numpy as np

class LidarClearance:
    def __init__(self, margin=1.0, braking=4.):
        self.margin=margin
        self.braking=float(braking)
        self.previous=np.zeros(2)

    @staticmethod
    def progress_cap(result,current_lateral,current_vertical,desired_lateral,desired_vertical):
        cap=result.get('cap')
        aligned=(result.get('active',False) and result.get('feasible',False)
                 and not result.get('stale',False)
                 and not result.get('recovery',False)
                 and abs(current_lateral-desired_lateral)<.25
                 and abs(current_vertical-desired_vertical)<.25)
        if aligned:
            # The baseline intersects the obstacle; the selected ray is clear
            # and the aircraft has already reached its offset.
            cap=max(cap or 0.,result.get('path_cap',
                    min(2.,result.get('obstacle_distance',0.)/1.5)))
        elif (result.get('active',False) and result.get('feasible',False)
              and result.get('obstacle_distance',float('inf'))<8.):
            # evaluate_path checks an initial shift in place. Close to the
            # surface, execute that shift before moving along its clear path.
            cap=0.
        return cap,aligned

    @staticmethod
    def preview_distance(speed, braking=4., latency=.5):
        # At 15 m/s the braking distance alone is 28 m, beyond the old 15 m ray.
        speed=max(0.,float(speed))
        return max(12.,min(50.,speed*latency+speed*speed/(2.*braking)+4.))

    @staticmethod
    def _segment_squared(points,starts,ends):
        vectors=ends-starts
        length2=np.sum(vectors*vectors,axis=1)
        dot=np.einsum('ij,kj->ik',points,vectors)-np.sum(starts*vectors,axis=1)
        t=np.clip(dot/np.maximum(length2,1e-9),0.,1.)
        delta2=(np.sum(points*points,axis=1)[:,None]+
                np.sum(starts*starts,axis=1)-2.*np.einsum('ij,kj->ik',points,starts))
        return np.maximum(0.,delta2+t*t*length2-2.*t*dot)

    def evaluate_path(self,points,reference,current_offset=(0.,0.)):
        """Check an initial displacement and the displaced, curved route.

        The reference is relative world NED, starting at the nominal route
        point abreast of the aircraft. Offsets remain absolute to that route.
        An aircraft already inside the 1 m buffer may escape only along paths
        that never reduce its existing clearance; the destination must restore
        the full buffer. This is not permission to enter a narrower gap.
        """
        points=np.asarray(points,dtype=float)
        reference=np.asarray(reference,dtype=float)
        empty=dict(lateral=0.,vertical=0.,cap=None,clearance=None,active=False)
        if len(reference)<2 or len(points)<20:return empty
        direction=reference[1,:2]-reference[0,:2]
        distance=float(np.sum(np.linalg.norm(np.diff(reference[:,:2],axis=0),axis=1)))
        if distance<1.:return empty
        f=direction/max(1e-6,np.linalg.norm(direction))
        side=np.array([-f[1],f[0],0.])
        longitudinal=points[:,:2]@f
        mask=(longitudinal>-3.)&(longitudinal<distance+2.)&(abs(points@side)<7.)
        mask&=abs(points[:,2])<max(6.,np.max(abs(reference[:,2]))+4.)
        points=points[mask & np.all(np.isfinite(points),axis=1)]
        if len(points)<10:return empty
        _,indices=np.unique(np.floor(points/.2).astype(np.int32),axis=0,return_index=True)
        # Keep every near point: voxel selection must not hide the limiting
        # surface when recovering from an existing buffer infringement.
        indices=np.union1d(indices,np.flatnonzero(np.linalg.norm(points,axis=1)<1.3))
        points=points[indices]
        longitudinal=points[:,:2]@f
        initial2=np.sum(points*points,axis=1)
        initial=float(np.sqrt(np.min(initial2)))
        threshold=np.minimum(self.margin**2,initial2)
        if initial>=self.margin:
            baseline=np.concatenate((np.zeros((1,3)),reference),axis=0)
            baseline_min=np.full(len(points),np.inf)
            for a,b in zip(baseline[:-1],baseline[1:]):
                baseline_min=np.minimum(baseline_min,
                    self._segment_squared(points,a[None,:],b[None,:])[:,0])
            if np.min(baseline_min)>=self.margin**2:
                self.previous*=.5
                return dict(empty,clearance=float(np.sqrt(np.min(baseline_min))),
                            initial_clearance=initial)
        choices=np.array([(y,z) for y in np.arange(-3.,3.01,.5)
                          for z in (-1.5,-1.,-.5,0.,.5,1.,1.5)])
        tangents=np.gradient(reference[:,:2],axis=0)
        tangents/=np.maximum(np.linalg.norm(tangents,axis=1)[:,None],1e-6)
        sides=np.column_stack([-tangents[:,1],tangents[:,0],np.zeros(len(reference))])
        shifted=reference[:,None,:]+choices[None,:,0,None]*sides[:,None,:]
        shifted[:,:,2]+=choices[None,:,1]
        offsets=shifted[0]
        minimum=self._segment_squared(points,np.zeros_like(offsets),shifted[0])
        for start,end in zip(shifted[:-1],shifted[1:]):
            # Points outside every candidate's bounding box cannot infringe
            # its margin. Avoid repeatedly projecting the near car onto far
            # route segments; clearance checks run inside the control loop.
            lo=np.minimum(start.min(axis=0),end.min(axis=0))-self.margin
            hi=np.maximum(start.max(axis=0),end.max(axis=0))+self.margin
            nearby=np.all((points>=lo)&(points<=hi),axis=1)
            if np.any(nearby):
                minimum[nearby]=np.minimum(minimum[nearby],
                    self._segment_squared(points[nearby],start,end))
        # End the initial shift with full clearance. Inside the buffer, permit
        # motion away from each close surface, with no loss of separation.
        shift_clear=np.sqrt(np.min(np.sum((points[:,None,:]-shifted[0][None,:,:])**2,axis=2),axis=0))
        feasible=np.all(minimum>=threshold[:,None]-1e-6,axis=0)&(shift_clear>=self.margin)
        clearance=np.sqrt(np.min(minimum,axis=0))
        zero=int(np.flatnonzero(np.all(choices==0.,axis=1))[0])
        if feasible[zero] and initial>=self.margin:
            self.previous*=.5
            return dict(empty,clearance=float(clearance[zero]),initial_clearance=initial)
        effort=np.sum(choices*choices,axis=1)+.25*np.sum((choices-self.previous)**2,axis=1)
        if np.any(feasible):
            index=int(np.argmin(np.where(feasible,effort,np.inf)))
            previous_index=int(np.argmin(np.sum((choices-self.previous)**2,axis=1)))
            if (np.linalg.norm(self.previous)>.25 and feasible[previous_index]
                    and np.linalg.norm(choices[previous_index]-self.previous)<.1):
                # Keep a safe side until the baseline is clear. Alternating
                # above/below between clouds never lets the aircraft align.
                index=previous_index
        else:index=int(np.argmax(clearance-.015*effort))
        self.previous=choices[index]
        baseline_close=minimum[:,zero]<self.margin**2
        obstacle=float(np.min(np.maximum(0.,longitudinal[baseline_close]))) if np.any(baseline_close) else distance
        remaining=np.abs(choices[index]-np.asarray(current_offset))
        response=max(.3,remaining[0]/1.5+remaining[1]/1.5+.3)
        path_cap=math.sqrt(2.*self.braking*max(0.,distance-self.margin-1.))
        recovery=initial<self.margin
        if recovery:path_cap=min(path_cap,1.)
        braking=math.sqrt(2.*self.braking*max(0.,obstacle-self.margin-.3))
        cap=min(path_cap,obstacle/response,braking)
        if not np.any(feasible):cap=0.
        return dict(lateral=float(choices[index,0]),vertical=float(choices[index,1]),
                    cap=cap,path_cap=path_cap,clearance=float(clearance[zero]),
                    selected_clearance=float(clearance[index]),shift_clearance=float(shift_clear[index]),
                    initial_clearance=initial,recovery=recovery,
                    feasible=bool(np.any(feasible)),obstacle_distance=obstacle,active=True)

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
        # Keep imminent surfaces: filtering everything within half a metre
        # made a blocking wall disappear just before contact.
        mask=(longitudinal>.15)&(longitudinal<distance+1.)&(abs(lateral)<7.)&(abs(points[:,2])<abs(end[2])+6.)
        points=points[mask]
        if len(points)<10:return empty
        # Spatial subsampling bounds CPU cost without deleting narrow frames.
        _,indices=np.unique(np.floor(points/.2).astype(np.int32),axis=0,return_index=True)
        points=points[indices]
        # Keep corrections inside the 10m x 5m road envelope when the nominal
        # path is near its middle; a distant gap outside the road is not usable.
        choices=np.array([(y,z) for y in np.arange(-3.,3.01,.5)
                          for z in (-1.5,-1.,-.5,0.,.5,1.,1.5)])
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
        # Slow early enough to achieve the requested displacement. Large
        # offsets cannot develop in the same time as a small correction.
        response=max(1.5,abs(choices[index,0])*.8+abs(choices[index,1])*.5+.4)
        braking_cap=math.sqrt(2.*4.*max(0.,obstacle-self.margin-.3))
        cap=min(max(0.,obstacle/response),braking_cap)
        if not np.any(feasible):cap=0.
        return dict(lateral=float(choices[index,0]),vertical=float(choices[index,1]),
                    cap=cap,clearance=float(clearance[zero]),selected_clearance=float(clearance[index]),
                    feasible=bool(np.any(feasible)),obstacle_distance=obstacle,active=True)
