#!/usr/bin/env python3
"""Exact point queries, optionally augmented by measured ceiling patches."""
import cv2
import numpy as np


def measured_car_faces(tracks,route,stamp):
    """Fresh stereo faces fill lidar holes without coarse distant extrusion."""
    faces=[]
    for track in tracks:
        age=stamp-track['stamp']
        # Stereo inference normally arrives 0.4--0.8 s after exposure. Retain
        # the same two-second lifetime as CarTracks, with age inflation.
        if not 0.<=age<2. or track['uncertainty']>=1.:continue
        center=np.asarray(track['world'])
        station=route.project_gate(*center[:2])
        forward=np.asarray(route.tangent(station)[:2])
        forward/=max(1e-6,np.linalg.norm(forward))
        faces.append(dict(world=center,forward=forward,side=np.array([-forward[1],forward[0]]),
                          extent=np.array([.6+track['uncertainty']+.3*age,
                                           track['half_width']+.25+.1*age,
                                           track['half_height']+.25+.1*age])))
    return faces


class PointIndex:
    def __init__(self,points,origin=None,faces=(),road_height=None):
        self.faces=faces
        self.road_height=road_height
        self.points=np.ascontiguousarray(points,dtype=np.float32)
        # Randomized KDTree can still overestimate nearest distances with
        # checks=-1 in this OpenCV build. Single KDTree exhaustive queries
        # avoid that unsafe dependence on the global random generator.
        self.tree=(cv2.flann_Index(self.points,dict(algorithm=4)) if len(points) else None)
        self.roof=None
        if origin is not None:self._roof(np.asarray(origin,dtype=float))

    def _roof(self,origin):
        if not len(self.points):return
        # Reconstruct only a well supported, nearly horizontal overhead patch.
        # A gap between laser returns is not a hole through a solid ceiling.
        q=self.points-origin
        q=q[(np.linalg.norm(q[:,:2],axis=1)<10.)&(q[:,2]<-.35)&(q[:,2]>-6.)]
        if len(q)<40:return
        q=q[np.argsort(q[:,2])]
        _,indices=np.unique(np.floor(q[:,:2]/.6).astype(int),axis=0,return_index=True)
        q=q[indices]
        if len(q)<40:return
        design=np.column_stack((q[:,:2],np.ones(len(q))))
        rng=np.random.default_rng(0);best=None
        for _ in range(96):
            pick=rng.choice(len(q),3,replace=False)
            try:coeff=np.linalg.solve(design[pick],q[pick,2])
            except np.linalg.LinAlgError:continue
            if np.linalg.norm(coeff[:2])>.65:continue
            mask=np.abs(q[:,2]-design@coeff)<.12
            if best is None or np.sum(mask)>np.sum(best):best=mask
        if best is None or np.sum(best)<30 or np.mean(best)<.3:return
        coeff=np.linalg.lstsq(design[best],q[best,2],rcond=None)[0]
        if np.linalg.norm(coeff[:2])>.65:return
        support=q[best,:2]
        if np.min(np.linalg.eigvalsh(np.cov(support.T)))<.5:return
        hull=cv2.convexHull(support.astype(np.float32)).reshape(-1,2)
        # Never extrapolate a fitted surface beyond its measured convex hull.
        edge=np.roll(hull,-1,axis=0)-hull
        normal=np.column_stack((-edge[:,1],edge[:,0]))
        normal/=np.linalg.norm(normal,axis=1)[:,None]
        middle=np.mean(hull,axis=0)
        if np.mean(np.sum((middle-hull)*normal,axis=1))<0.:normal=-normal
        self.roof=(origin,coeff,hull,normal)

    def surface_distance(self,queries):
        queries=np.asarray(queries,dtype=float).reshape(-1,3)
        result=np.full(len(queries),np.inf)
        if self.roof is None:return result
        origin,coeff,hull,normal=self.roof
        q=queries-origin
        inside=np.all(np.sum((q[:,None,:2]-hull[None,:,:])*normal[None,:,:],axis=2)>=0.,axis=1)
        height=q[inside,2]-q[inside,:2]@coeff[:2]-coeff[2]
        signed=(np.minimum(height,self.road_height-height) if self.road_height is not None else abs(height))
        result[inside]=signed/np.sqrt(1.+np.sum(coeff[:2]**2))
        return result

    def floor_limit(self,queries,margin):
        """Maximum NED Z inside a measured road ceiling's support hull."""
        queries=np.asarray(queries,dtype=float).reshape(-1,3)
        result=np.full(len(queries),np.nan)
        if self.roof is None or self.road_height is None:return result
        origin,coeff,hull,normal=self.roof;q=queries-origin
        inside=np.all(np.sum((q[:,None,:2]-hull[None,:,:])*normal[None,:,:],axis=2)>=0.,axis=1)
        result[inside]=origin[2]+q[inside,:2]@coeff[:2]+coeff[2]+self.road_height-margin*np.sqrt(1.+np.sum(coeff[:2]**2))
        return result

    def face_distance(self,queries,signed=False):
        queries=np.asarray(queries).reshape(-1,3)
        result=np.full(len(queries),np.inf)
        for face in self.faces:
            delta=queries-face['world']
            local=np.column_stack((delta[:,:2]@face['forward'],delta[:,:2]@face['side'],delta[:,2]))
            separation=abs(local)-face['extent']
            distance=np.linalg.norm(np.maximum(separation,0.),axis=1)
            if signed:distance+=np.minimum(np.max(separation,axis=1),0.)
            result=np.minimum(result,distance)
        return result

    def distance(self,queries):
        queries=np.ascontiguousarray(queries,dtype=np.float32).reshape(-1,3)
        if self.tree is None:return np.minimum(self.surface_distance(queries),self.face_distance(queries))
        _,distance=self.tree.knnSearch(queries,1,params=dict(checks=-1))
        return np.minimum(np.minimum(np.sqrt(np.maximum(0.,distance[:,0])),self.surface_distance(queries)),
                          self.face_distance(queries))
