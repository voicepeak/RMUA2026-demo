#!/usr/bin/env python3
"""Exact point queries, optionally augmented by measured ceiling patches."""
import cv2
import numpy as np
from response_native import NativeResponse
try:
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree=None

_patch_backend=None

def native_patch_backend():
    # Process-local handle: PointIndex remains pickleable for geometry workers.
    global _patch_backend
    if _patch_backend is None:_patch_backend=NativeResponse()
    return _patch_backend if _patch_backend.library is not None else None


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
    def __init__(self,points,origin=None,faces=(),road_height=None,continuous_surfaces=False,surface_seeds=None):
        self.faces=faces
        self.road_height=road_height
        self.points=np.ascontiguousarray(points,dtype=np.float32)
        # Randomized KDTree can still overestimate nearest distances with
        # checks=-1 in this OpenCV build. Single KDTree exhaustive queries
        # avoid that unsafe dependence on the global random generator.
        self.tree=((cKDTree(self.points) if cKDTree is not None else
                    cv2.flann_Index(self.points,dict(algorithm=4))) if len(points) else None)
        self.roof=None
        if origin is not None:self._roof(np.asarray(origin,dtype=float))
        self.patches=None
        if continuous_surfaces and origin is not None and cKDTree is not None:
            self._patches(np.asarray(origin,dtype=float),surface_seeds)

    def _patches(self,origin,surface_seeds=None):
        """Fill only small, planar, convex supported surface patches.

        Original returns remain in the index. No global car convex hull or
        extrapolation beyond a patch's measured support is introduced.
        """
        if len(self.points)<16:return
        near=(self.points[np.linalg.norm(self.points-origin,axis=1)<20.] if surface_seeds is None else
              np.asarray(surface_seeds))
        if len(near)<16:return
        _,ids=np.unique(np.floor(near/.8).astype(np.int32),axis=0,return_index=True)
        seeds=near[ids]
        indices,distance=self.neighbors(seeds,16)
        local=self.points[indices].astype(float)
        weights=(distance<=1.2**2).astype(float);count=weights.sum(axis=1)
        mean=np.sum(local*weights[:,:,None],axis=1)/np.maximum(1.,count)[:,None]
        delta=local-mean[:,None,:]
        covariance=np.einsum('nki,nkj,nk->nij',delta,delta,weights)/np.maximum(1.,count)[:,None,None]
        values,vectors=np.linalg.eigh(covariance)
        valid=(count>=8)&(values[:,1]>.012)&(values[:,0]<.02*values[:,1])
        centers=[];normals=[];bases=[];hulls=[]
        for i in np.flatnonzero(valid):
            selected=delta[i,weights[i]>0]
            normal=vectors[i,:,0];basis=vectors[i,:,1:]
            if np.max(abs(selected@normal))>.10:continue
            support=selected@basis
            hull=cv2.convexHull(support.astype(np.float32)).reshape(-1,2)
            if len(hull)<3 or cv2.contourArea(hull)<.05:continue
            centers.append(mean[i]);normals.append(normal);bases.append(basis);hulls.append(hull)
        if centers:
            counts=np.array([len(hull) for hull in hulls])
            padded=np.zeros((len(hulls),16,2),dtype=np.float32)
            for i,hull in enumerate(hulls):padded[i,:len(hull)]=hull
            rows=np.arange(16)[None,:];supported=rows<counts[:,None]
            following=(rows+1)%counts[:,None]
            rolled=np.take_along_axis(padded,np.repeat(following[:,:,None],2,axis=2),axis=1)
            edges=rolled-padded
            inward=np.stack((-edges[:,:,1],edges[:,:,0]),axis=2)
            inward/=np.maximum(1e-9,np.linalg.norm(inward,axis=2))[:,:,None]
            area=np.sum(np.where(supported,padded[:,:,0]*rolled[:,:,1]-padded[:,:,1]*rolled[:,:,0],0.),axis=1)
            inward[area<0]*=-1
            boundary=inward.astype(float);boundary[~supported]=0.
            offset=np.full(supported.shape,-np.inf)
            values=np.sum(padded*inward,axis=2)
            offset[supported]=values[supported]
            centers=np.array(centers)
            self.patches=(centers,np.array(normals),np.array(bases),boundary,offset,cKDTree(centers))

    def patch_distance(self,queries):
        queries=np.asarray(queries).reshape(-1,3)
        if self.patches is None:return np.full(len(queries),np.inf)
        centers,normals,bases,boundaries,offsets,tree=self.patches
        count=min(4,len(centers))
        distance,ids=tree.query(queries,k=count,distance_upper_bound=2.)
        distance=np.asarray(distance).reshape(-1,count);ids=np.asarray(ids).reshape(-1,count)
        native=native_patch_backend()
        if native is not None:return native.patch_distances(queries,ids,self.patches)
        valid=np.isfinite(distance);ids=np.minimum(ids,len(centers)-1)
        delta=queries[:,None,:]-centers[ids]
        local=np.einsum('nki,nkij->nkj',delta,bases[ids])
        inside=np.all(np.einsum('nki,nkji->nkj',local,boundaries[ids])>=offsets[ids]-1e-7,axis=2)
        clearance=abs(np.sum(delta*normals[ids],axis=2))
        clearance[~(valid&inside)]=np.inf
        return np.min(clearance,axis=1)

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
        # Returns on two vertical walls can fit a horizontal plane at the
        # same laser elevation. Their convex hull spans empty air across the
        # road, so covariance/area alone cannot certify a ceiling. Require
        # actual local surface normals to agree with the fitted plane. Use
        # the original cloud, including neighbors below the overhead band.
        support=q[best]
        count=min(32,len(self.points))
        ids,distance=self.neighbors(support+origin,count)
        local=self.points[ids].astype(float)
        weights=(distance<=1.5**2).astype(float)
        samples=np.sum(weights,axis=1)
        mean=np.sum(local*weights[:,:,None],axis=1)/np.maximum(samples,1.)[:,None]
        delta=local-mean[:,None,:]
        covariance=np.einsum('nki,nkj,nk->nij',delta,delta,weights)/np.maximum(samples,1.)[:,None,None]
        values,vectors=np.linalg.eigh(covariance)
        normal=np.r_[-coeff[:2],1.];normal/=np.linalg.norm(normal)
        supported=((samples>=6)&(values[:,1]>1e-4)&
                   (values[:,0]<=.1*values[:,1])&
                   (abs(vectors[:,:,0]@normal)>=np.cos(np.pi/6.)))
        if np.sum(supported)<30 or np.mean(supported)<.3:return
        support=support[supported]
        coeff=np.linalg.lstsq(np.column_stack((support[:,:2],np.ones(len(support)))),support[:,2],rcond=None)[0]
        if np.linalg.norm(coeff[:2])>.65:return
        support=support[:,:2]
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
        native=native_patch_backend()
        if native is not None:return native.roof_queries(queries,self.roof,self.road_height)[0]
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
        native=native_patch_backend()
        if native is not None:return native.roof_queries(queries,self.roof,self.road_height,margin)[1]
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

    def distance(self,queries,limit=None):
        queries=np.ascontiguousarray(queries,dtype=np.float32).reshape(-1,3)
        if self.tree is None:return np.minimum(self.surface_distance(queries),self.face_distance(queries))
        _,distance=self.neighbors(queries,1,float('inf') if limit is None else limit)
        return np.minimum(np.minimum(np.minimum(np.sqrt(np.maximum(0.,distance[:,0])),self.surface_distance(queries)),
                          self.face_distance(queries)),self.patch_distance(queries))

    def neighbors(self,queries,count,limit=float('inf')):
        if cKDTree is not None:
            distance,indices=self.tree.query(queries,k=count,eps=0.,distance_upper_bound=limit)
            return np.asarray(indices).reshape(-1,count),np.asarray(distance).reshape(-1,count)**2
        return self.tree.knnSearch(np.ascontiguousarray(queries,dtype=np.float32),count,
                                   params=dict(checks=-1))
