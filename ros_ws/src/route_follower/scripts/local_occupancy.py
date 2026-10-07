#!/usr/bin/env python3
"""World-aligned rolling LiDAR occupancy with explicit unknown space.

Ray evidence uses the sensor origin at exposure, never a later planning pose.
Static and moving-return evidence are separate; removing a moving hit does not
invent free space. A snapshot's freshness is evaluated at *planning/execution
now*, not at the future trajectory sample time (dynamic prediction owns that).
"""
from dataclasses import dataclass
import numpy as np

UNKNOWN=0
FREE=1
OCCUPIED=2


@dataclass(frozen=True)
class OccupancyConfig:
    resolution: float = .5
    forward: float = 25.
    backward: float = 2.
    lateral: float = 4.
    vertical_min: float = -3.
    vertical_max: float = 3.
    free_ttl: float = .5
    occupied_ttl: float = 1.
    dynamic_ttl: float = .75
    max_rays: int = 8192
    max_voxels: int = 1_000_000
    min_dynamic_confidence: float = .25
    min_dynamic_speed: float = .3

    def __post_init__(self):
        if any(not np.isfinite(v) for v in vars(self).values()):
            raise ValueError('Occupancy parameters must be finite')
        if any(getattr(self,k)<=0 for k in ('resolution','forward','lateral','free_ttl',
                                           'occupied_ttl','dynamic_ttl')) or self.backward<0:
            raise ValueError('Invalid occupancy extent/resolution/lifetime')
        if self.vertical_max<=self.vertical_min:raise ValueError('Invalid vertical bounds')
        for k in ('max_rays','max_voxels'):
            value=getattr(self,k)
            if isinstance(value,bool) or not isinstance(value,(int,np.integer)) or value<1:
                raise ValueError(k+' must be a positive integer')
        if not 0.<=self.min_dynamic_confidence<=1. or self.min_dynamic_speed<0:
            raise ValueError('Invalid dynamic ownership threshold')


def _readonly(value):
    value=np.asarray(value).copy();value.setflags(write=False);return value


def _inside(points,center,forward,config):
    relative=points-center
    along=relative[...,:2]@forward
    cross=relative[...,:2]@np.array([-forward[1],forward[0]])
    return ((along>=-config.backward)&(along<config.forward)&
            (cross>=-config.lateral)&(cross<config.lateral)&
            (relative[...,2]>=config.vertical_min)&(relative[...,2]<config.vertical_max))


def owned_dynamic_returns(point_count,obstacles,stamp,config=None):
    """Only credible CURRENT moving returns leave the static hit layer.

    All obstacle predictions (including low-speed/coasting/tentative tracks)
    must still be checked by the later trajectory checker. Conflicting return
    ownership falls back to static; voxel-level mixing is handled by the map.
    """
    config=OccupancyConfig() if config is None else config
    if (not np.isfinite(stamp) or isinstance(point_count,bool) or
            not isinstance(point_count,(int,np.integer)) or point_count<0):
        raise ValueError('Invalid return count/timestamp')
    owners=np.zeros(point_count,dtype=np.int64);conflict=np.zeros(point_count,dtype=bool)
    for track in obstacles:
        indices=np.asarray(track.point_indices)
        if indices.ndim!=1 or not np.issubdtype(indices.dtype,np.integer) or np.any(indices<0) or np.any(indices>=point_count):
            raise ValueError('Track return ownership is out of bounds')
        if abs(track.timestamp-stamp)>1e-9:raise ValueError('Track and cloud snapshots differ in time')
        if (track.status!='CONFIRMED' or track.age>0. or not track.motion_confirmed or
                track.truncated or track.confidence<config.min_dynamic_confidence or
                np.linalg.norm(track.velocity)<config.min_dynamic_speed):continue
        if track.track_id<=0:raise ValueError('Track IDs must be positive')
        conflict[indices]|=(owners[indices]!=0)&(owners[indices]!=track.track_id)
        owners[indices]=track.track_id
    owners[conflict]=0
    return owners


@dataclass(frozen=True)
class OccupancySnapshot:
    stamp: object
    evaluated_at: float
    epoch: int
    version: int
    valid: bool
    config: OccupancyConfig
    center: np.ndarray
    forward: np.ndarray
    cell_origin: np.ndarray
    states: np.ndarray
    static_states: np.ndarray
    dynamic_owners: np.ndarray

    def query(self,points,layer='observed'):
        """Return tri-state values; outside/unobserved/nonfinite is UNKNOWN."""
        if layer not in ('observed','static'):raise ValueError('Unknown occupancy layer')
        points=np.asarray(points,dtype=float)
        if points.ndim!=2 or points.shape[1]!=3:raise ValueError('Queries must be Nx3')
        result=np.full(len(points),UNKNOWN,dtype=np.uint8)
        usable=np.all(np.isfinite(points),axis=1)&np.all(abs(points)<1e9,axis=1)
        ids=np.flatnonzero(usable)
        if not len(ids) or not self.valid:return result
        selected=points[ids]
        cells=np.floor(selected/self.config.resolution).astype(np.int64)-self.cell_origin
        inside=_inside(selected,self.center,self.forward,self.config)&np.all(cells>=0,axis=1)&np.all(cells<self.states.shape,axis=1)
        data=self.states if layer=='observed' else self.static_states
        result[ids[inside]]=data[tuple(cells[inside].T)]
        return result

    def query_boxes(self,lows,highs,layer='observed'):
        """Conservative complete AABB footprint queries, not just centers."""
        if layer not in ('observed','static'):raise ValueError('Unknown occupancy layer')
        lows=np.asarray(lows,dtype=float);highs=np.asarray(highs,dtype=float)
        if (lows.ndim!=2 or lows.shape[1]!=3 or highs.shape!=lows.shape or
                not np.all(np.isfinite(lows)) or not np.all(np.isfinite(highs)) or
                np.any(abs(lows)>=1e9) or np.any(abs(highs)>=1e9) or np.any(highs<lows)):
            raise ValueError('Invalid footprint boxes')
        result=np.full(len(lows),UNKNOWN,dtype=np.uint8)
        if not self.valid:return result
        data=self.states if layer=='observed' else self.static_states
        corners=np.array([[x,y,z] for x in (0,1) for y in (0,1) for z in (0,1)])
        for i,(low,high) in enumerate(zip(lows,highs)):
            first=np.floor(low/self.config.resolution).astype(np.int64)-self.cell_origin
            last=np.floor(high/self.config.resolution).astype(np.int64)-self.cell_origin
            if np.any(last<0) or np.any(first>=data.shape):continue
            a=np.maximum(first,0);b=np.minimum(last,np.asarray(data.shape)-1)
            values=data[tuple(slice(int(x),int(y)+1) for x,y in zip(a,b))]
            if np.any(values==OCCUPIED):result[i]=OCCUPIED
            elif (np.all(values==FREE) and np.all(first>=0) and np.all(last<data.shape) and
                  np.all(_inside(low+corners*(high-low),self.center,self.forward,self.config))):
                result[i]=FREE
        return result

    def voxel_centers(self,state,layer='observed',max_count=4096):
        if state not in (UNKNOWN,FREE,OCCUPIED) or layer not in ('observed','static') or max_count<1:
            raise ValueError('Invalid visualization query')
        data=self.states if layer=='observed' else self.static_states
        cells=np.argwhere(data==state)
        if len(cells)>max_count:cells=cells[np.linspace(0,len(cells)-1,max_count,dtype=int)]
        world=(cells+self.cell_origin+.5)*self.config.resolution
        return world[_inside(world,self.center,self.forward,self.config)]


@dataclass(frozen=True)
class OccupancyEvidence:
    """Immutable ray clocks; evaluate exact individual TTLs without owner lock."""
    stamp: object
    scan_valid: bool
    epoch: int
    version: int
    config: OccupancyConfig
    center: np.ndarray
    forward: np.ndarray
    cell_origin: np.ndarray
    free: np.ndarray
    static: np.ndarray
    dynamic: np.ndarray
    owner: np.ndarray

    def snapshot(self,now):
        if not np.isfinite(now) or (self.stamp is not None and now<self.stamp-1e-9):
            raise ValueError('Snapshot evaluation time precedes cloud')
        valid=bool(self.scan_valid and self.stamp is not None and now-self.stamp<self.config.free_ttl)
        static=np.full(self.free.shape,UNKNOWN,dtype=np.uint8);observed=static.copy()
        if valid:
            static[now-self.free<self.config.free_ttl]=FREE
            static[now-self.static<self.config.occupied_ttl]=OCCUPIED
            observed=static.copy();observed[now-self.dynamic<self.config.dynamic_ttl]=OCCUPIED
        owners=self.owner.copy();owners[now-self.dynamic>=self.config.dynamic_ttl]=-1
        if not valid:owners.fill(-1)
        return OccupancySnapshot(self.stamp,float(now),self.epoch,self.version,valid,self.config,
            self.center,self.forward,self.cell_origin,_readonly(observed),_readonly(static),_readonly(owners))


class LocalOccupancy:
    def __init__(self,config=None):
        self.config=OccupancyConfig() if config is None else config
        self.epoch=0;self.version=0;self.source_epoch=None
        self.center=np.zeros(3);self.forward=np.array([1.,0.]);self.cell_origin=np.zeros(3,dtype=np.int64)
        self._allocate((1,1,1));self.stamp=None;self.scan_valid=False;self.last_info={}

    def _allocate(self,shape):
        self._free=np.full(shape,-np.inf);self._static=np.full(shape,-np.inf)
        self._dynamic=np.full(shape,-np.inf);self._owner=np.full(shape,-1,dtype=np.int64)

    def reset(self):
        self.epoch+=1;self.version+=1;self.source_epoch=None;self.stamp=None
        self.scan_valid=False;self._allocate(self._free.shape);self.last_info={}

    def _geometry(self,center,forward):
        center=np.asarray(center,dtype=float);forward=np.asarray(forward,dtype=float)
        if (center.shape!=(3,) or forward.shape!=(2,) or not np.all(np.isfinite(np.r_[center,forward])) or
                np.any(abs(center)>=1e9) or not np.isfinite(np.linalg.norm(forward)) or
                np.linalg.norm(forward)<1e-9):
            raise ValueError('Invalid rolling-map geometry')
        forward=forward/np.linalg.norm(forward);side=np.array([-forward[1],forward[0]])
        corners=np.array([center+np.r_[a*forward+b*side,z] for a in (-self.config.backward,self.config.forward)
                          for b in (-self.config.lateral,self.config.lateral)
                          for z in (self.config.vertical_min,self.config.vertical_max)])
        scaled=corners/self.config.resolution
        if not np.all(np.isfinite(scaled)) or np.any(abs(scaled)>=2**62):
            raise ValueError('Rolling-map cell coordinates exceed integer bounds')
        origin=np.floor(scaled.min(axis=0)).astype(np.int64)
        shape=np.floor(scaled.max(axis=0)).astype(np.int64)-origin+1
        if np.prod(shape.astype(float))>self.config.max_voxels:raise ValueError('Rolling map exceeds voxel budget')
        return center,forward,origin,shape

    def recenter(self,center,forward):
        center,forward,origin,shape=self._geometry(center,forward)
        if np.array_equal(center,self.center) and np.array_equal(forward,self.forward) and np.array_equal(shape,self._free.shape):return
        old_origin=self.cell_origin.copy();old_shape=np.asarray(self._free.shape)
        previous=(self._free,self._static,self._dynamic,self._owner)
        self._allocate(tuple(shape))
        low=np.maximum(origin,old_origin);high=np.minimum(origin+shape,old_origin+old_shape)
        if np.all(high>low):
            src=tuple(slice(int(a),int(b)) for a,b in zip(low-old_origin,high-old_origin))
            dst=tuple(slice(int(a),int(b)) for a,b in zip(low-origin,high-origin))
            for old,new in zip(previous,(self._free,self._static,self._dynamic,self._owner)):new[dst]=old[src]
        self.center=center.copy();self.forward=forward.copy();self.cell_origin=origin
        cells=np.indices(tuple(shape)).reshape(3,-1).T
        active=_inside((cells+origin+.5)*self.config.resolution,center,forward,self.config).reshape(tuple(shape))
        for data in (self._free,self._static,self._dynamic):data[~active]=-np.inf
        self._owner[~active]=-1;self.version+=1

    def _cast(self,endpoints,sensor_origin,frame_hits):
        """Vectorized exact voxel DDA; ties step all axes, not false free sides."""
        shape=np.asarray(self._free.shape);resolution=self.config.resolution
        lower=self.cell_origin*resolution;upper=(self.cell_origin+shape)*resolution
        directions=endpoints-sensor_origin
        nonzero=abs(directions)>1e-15
        inverse=np.zeros_like(directions);np.divide(1.,directions,out=inverse,where=nonzero)
        near=(lower-sensor_origin)*inverse;far=(upper-sensor_origin)*inverse
        entry_axes=np.minimum(near,far);exit_axes=np.maximum(near,far)
        entry_axes[~nonzero]=-np.inf;exit_axes[~nonzero]=np.inf
        parallel_out=np.any((~nonzero)&((sensor_origin<lower)|(sensor_origin>=upper)),axis=1)
        entry=np.maximum(0.,entry_axes.max(axis=1));exit=np.minimum(1.,exit_axes.min(axis=1))
        active=(entry<exit-1e-12)&~parallel_out
        start=sensor_origin+directions*entry[:,None]
        interior=np.nextafter(start,np.where(directions>0,np.inf,np.where(directions<0,-np.inf,start)))
        cells=np.floor(interior/resolution).astype(np.int64)-self.cell_origin
        step=np.sign(directions).astype(np.int64)
        boundary=(cells+self.cell_origin+(step>0))*resolution
        crossing=(boundary-sensor_origin)*inverse;crossing[~nonzero]=np.inf
        stride=np.full_like(directions,np.inf);np.divide(resolution,abs(directions),out=stride,where=nonzero)
        free=np.zeros(self._free.shape,dtype=bool);hit_flat=frame_hits.ravel();free_flat=free.ravel()
        cast_steps=0
        for _ in range(int(shape.sum())+3):
            active&=np.all(cells>=0,axis=1)&np.all(cells<shape,axis=1)
            ids=np.flatnonzero(active)
            if not len(ids):break
            flat=np.ravel_multi_index(cells[ids].T,tuple(shape))
            blocked=hit_flat[flat]
            active[ids[blocked]]=False;ids=ids[~blocked];flat=flat[~blocked]
            if not len(ids):continue
            nearest=crossing[ids].min(axis=1)
            length=np.minimum(nearest,exit[ids])-entry[ids]
            centers=(cells[ids]+self.cell_origin+.5)*resolution
            observed=(length>1e-12)&_inside(centers,self.center,self.forward,self.config)
            free_flat[flat[observed]]=True;cast_steps+=len(ids)
            continuing=nearest<exit[ids]-1e-12
            active[ids[~continuing]]=False
            ids=ids[continuing];nearest=nearest[continuing]
            ties=abs(crossing[ids]-nearest[:,None])<=1e-12
            cells[ids]+=step[ids]*ties
            crossing[ids]=np.where(ties,crossing[ids]+stride[ids],crossing[ids])
            entry[ids]=nearest
        else:
            if np.any(active):raise RuntimeError('DDA exceeded bounded voxel traversal')
        return free,cast_steps

    def update(self,points,sensor_origin,stamp,center=None,forward=(1.,0.),dynamic_ids=None,source_epoch=None):
        points=np.asarray(points,dtype=float);sensor_origin=np.asarray(sensor_origin,dtype=float)
        if (points.ndim!=2 or points.shape[1]!=3 or sensor_origin.shape!=(3,) or
                not np.isfinite(stamp) or not np.all(np.isfinite(sensor_origin)) or np.any(abs(sensor_origin)>=1e9)):
            raise ValueError('Invalid cloud timestamp/origin/shape')
        owners=np.zeros(len(points),dtype=np.int64) if dynamic_ids is None else np.asarray(dynamic_ids)
        if owners.shape!=(len(points),) or not np.issubdtype(owners.dtype,np.integer) or np.any(owners<0):
            raise ValueError('Invalid dynamic return IDs')
        center=sensor_origin if center is None else np.asarray(center,dtype=float)
        # Reject invalid new geometry before a clock/epoch reset can erase data.
        self._geometry(center,forward)
        if self.stamp is not None and stamp==self.stamp and source_epoch==self.source_epoch:return self.last_info.copy()
        if ((self.stamp is not None and stamp<self.stamp) or
                (self.source_epoch is not None and source_epoch!=self.source_epoch)):self.reset()
        self.recenter(center,forward);self.source_epoch=source_epoch
        valid=(np.all(np.isfinite(points),axis=1)&np.all(abs(points)<1e9,axis=1)&
               (np.sum((points-sensor_origin)**2,axis=1)>1e-8))
        points=points[valid];owners=owners[valid]
        self.stamp=float(stamp);self.scan_valid=bool(len(points));self.version+=1
        hit_points=_inside(points,self.center,self.forward,self.config)
        cells=np.floor(points[hit_points]/self.config.resolution).astype(np.int64)-self.cell_origin
        flat=np.ravel_multi_index(cells.T,self._free.shape) if len(cells) else np.empty(0,dtype=np.int64)
        hit_owners=owners[hit_points];frame_hits=np.zeros(self._free.shape,dtype=bool)
        frame_hits.ravel()[flat]=True
        # Every endpoint is retained even when free-space ray processing is capped.
        static_flat=np.unique(flat[hit_owners==0]);dynamic_mask=hit_owners>0
        dynamic_flat=np.unique(flat[dynamic_mask])
        count=len(points)
        selected=points if count<=self.config.max_rays else points[np.linspace(0,count-1,self.config.max_rays,dtype=int)]
        free,steps=self._cast(selected,sensor_origin,frame_hits) if len(selected) else (np.zeros(self._free.shape,dtype=bool),0)
        free&=~frame_hits
        self._free[free]=stamp;self._static[free]=-np.inf;self._dynamic[free]=-np.inf;self._owner[free]=-1
        self._static.ravel()[static_flat]=stamp
        self._dynamic.ravel()[dynamic_flat]=stamp
        if len(dynamic_flat):
            minimum=np.full(self._free.size,np.iinfo(np.int64).max,dtype=np.int64)
            maximum=np.zeros(self._free.size,dtype=np.int64)
            np.minimum.at(minimum,flat[dynamic_mask],hit_owners[dynamic_mask])
            np.maximum.at(maximum,flat[dynamic_mask],hit_owners[dynamic_mask])
            self._owner.ravel()[dynamic_flat]=np.where(minimum[dynamic_flat]==maximum[dynamic_flat],maximum[dynamic_flat],-2)
        self.last_info=dict(input_returns=int(len(valid)),valid_returns=count,rays=int(len(selected)),
                            rays_skipped=count-len(selected),dda_steps=steps,
                            free_voxels_observed=int(free.sum()),hit_voxels=int(frame_hits.sum()),
                            static_hit_voxels=len(static_flat),dynamic_hit_voxels=len(dynamic_flat),
                            mixed_hit_voxels=len(np.intersect1d(static_flat,dynamic_flat)))
        return self.last_info.copy()

    def snapshot(self,now=None):
        now=(self.stamp if self.stamp is not None else 0.) if now is None else float(now)
        if not np.isfinite(now) or (self.stamp is not None and now<self.stamp-1e-9):
            raise ValueError('Snapshot evaluation time precedes cloud')
        valid=bool(self.scan_valid and self.stamp is not None and now-self.stamp<self.config.free_ttl)
        static=np.full(self._free.shape,UNKNOWN,dtype=np.uint8);observed=static.copy()
        if valid:
            free=(now-self._free<self.config.free_ttl)
            static[free]=FREE;static[now-self._static<self.config.occupied_ttl]=OCCUPIED
            observed=static.copy();observed[now-self._dynamic<self.config.dynamic_ttl]=OCCUPIED
        owners=self._owner.copy();owners[now-self._dynamic>=self.config.dynamic_ttl]=-1
        if not valid:owners.fill(-1)
        return OccupancySnapshot(self.stamp,now,self.epoch,self.version,valid,self.config,
            _readonly(self.center),_readonly(self.forward),_readonly(self.cell_origin),
            _readonly(observed),_readonly(static),_readonly(owners))

    def evidence(self):
        return OccupancyEvidence(self.stamp,self.scan_valid,self.epoch,self.version,self.config,
            _readonly(self.center),_readonly(self.forward),_readonly(self.cell_origin),
            _readonly(self._free),_readonly(self._static),_readonly(self._dynamic),_readonly(self._owner))

    def summary(self,now=None):
        snapshot=self.snapshot(now)
        return dict(self.last_info,stamp=self.stamp,evaluated_at=snapshot.evaluated_at,
                    epoch=self.epoch,version=self.version,valid=snapshot.valid,
                    shape=list(snapshot.states.shape),voxel_count=int(snapshot.states.size),
                    free=int(np.sum(snapshot.states==FREE)),occupied=int(np.sum(snapshot.states==OCCUPIED)),
                    unknown=int(np.sum(snapshot.states==UNKNOWN)),
                    static_occupied=int(np.sum(snapshot.static_states==OCCUPIED)),
                    dynamic_voxels=int(np.sum(snapshot.dynamic_owners!=-1)))
