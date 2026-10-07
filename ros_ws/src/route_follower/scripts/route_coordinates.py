#!/usr/bin/env python3
"""Frozen local route frame and shared road/height prior; no live closures."""
from dataclasses import dataclass,field
import numpy as np
from trajectory_types import readonly


@dataclass(frozen=True)
class RouteCoordinates:
    stations: np.ndarray
    points: np.ndarray
    lateral_limit: float = 2.25
    vertical_limit: float = 1.25
    floor_limits: object = None
    _segments: np.ndarray = field(init=False,repr=False)
    _length2: np.ndarray = field(init=False,repr=False)
    _straight: bool = field(init=False,repr=False)
    _arc: np.ndarray = field(init=False,repr=False)

    def __post_init__(self):
        stations=readonly(self.stations);points=readonly(self.points)
        if (stations.ndim!=1 or len(stations)<2 or points.shape!=(len(stations),3) or
                not np.all(np.isfinite(stations)) or not np.all(np.isfinite(points)) or
                np.any(np.diff(stations)<=0) or not np.isfinite(self.lateral_limit) or self.lateral_limit<=0 or
                not np.isfinite(self.vertical_limit) or self.vertical_limit<=0):raise ValueError('Invalid route samples/bounds')
        segments=np.diff(points[:,:2],axis=0);length2=np.sum(segments**2,axis=1)
        if np.any(length2<1e-8):raise ValueError('Route has a zero-length horizontal segment')
        floor=np.full(len(stations),np.inf) if self.floor_limits is None else readonly(self.floor_limits)
        if np.shape(floor)!=stations.shape or np.any(np.isnan(floor)) or np.any(np.isneginf(floor)):
            raise ValueError('Invalid NED floor limits')
        object.__setattr__(self,'stations',stations);object.__setattr__(self,'points',points)
        object.__setattr__(self,'floor_limits',readonly(floor))
        object.__setattr__(self,'_segments',readonly(segments));object.__setattr__(self,'_length2',readonly(length2))
        axis=segments[0]/np.sqrt(length2[0])
        object.__setattr__(self,'_straight',bool(np.all(abs(segments/np.sqrt(length2)[:,None]-axis)<1e-12)))
        object.__setattr__(self,'_arc',readonly(np.r_[0.,np.cumsum(np.sqrt(length2))]))

    def center(self,stations):
        stations=np.asarray(stations,dtype=float)
        if not np.all(np.isfinite(stations)):raise ValueError('Invalid route coordinate')
        return np.stack([np.interp(stations,self.stations,self.points[:,axis]) for axis in range(3)],axis=-1)

    def frame(self,stations):
        stations=np.asarray(stations,dtype=float)
        indices=np.clip(np.searchsorted(self.stations,stations,side='right')-1,0,len(self._segments)-1)
        tangent=self._segments[indices]/np.sqrt(self._length2[indices])[...,None]
        side=np.stack([-tangent[...,1],tangent[...,0]],axis=-1)
        return tangent,side

    def world(self,stations,lateral=0.,height=0.):
        points=self.center(stations).copy();tangent,side=self.frame(stations)
        points[...,:2]+=np.asarray(lateral)[...,None]*side;points[...,2]+=height
        return points

    def project(self,points):
        points=np.asarray(points,dtype=float)
        if points.ndim!=2 or points.shape[1]!=3 or not np.all(np.isfinite(points)):
            raise ValueError('Projection must be finite Nx3')
        if self._straight:
            axis=self._segments[0]/np.sqrt(self._length2[0]);side=np.array([-axis[1],axis[0]])
            delta=points[:,:2]-self.points[0,:2];along=delta@axis
            indices=np.clip(np.searchsorted(self._arc,along,side='right')-1,0,len(self._segments)-1)
            fraction=(along-self._arc[indices])/np.sqrt(self._length2[indices])
            station=self.stations[indices]+fraction*np.diff(self.stations)[indices]
            height=self.points[indices,2]+fraction*np.diff(self.points[:,2])[indices]
            return np.column_stack([station,delta@side,points[:,2]-height])
        delta=points[:,None,:2]-self.points[None,:-1,:2]
        fraction=np.sum(delta*self._segments,axis=2)/self._length2
        clipped=np.clip(fraction,0.,1.)
        residual=delta-clipped[:,:,None]*self._segments
        nearest=np.argmin(np.sum(residual**2,axis=2),axis=1);ids=np.arange(len(points))
        f=clipped[ids,nearest]
        # Preserve longitudinal out-of-window errors instead of clamping safe.
        f=np.where((nearest==0)&(fraction[ids,nearest]<0.),fraction[ids,nearest],f)
        f=np.where((nearest==len(self._segments)-1)&(fraction[ids,nearest]>1.),fraction[ids,nearest],f)
        station=self.stations[nearest]+f*np.diff(self.stations)[nearest]
        base=self.points[nearest]+f[:,None]*np.diff(self.points,axis=0)[nearest]
        tangent=self._segments[nearest]/np.sqrt(self._length2[nearest])[:,None]
        side=np.column_stack([-tangent[:,1],tangent[:,0]])
        lateral=np.sum((points[:,:2]-base[:,:2])*side,axis=1)
        return np.column_stack([station,lateral,points[:,2]-base[:,2]])

    def violations(self,points):
        coordinates=self.project(points);s,y,z=coordinates.T
        floor=np.interp(s,self.stations,self.floor_limits)
        return ((s<self.stations[0]-1e-8)|(s>self.stations[-1]+1e-8)|
                (abs(y)>self.lateral_limit+1e-8)|(abs(z)>self.vertical_limit+1e-8)|
                (np.asarray(points)[:,2]>floor+1e-8))


@dataclass(frozen=True)
class HeightProfile:
    route: RouteCoordinates
    lateral: float = 0.
    height: float = 0.
    height_gain: float = 1.
    lateral_gain: float = 0.
    lateral_speed_limit: float = 1.5
    lateral_damping: float = 2.
    path: np.ndarray = field(init=False,repr=False)
    _segments: np.ndarray = field(init=False,repr=False)
    _length2: np.ndarray = field(init=False,repr=False)
    _arc: np.ndarray = field(init=False,repr=False)

    def __post_init__(self):
        if (not np.all(np.isfinite([self.lateral,self.height,self.height_gain,self.lateral_gain,self.lateral_speed_limit,self.lateral_damping])) or
                self.height_gain<=0 or self.lateral_gain<0 or self.lateral_speed_limit<0 or self.lateral_damping<0):
            raise ValueError('Invalid height profile')
        path=readonly(self.route.world(self.route.stations,self.lateral,self.height))
        if np.any(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1)<1e-4):raise ValueError('Offset height path has a degenerate segment')
        object.__setattr__(self,'path',path)
        segments=np.diff(path[:,:2],axis=0);length2=np.sum(segments**2,axis=1)
        object.__setattr__(self,'_segments',readonly(segments));object.__setattr__(self,'_length2',readonly(length2))
        object.__setattr__(self,'_arc',readonly(np.r_[0.,np.cumsum(np.sqrt(length2))]))

    def xy_target(self,target,positions,velocities):
        """Frozen offset tracking policy used identically in rollout/execution."""
        if self.lateral_gain==0.:return np.broadcast_to(target[:2],(len(positions),2)).copy()
        coordinates=self.route.project(positions);tangent,side=self.route.frame(coordinates[:,0])
        along=float(np.linalg.norm(target[:2]))
        lateral=np.clip(self.lateral_gain*(self.lateral-coordinates[:,1])-
                        self.lateral_damping*np.sum(velocities[:,:2]*side,axis=1),-self.lateral_speed_limit,self.lateral_speed_limit)
        return along*tangent+lateral[:,None]*side

    def __call__(self,positions,velocities):
        # Same grade feed-forward and +/-2m smoothing as the legacy stop profile.
        positions=np.asarray(positions);velocities=np.asarray(velocities)
        segments=self._segments;length2=self._length2;arc=self._arc
        delta=positions[:,None,:2]-self.path[None,:-1,:2]
        fraction=np.clip(np.sum(delta*segments,axis=2)/length2,0.,1.)
        nearest=np.argmin(np.sum((delta-fraction[:,:,None]*segments)**2,axis=2),axis=1)
        f=fraction[np.arange(len(positions)),nearest]
        reference=self.path[nearest,2]+f*np.diff(self.path[:,2])[nearest]
        along=arc[nearest]+f*np.sqrt(length2[nearest]);lo=np.maximum(0.,along-2.);hi=np.minimum(arc[-1],along+2.)
        direction=np.column_stack([np.interp(hi,arc,self.path[:,axis])-np.interp(lo,arc,self.path[:,axis]) for axis in (0,1)])
        direction/=np.maximum(1e-8,np.linalg.norm(direction,axis=1))[:,None]
        slope=(np.interp(hi,arc,self.path[:,2])-np.interp(lo,arc,self.path[:,2]))/np.maximum(1e-8,hi-lo)
        return np.clip(np.sum(velocities[:,:2]*direction,axis=1)*slope+
                       self.height_gain*(reference-positions[:,2]),-4.,4.5)
