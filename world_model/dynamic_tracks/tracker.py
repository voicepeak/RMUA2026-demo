"""Causal association of object candidates; no verified obstacle envelope."""
from dataclasses import dataclass, field, replace
import math
import threading

from interfaces.contracts import finite_vec3


@dataclass(frozen=True)
class ObjectTrack:
    track_id: int
    epoch: int
    center_ned: tuple
    half_extents_m: tuple
    velocity_ned_m_s: tuple
    observations: int
    stamp: float
    valid_until: float
    # Cluster geometry, motion and future occupancy are not yet certified.
    class_verified: bool = field(default=False,init=False)
    safety_certified: bool = field(default=False,init=False)
    control_eligible: bool = field(default=False,init=False)


class DynamicTracker:
    def __init__(self,epoch=0,max_gap_s=.5,max_speed_m_s=20.,association_margin_m=1.,capacity=128):
        if (isinstance(epoch,bool) or not isinstance(epoch,int) or epoch<0
                or not all(math.isfinite(x) and x>0 for x in (max_gap_s,max_speed_m_s,association_margin_m))
                or max_gap_s>2 or max_speed_m_s>50 or association_margin_m>5
                or isinstance(capacity,bool) or not isinstance(capacity,int) or not 1<=capacity<=256):
            raise ValueError('invalid dynamic tracker limits')
        self.epoch=epoch;self.max_gap_s=max_gap_s;self.max_speed_m_s=max_speed_m_s
        self.association_margin_m=association_margin_m;self.capacity=capacity
        self._tracks={};self._last_stamp=None;self._last_now=None;self._next_id=0;self._lock=threading.RLock()

    def reset(self,epoch):
        with self._lock:
            if isinstance(epoch,bool) or not isinstance(epoch,int) or epoch<=self.epoch:raise ValueError('epoch must increase')
            self.epoch=epoch;self._tracks={};self._last_stamp=None;self._last_now=None;self._next_id=0

    def ingest(self,frame,now,epoch):
        with self._lock:
            meta=frame.metadata
            if (epoch!=self.epoch or meta.frame_id!='world_ned' or not meta.current(now,epoch)
                    or (self._last_now is not None and now<self._last_now)
                    or (self._last_stamp is not None and meta.stamp<=self._last_stamp)
                    or not isinstance(frame.clusters,tuple) or len(frame.clusters)>256):
                raise ValueError('invalid, expired or duplicate cluster frame')
            for obj in frame.clusters:
                finite_vec3(obj.center_ned);finite_vec3(obj.half_extents_m)
                if any(v<=0 for v in obj.half_extents_m):raise ValueError('invalid cluster extents')
            live={k:t for k,t in self._tracks.items() if 0 < meta.stamp-t.stamp <= self.max_gap_s}
            edges=[]
            for i,obj in enumerate(frame.clusters):
                for track_id,t in live.items():
                    dt=meta.stamp-t.stamp
                    predicted=tuple(p+v*dt for p,v in zip(t.center_ned,t.velocity_ned_m_s))
                    if (math.dist(predicted,obj.center_ned)<=self.association_margin_m+self.max_speed_m_s*dt
                            and all(abs(a-b)<=max(2.,a,b) for a,b in zip(obj.half_extents_m,t.half_extents_m))):
                        edges.append((i,track_id))
            degree={k:sum(b==k for _,b in edges) for k in live};updated=dict(live);next_id=self._next_id
            matched=created=ambiguous=0
            for i,obj in enumerate(frame.clusters):
                possible=[k for a,k in edges if a==i]
                if len(possible)==1 and degree[possible[0]]==1:
                    t=updated[possible[0]];dt=meta.stamp-t.stamp
                    velocity=tuple((a-b)/dt for a,b in zip(obj.center_ned,t.center_ned))
                    if math.hypot(*velocity)>self.max_speed_m_s:
                        ambiguous+=1;continue
                    updated[t.track_id]=replace(t,center_ned=obj.center_ned,half_extents_m=obj.half_extents_m,
                        velocity_ned_m_s=velocity,observations=t.observations+1,stamp=meta.stamp,valid_until=meta.valid_until)
                    matched+=1
                elif not possible:
                    if len(updated)>=self.capacity:raise ValueError('dynamic track capacity exceeded')
                    updated[next_id]=ObjectTrack(next_id,epoch,obj.center_ned,obj.half_extents_m,(0.,0.,0.),1,meta.stamp,meta.valid_until)
                    next_id+=1;created+=1
                else:ambiguous+=1
            self._tracks=updated;self._next_id=next_id;self._last_stamp=meta.stamp;self._last_now=now
            return {'matched':matched,'created':created,'ambiguous':ambiguous}

    def snapshot(self,now,epoch):
        with self._lock:
            if epoch!=self.epoch or not math.isfinite(now) or now<0 or (self._last_now is not None and now<self._last_now):
                raise ValueError('dynamic track snapshot clock mismatch')
            self._last_now=now
            return tuple(t for _,t in sorted(self._tracks.items()) if t.stamp<=now<t.valid_until)


@dataclass(frozen=True)
class PredictedOccupancy:
    track_id: int
    epoch: int
    stamp: float
    center_ned: tuple
    half_extents_m: tuple
    source_stamp: float
    source_valid_until: float
    safety_certified: bool = field(default=False,init=False)
    control_eligible: bool = field(default=False,init=False)


def predict_candidate(track, now, stamp, epoch, *, position_error_m, velocity_error_m_s, acceleration_bound_m_s2):
    """Predict a provisional axis-aligned envelope under explicit error bounds.

    Source geometry and stated error bounds are not calibrated. This function
    does not establish that an object, including unseen portions, fits the box.
    """
    if (epoch!=track.epoch or not all(math.isfinite(x) for x in
            (now,stamp,position_error_m,velocity_error_m_s,acceleration_bound_m_s2))
            or min(position_error_m,velocity_error_m_s,acceleration_bound_m_s2)<0
            or not track.stamp<=now<track.valid_until or not now<=stamp<=now+.5
            or track.observations<2):
        raise ValueError('invalid prediction source, horizon or bounds')
    dt=stamp-track.stamp
    center=tuple(p+v*dt for p,v in zip(track.center_ned,track.velocity_ned_m_s))
    padding=position_error_m+velocity_error_m_s*dt+.5*acceleration_bound_m_s2*dt*dt
    extents=tuple(v+padding for v in track.half_extents_m)
    finite_vec3(center);finite_vec3(extents)
    return PredictedOccupancy(track.track_id,epoch,stamp,center,extents,track.stamp,track.valid_until)
