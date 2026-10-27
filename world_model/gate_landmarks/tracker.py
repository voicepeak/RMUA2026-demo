"""Bounded association of nominal gate candidates in estimated world coordinates."""
from dataclasses import dataclass, field, replace
import math
import threading

import numpy as np

from estimation.geometry import rotation
from interfaces.contracts import finite_vec3


@dataclass(frozen=True)
class GateTrack:
    track_id: int
    epoch: int
    center_world_ned: tuple
    width_m: float
    height_m: float
    first_stamp: float
    last_stamp: float
    last_valid_until: float
    observations: int
    # Repeated matches are association evidence only, not geometric certification.
    aperture_verified: bool = field(default=False, init=False)
    metric_calibration_verified: bool = field(default=False, init=False)
    control_eligible: bool = field(default=False, init=False)


class GateTracker:
    """Nearest unambiguous associations; historical tracks never renew source leases."""
    def __init__(self, epoch=0, max_gap_s=4., max_distance_m=2., size_tolerance_m=2., capacity=64):
        if (isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 0
                or not all(math.isfinite(x) and x > 0 for x in (max_gap_s,max_distance_m,size_tolerance_m))
                or max_gap_s > 10 or max_distance_m > 10 or size_tolerance_m > 10
                or isinstance(capacity,bool) or not isinstance(capacity,int) or not 1 <= capacity <= 256):
            raise ValueError('invalid gate tracking limits')
        self.epoch=epoch; self.max_gap_s=max_gap_s; self.max_distance_m=max_distance_m
        self.size_tolerance_m=size_tolerance_m; self.capacity=capacity
        self._tracks={}; self._last_stamp=None; self._last_now=None; self._next_id=0
        self._lock=threading.RLock()

    def reset(self, epoch):
        with self._lock:
            if isinstance(epoch,bool) or not isinstance(epoch,int) or epoch <= self.epoch:
                raise ValueError('gate epoch must increase')
            self.epoch=epoch; self._tracks={}; self._last_stamp=None; self._last_now=None; self._next_id=0

    def ingest(self, candidates, pose, now, epoch):
        """Associate one frame atomically, using a same-stamp historical pose.

        An empty frame does not refresh any existing track. Each observation may
        match at most one track and vice versa; ambiguous associations are held.
        """
        with self._lock:
            if (epoch != self.epoch or pose.epoch != epoch or pose.frame_id != 'world_ned'
                    or not math.isfinite(now) or now < pose.stamp or pose.bracket_stamps[1] > now
                    or (self._last_now is not None and now < self._last_now)
                    or (self._last_stamp is not None and pose.stamp <= self._last_stamp)
                    or pose.position_correction_residual_m > .1 or pose.rotation_correction_residual_rad > .05
                    or not isinstance(candidates,tuple) or len(candidates)>32):
                raise ValueError('invalid gate frame, epoch, history or clock')
            prepared=[]
            for candidate in candidates:
                meta=candidate.metadata
                finite_vec3(candidate.center_body_frd_m)
                if (meta.frame_id!='body_frd' or meta.stamp!=pose.stamp or not meta.current(now,epoch)
                        or not all(math.isfinite(x) and x>0 for x in (candidate.nominal_width_m,candidate.nominal_height_m))):
                    raise ValueError('gate candidate source invalid')
                point=np.asarray(pose.position_ned)+rotation(pose.orientation_wxyz)@np.asarray(candidate.center_body_frd_m)
                if not np.isfinite(point).all():raise ValueError('nonfinite world gate')
                prepared.append((candidate,tuple(float(x) for x in point)))
            live={k:v for k,v in self._tracks.items() if now-v.last_stamp <= self.max_gap_s}
            # Build the whole bipartite compatibility graph. Only degree-1 on
            # both sides is associated; this refuses accidental many-to-one.
            edges=[]
            for index,(candidate,point) in enumerate(prepared):
                for track_id,track in live.items():
                    if (math.dist(point,track.center_world_ned)<=self.max_distance_m
                            and abs(candidate.nominal_width_m-track.width_m)<=self.size_tolerance_m
                            and abs(candidate.nominal_height_m-track.height_m)<=self.size_tolerance_m):
                        edges.append((index,track_id))
            degrees_track={k:sum(b==k for _,b in edges) for k in live}
            updates=dict(live); next_id=self._next_id; matched=created=ambiguous=0
            for i,(candidate,point) in enumerate(prepared):
                possible=[k for a,k in edges if a==i]
                if len(possible)==1 and degrees_track[possible[0]]==1:
                    t=updates[possible[0]]; n=t.observations+1
                    mean=tuple((a*t.observations+b)/n for a,b in zip(t.center_world_ned,point))
                    updates[t.track_id]=replace(t,center_world_ned=mean,
                        width_m=(t.width_m*t.observations+candidate.nominal_width_m)/n,
                        height_m=(t.height_m*t.observations+candidate.nominal_height_m)/n,
                        last_stamp=pose.stamp,last_valid_until=candidate.metadata.valid_until,observations=n)
                    matched+=1
                elif not possible:
                    if len(updates)>=self.capacity:raise ValueError('gate track capacity exceeded')
                    updates[next_id]=GateTrack(next_id,epoch,point,candidate.nominal_width_m,
                        candidate.nominal_height_m,pose.stamp,pose.stamp,candidate.metadata.valid_until,1)
                    next_id+=1;created+=1
                else:ambiguous+=1
            self._tracks=updates; self._next_id=next_id; self._last_stamp=pose.stamp;self._last_now=now
            return {'matched':matched,'created':created,'ambiguous':ambiguous}

    def snapshot(self, now, epoch):
        with self._lock:
            if (epoch!=self.epoch or not math.isfinite(now) or now<0
                    or (self._last_now is not None and now<self._last_now)):
                raise ValueError('gate snapshot clock or epoch mismatch')
            self._last_now=now
            return tuple(t for _,t in sorted(self._tracks.items())
                         if t.last_stamp<=now<t.last_valid_until and now-t.last_stamp<=self.max_gap_s)
