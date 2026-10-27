"""Continuous candidate-obstacle veto; does not certify dynamic clearance."""
from dataclasses import dataclass, field
import math
import sys

from interfaces.contracts import Metadata, finite_vec3
from planning.trajectory_validator.static_sweep import SweepBounds, _contact_interval


@dataclass(frozen=True)
class DynamicScene:
    metadata: Metadata
    tracks: tuple
    coverage_verified: bool = field(default=False, init=False)

    def __post_init__(self):
        if (not isinstance(self.metadata,Metadata) or not isinstance(self.tracks,tuple) or len(self.tracks)>128
                or self.metadata.frame_id!='world_ned'):
            raise ValueError('invalid dynamic candidate snapshot')
        ids=[track.track_id for track in self.tracks]
        if len(set(ids))!=len(ids):raise ValueError('duplicate dynamic track ID')


@dataclass(frozen=True)
class DynamicBounds:
    position_error_m: float
    velocity_error_m_s: float
    acceleration_bound_m_s2: float
    single_observation_speed_bound_m_s: float
    max_horizon_s: float = .5
    max_pair_tests: int = 32768

    def __post_init__(self):
        if (not all(math.isfinite(x) and x>=0 for x in (self.position_error_m,self.velocity_error_m_s,
                self.acceleration_bound_m_s2,self.single_observation_speed_bound_m_s))
                or not math.isfinite(self.max_horizon_s) or not 0<self.max_horizon_s<=.5
                or isinstance(self.max_pair_tests,bool) or not isinstance(self.max_pair_tests,int)
                or not 1<=self.max_pair_tests<=262144):
            raise ValueError('invalid dynamic candidate bounds')


@dataclass(frozen=True)
class DynamicCheck:
    allowed: bool
    reason: str
    pair_tests: int = 0
    track_id: int = None
    segment_index: int = None
    # Passing this veto does not certify detection coverage or object geometry.
    safety_certified: bool = field(default=False, init=False)


def check_dynamic_candidates(trajectory, scene, now, epoch, body, bounds):
    """Check full sampled trajectory against linearly predicted candidate boxes.

    Inflate by tracking error, continuous drone-acceleration chord bound and
    explicit position/velocity/acceleration uncertainty for the obstacle. The
    interval uses its maximum uncertainty and closed faces so crossings and
    tangencies between samples are rejected. Missing/uncertified objects cannot
    be proved absent; a passing result is only a candidate conflict veto.
    """
    if not isinstance(scene,DynamicScene) or not isinstance(body,SweepBounds) or not isinstance(bounds,DynamicBounds):
        raise ValueError('dynamic scene and bounds required')
    if not trajectory.metadata.current(now,epoch) or not scene.metadata.current(now,epoch):
        return DynamicCheck(False,'DYNAMIC_SOURCE_UNAVAILABLE')
    if trajectory.metadata.frame_id!='world_ned' or scene.metadata.frame_id!='world_ned':
        return DynamicCheck(False,'DYNAMIC_FRAME')
    horizon=trajectory.metadata.valid_until
    if horizon-now > bounds.max_horizon_s + 1e-9:
        return DynamicCheck(False,'DYNAMIC_HORIZON')
    samples=trajectory.samples
    if len(samples)-1>body.max_segments or (len(samples)-1)*len(scene.tracks)>bounds.max_pair_tests:
        return DynamicCheck(False,'DYNAMIC_BUDGET')
    if any(math.hypot(*sample.acceleration_ned)>body.acceleration_bound_m_s2+1e-9 for sample in samples):
        return DynamicCheck(False,'DYNAMIC_ACCELERATION_BOUND')
    for track in scene.tracks:
        try:
            finite_vec3(track.center_ned);finite_vec3(track.velocity_ned_m_s);finite_vec3(track.half_extents_m)
            if (track.epoch!=epoch or not track.stamp<=now<track.valid_until or track.observations<1
                    or any(v<=0 for v in track.half_extents_m)):
                return DynamicCheck(False,'DYNAMIC_TRACK_UNAVAILABLE',track_id=track.track_id)
        except (TypeError,ValueError):
            return DynamicCheck(False,'DYNAMIC_TRACK_INVALID')
    tested=0
    for i,(first,second) in enumerate(zip(samples,samples[1:])):
        begin,finish=max(now,first.stamp),min(horizon,second.stamp)
        if begin>finish:continue
        duration=second.stamp-first.stamp
        if not math.isfinite(duration) or duration<=0:return DynamicCheck(False,'DYNAMIC_NUMERICAL_RANGE',tested,segment_index=i)
        def drone_at(stamp):
            fraction=(stamp-first.stamp)/duration
            return tuple((1-fraction)*a+fraction*b for a,b in zip(first.position_ned,second.position_ned))
        start,end=drone_at(begin),drone_at(finish)
        deviation=body.tracking_error_m+body.acceleration_bound_m_s2*duration*duration/8
        for track in scene.tracks:
            tested+=1
            dt0,dt1=begin-track.stamp,finish-track.stamp
            velocity=track.velocity_ned_m_s if track.observations>=2 else (0.,0.,0.)
            velocity_error=(bounds.velocity_error_m_s if track.observations>=2
                            else bounds.single_observation_speed_bound_m_s)
            inflation=bounds.position_error_m+velocity_error*dt1+.5*bounds.acceleration_bound_m_s2*dt1*dt1
            relative0=tuple(p-(q+v*dt0) for p,q,v in zip(start,track.center_ned,velocity))
            relative1=tuple(p-(q+v*dt1) for p,q,v in zip(end,track.center_ned,velocity))
            extents=tuple(a+b+deviation+inflation for a,b in zip(body.half_extents_m,track.half_extents_m))
            values=relative0+relative1+extents
            if not all(math.isfinite(v) for v in values):return DynamicCheck(False,'DYNAMIC_NUMERICAL_RANGE',tested,track.track_id,i)
            scale=max(1.,*(abs(v) for v in values));padding=32*sys.float_info.epsilon*scale
            extents=tuple(v+padding for v in extents)
            if _contact_interval(relative0,relative1,tuple(-v for v in extents),extents) is not None:
                return DynamicCheck(False,'DYNAMIC_CANDIDATE_CONFLICT',tested,track.track_id,i)
    return DynamicCheck(True,'NO_DETECTED_DYNAMIC_CONFLICT_UNCERTIFIED',tested)
