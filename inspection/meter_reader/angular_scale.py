"""Provisional angular meter reading with explicit geometric uncertainty."""
from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class MeterCandidate:
    status: str
    value: int = None
    lower_value: float = None
    upper_value: float = None
    # Neither geometric inputs nor the selected equipment are verified.
    report_eligible: bool = field(default=False, init=False)
    scale_calibrated: bool = field(default=False, init=False)


def read_angle(center, pointer_tip, zero_angle_rad, sweep_rad, angular_uncertainty_rad):
    """Map a manually specified 0..50 dial arc to an integer candidate.

    Angles use image x-right/y-down coordinates, with signed sweep. The input
    pointer, center, endpoints, and uncertainty must come from the same image;
    no association, image detection, calibration, or reporting is implied.
    """
    try:
        cx,cy=center;px,py=pointer_tip
        vals=(cx,cy,px,py,zero_angle_rad,sweep_rad,angular_uncertainty_rad)
        valid=(all(math.isfinite(v) for v in vals) and 0<abs(sweep_rad)<2*math.pi
               and 0<=angular_uncertainty_rad<abs(sweep_rad)/2 and math.hypot(px-cx,py-cy)>=2.)
    except (TypeError,ValueError):
        valid=False
    if not valid:raise ValueError('invalid dial geometry')
    direction=1 if sweep_rad>0 else -1
    angle=math.atan2(py-cy,px-cx)
    offset=(direction*(angle-zero_angle_rad))%(2*math.pi)
    if offset-angular_uncertainty_rad<0 or offset+angular_uncertainty_rad>abs(sweep_rad):
        return MeterCandidate('OUTSIDE_CALIBRATED_ARC')
    lower=(offset-angular_uncertainty_rad)*50/abs(sweep_rad)
    upper=(offset+angular_uncertainty_rad)*50/abs(sweep_rad)
    # Half-integer boundaries are ambiguous, including exact endpoints.
    first=math.floor(lower+.5);last=math.floor(upper+.5)
    if first!=last or abs((lower+.5)-round(lower+.5))<1e-12 or abs((upper+.5)-round(upper+.5))<1e-12:
        return MeterCandidate('AMBIGUOUS',None,lower,upper)
    return MeterCandidate('CANDIDATE',first,lower,upper)
