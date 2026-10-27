"""Compare relative visual rotation to causal gyroscope integration."""
import bisect
import math
import numpy as np
from estimation.geometry import delta_quaternion, rotation


def gyro_rotation(samples, start, end, max_gap_s=.04):
    """Integrate unbiased-or-raw body gyro; no extrapolation or gap bridging.

    samples are (stamp, angular_velocity_body), already restricted to one epoch.
    The returned previous-from-current orientation is diagnostic and has no
    calibrated error bound. Bias correction, if any, is the caller's input.
    """
    if (not math.isfinite(start) or not math.isfinite(end) or not 0 <= start < end
            or not math.isfinite(max_gap_s) or not 0 < max_gap_s <= .1):
        raise ValueError('invalid gyro interval')
    stamps = [x[0] for x in samples]
    if (len(samples)<2 or any(not math.isfinite(s) for s in stamps)
            or any(b <= a for a,b in zip(stamps,stamps[1:]))):
        raise ValueError('invalid gyro timestamps')
    left=bisect.bisect_right(stamps,start)-1;right=bisect.bisect_left(stamps,end)
    if left<0 or right>=len(stamps):raise ValueError('gyro interval not bracketed')
    selection = samples[left:right+1]
    if any(b[0]-a[0]>max_gap_s for a,b in zip(selection,selection[1:])):
        raise ValueError('gyro sample gap')
    rates=[]
    for _, rate in selection:
        v=np.asarray(rate,dtype=float)
        if v.shape!=(3,) or not np.isfinite(v).all():raise ValueError('invalid gyro rate')
        rates.append(v)
    def interpolate(t):
        i=min(bisect.bisect_right(stamps,t)-1,right-1)
        a,b=stamps[i],stamps[i+1];f=(t-a)/(b-a)
        return (1-f)*rates[i-left]+f*rates[i-left+1]
    times=[start]+[s for s in stamps[left+1:right] if start<s<end]+[end]
    result=np.eye(3)
    for a,b in zip(times,times[1:]):
        result=result @ rotation(delta_quaternion((interpolate(a)+interpolate(b))*.5*(b-a)))
    return result


def rotation_residual(visual_rotation, gyro):
    visual=np.asarray(visual_rotation,dtype=float);gyro=np.asarray(gyro,dtype=float)
    if (visual.shape!=(3,3) or gyro.shape!=(3,3) or not np.isfinite(visual).all() or not np.isfinite(gyro).all()
            or any(not np.allclose(r.T@r,np.eye(3),rtol=0,atol=1e-5) or abs(np.linalg.det(r)-1)>1e-5
                   for r in (visual,gyro))):
        raise ValueError('invalid rotation')
    return float(np.arccos(np.clip((np.trace(gyro.T @ visual)-1)/2,-1,1)))
