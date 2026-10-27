"""Diagnostic loosely coupled visual/inertial propagation, never control-ready."""
import bisect
import math
import numpy as np
from estimation.geometry import rotation, unit
from estimation.history import slerp


class InertialVisualTrajectory:
    """Initial-body local position, with explicitly uncalibrated blend gains.

    IMU orientation is taken from the official sensor, not inferred truth.
    The caller supplies visual relative displacement in the previous body frame.
    Initial velocity is a zero assumption; correlations, bias, uncertainty and
    delayed updates are not modeled. Any missing interval breaks the chain.
    """
    def __init__(self, stamp, epoch, anchor_orientation, position_gain=.5, velocity_gain=.3):
        if (not math.isfinite(stamp) or stamp<0 or isinstance(epoch,bool) or not isinstance(epoch,int) or epoch<0
                or not all(math.isfinite(x) and 0<=x<=1 for x in (position_gain,velocity_gain))):
            raise ValueError('invalid fusion parameters')
        self.stamp,self.epoch=stamp,epoch
        self.anchor_rotation=rotation(unit(anchor_orientation));self.p=np.zeros(3);self.v=np.zeros(3)
        self.position_gain,self.velocity_gain=position_gain,velocity_gain;self.steps=0;self.broken=False

    def append(self, current_stamp, visual_translation, samples, max_gap_s=.04):
        if self.broken:raise ValueError('fusion chain broken')
        try:
            return self._append(current_stamp,visual_translation,samples,max_gap_s)
        except (ValueError,ArithmeticError,TypeError):
            self.broken=True
            raise

    def _append(self, end, visual_translation, samples, max_gap_s):
        start=self.stamp;delta=np.asarray(visual_translation,dtype=float)
        if (delta.shape!=(3,) or not np.isfinite(delta).all() or not math.isfinite(end)
                or not .02<=end-start<=1. or not math.isfinite(max_gap_s) or not 0<max_gap_s<=.1):
            raise ValueError('invalid fusion interval')
        times=[s[0] for s in samples]
        if (len(times)<2 or any(not math.isfinite(x) for x in times)
                or any(b<=a for a,b in zip(times,times[1:]))):raise ValueError('invalid imu timestamps')
        left=bisect.bisect_right(times,start)-1;right=bisect.bisect_left(times,end)
        if left<0 or right>=len(times):raise ValueError('imu interval not bracketed')
        portion=samples[left:right+1]
        if any(b[0]-a[0]>max_gap_s for a,b in zip(portion,portion[1:])):raise ValueError('imu sample gap')
        values=[]
        for _,acc,q in portion:
            acc=np.asarray(acc,dtype=float)
            if acc.shape!=(3,) or not np.isfinite(acc).all():raise ValueError('invalid acceleration')
            values.append((acc,unit(q)))
        def imu(t):
            i=min(bisect.bisect_right(times,t)-1,right-1);f=(t-times[i])/(times[i+1]-times[i])
            a,q=values[i-left];b,r=values[i-left+1]
            return (1-f)*a+f*b,rotation(slerp(q,r,f))
        points=[start]+[t for t in times[left+1:right] if start<t<end]+[end]
        anchor_p=self.p.copy();p=self.p.copy();v=self.v.copy();_,start_r=imu(start)
        for first,last in zip(points,points[1:]):
            a0,r0=imu(first);a1,r1=imu(last);dt=last-first
            accel=self.anchor_rotation.T @ (.5*(r0@a0+r1@a1)+np.array((0.,0.,9.81)))
            p+=v*dt+.5*accel*dt*dt;v+=accel*dt
        visual=anchor_p+self.anchor_rotation.T@start_r@delta
        residual=visual-p
        p+=self.position_gain*residual;v+=self.velocity_gain*residual/(end-start)
        if not np.isfinite(p).all() or not np.isfinite(v).all():raise ValueError('fusion numerical range')
        _,end_r=imu(end);self.p,self.v,self.stamp=p,v,end;self.steps+=1
        return {'stamp':end,'epoch':self.epoch,'position_initial_body_m':tuple(p),
                'velocity_initial_body_m_s':tuple(v),
                'rotation_initial_from_body':tuple(tuple(float(x) for x in row) for row in self.anchor_rotation.T@end_r),
                'steps':self.steps,'control_eligible':False,'covariance_calibrated':False}
