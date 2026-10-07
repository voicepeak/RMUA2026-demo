#!/usr/bin/env python3
"""Filtered measured motion, projected onto the road rather than command speed."""
import math
import numpy as np


class MotionEstimator:
    def __init__(self,time_constant=.12):
        self.time_constant=max(0.,float(time_constant))
        self.reset()

    def reset(self):
        self.velocity=np.zeros(3)

    @staticmethod
    def terminal_velocity(rows,stamp,window=.18):
        """Causal endpoint derivative of high-rate poses, with gap rejection.

        Center position and scale time before fitting to preserve precision at
        large world coordinates and epoch timestamps. Never bridge a sensor
        outage or teleport; the caller retains its finite-difference fallback.
        """
        selected=[(t,p) for t,p,*_ in rows if stamp-window<=t<=stamp]
        if len(selected)<6:return None
        times=np.array([r[0] for r in selected],dtype=float)
        positions=np.asarray([r[1] for r in selected],dtype=float)
        gaps=np.diff(times)
        if (not np.all(np.isfinite(positions)) or times[-1]!=stamp or
                times[-1]-times[0]<.10 or np.any(gaps<=0.) or np.any(gaps>.05)):
            return None
        if np.any(np.linalg.norm(np.diff(positions,axis=0),axis=1)>60.*gaps+.02):return None
        x=(times-stamp)/window
        design=np.column_stack((np.ones(len(x)),x,x*x))
        coefficient=np.linalg.lstsq(design,positions-positions[-1],rcond=None)[0]
        return coefficient[1]/window

    def project(self,tangent):
        return max(0.,float(np.dot(self.velocity[:2],tangent[:2]))),-float(self.velocity[2])

    def update(self,previous,current,dt,tangent):
        if dt>0.:
            measured=(np.asarray(current)-np.asarray(previous))/dt
            beta=1. if self.time_constant==0. else -math.expm1(-dt/self.time_constant)
            self.velocity+=beta*(measured-self.velocity)
        return self.project(tangent)
