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

    def update(self,previous,current,dt,tangent):
        if dt>0.:
            measured=(np.asarray(current)-np.asarray(previous))/dt
            beta=1. if self.time_constant==0. else -math.expm1(-dt/self.time_constant)
            self.velocity+=beta*(measured-self.velocity)
        progress=max(0.,float(np.dot(self.velocity[:2],tangent[:2])))
        return progress,-float(self.velocity[2])
