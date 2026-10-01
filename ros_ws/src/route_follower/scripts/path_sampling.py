#!/usr/bin/env python3
"""Vectorized swept polyline sampling shared by planning and execution."""
import numpy as np


def swept_samples(path,spacing=.18):
    path=np.asarray(path,dtype=float)
    delta=np.diff(path,axis=0)
    counts=np.maximum(1,np.ceil(np.linalg.norm(delta,axis=1)/spacing).astype(int))
    segments=np.repeat(np.arange(len(delta)),counts)
    starts=np.cumsum(counts)-counts
    fraction=(np.arange(np.sum(counts))-np.repeat(starts,counts))/counts[segments]
    queries=path[segments]+fraction[:,None]*delta[segments]
    return np.vstack([queries,path[-1]]),np.r_[segments,len(delta)-1],np.r_[fraction,1.]
