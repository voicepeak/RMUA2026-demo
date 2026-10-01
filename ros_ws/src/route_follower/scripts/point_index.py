#!/usr/bin/env python3
"""Exact nearest point queries using the installed OpenCV FLANN KD tree."""
import cv2
import numpy as np


class PointIndex:
    def __init__(self,points):
        self.points=np.ascontiguousarray(points,dtype=np.float32)
        self.tree=(cv2.flann_Index(self.points,dict(algorithm=1,trees=1)) if len(points) else None)

    def distance(self,queries):
        queries=np.ascontiguousarray(queries,dtype=np.float32).reshape(-1,3)
        if self.tree is None:return np.full(len(queries),np.inf)
        _,distance=self.tree.knnSearch(queries,1,params=dict(checks=-1))
        return np.sqrt(np.maximum(0.,distance[:,0]))
