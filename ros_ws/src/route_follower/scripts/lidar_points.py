#!/usr/bin/env python3
"""Discard nonfinite/zero-range samples before any sensor-frame transform."""
import numpy as np


def valid_points(points):
    points=np.asarray(points)
    # A return at the emitting origin carries no obstacle position. Do not
    # reject a radius around the aircraft: genuine near surfaces still matter.
    return points[np.all(np.isfinite(points),axis=1)&(np.sum(points*points,axis=1)>1e-8)]
