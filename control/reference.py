"""Identical bounded position reference for independent PWM and VEL scenes."""
import math
import numpy as np


def benchmark_reference(origin, elapsed, step=False):
    if not math.isfinite(elapsed) or elapsed < 0:
        raise ValueError('invalid reference time')
    origin = np.asarray(origin, dtype=float)
    if origin.shape != (3,) or not np.all(np.isfinite(origin)):
        raise ValueError('invalid reference origin')
    s = min(1., elapsed/3.)
    h = 10*s**3-15*s**4+6*s**5
    dh = (30*s**2-60*s**3+30*s**4)/3
    ddh = (60*s-180*s**2+120*s**3)/9 if s < 1 else 0.
    position = origin+(0., 0., -h)
    if step and elapsed >= 4:
        position[0] += .4 if elapsed < 8 else -.4
    return tuple(position), (0., 0., -dh), (0., 0., -ddh)


def benchmark_yaw(initial_yaw, elapsed, enabled=False):
    if not math.isfinite(initial_yaw) or not math.isfinite(elapsed) or elapsed < 0:
        raise ValueError('invalid yaw reference')
    if not enabled:
        return initial_yaw, 0.
    progress = min(2., max(0., elapsed-5.))
    return initial_yaw+.1*progress, .1 if 5 <= elapsed < 7 else 0.
