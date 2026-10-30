"""Identical bounded position reference for independent PWM and VEL scenes."""
import math
import numpy as np


def benchmark_reference(origin, elapsed, step=False, braking=False, round_trip=False):
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
    if round_trip and (step or braking):
        raise ValueError('round trip must be isolated')
    vx = 0.
    ax = 0.
    if round_trip:
        # Two smooth 0.3 m legs, each ending at zero velocity (braking).
        leg = 1 if 4 <= elapsed < 8 else -1 if 9 <= elapsed < 13 else 0
        if leg:
            u = (elapsed-(4 if leg == 1 else 9))/4
            q = 10*u**3-15*u**4+6*u**5
            dq = (30*u**2-60*u**3+30*u**4)/4
            ddq = (60*u-180*u**2+120*u**3)/16
            position[0] += .3*(q if leg == 1 else 1-q)
            vx, ax = leg*.3*dq, leg*.3*ddq
        elif 8 <= elapsed < 9:
            position[0] += .3
    if braking:
        # Deliberate velocity steps identify response; not a mission trajectory.
        position[0] += .4*min(2.,max(0.,elapsed-4.))
        vx = .4 if 4 <= elapsed < 6 else 0.
    return tuple(position), (vx, 0., -dh), (ax, 0., -ddh)


def benchmark_yaw(initial_yaw, elapsed, enabled=False):
    if not math.isfinite(initial_yaw) or not math.isfinite(elapsed) or elapsed < 0:
        raise ValueError('invalid yaw reference')
    if not enabled:
        return initial_yaw, 0.
    progress = min(2., max(0., elapsed-5.))
    return initial_yaw+.1*progress, .1 if 5 <= elapsed < 7 else 0.
