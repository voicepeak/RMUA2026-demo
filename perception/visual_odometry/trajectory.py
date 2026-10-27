"""Accumulate contiguous relative VO into a local, non-control trajectory."""
from dataclasses import dataclass, field
import math
import numpy as np


@dataclass(frozen=True)
class LocalVisualPose:
    stamp: float
    epoch: int
    position_initial_body_m: tuple
    rotation_initial_from_body: tuple
    steps: int
    control_eligible: bool = field(default=False, init=False)
    covariance_calibrated: bool = field(default=False, init=False)


class VisualTrajectory:
    """A missing relative step ends the chain; no implicit relocalization."""
    def __init__(self, stamp, epoch, max_step_s=1.):
        if (not math.isfinite(stamp) or stamp < 0 or isinstance(epoch, bool) or not isinstance(epoch, int)
                or epoch < 0 or not math.isfinite(max_step_s) or not 0 < max_step_s <= 2):
            raise ValueError('invalid visual anchor')
        self.stamp, self.epoch, self.max_step_s = stamp, epoch, max_step_s
        self.position = np.zeros(3); self.rotation = np.eye(3); self.steps = 0; self.broken = False

    def invalidate(self):
        self.broken = True

    def append(self, motion):
        if self.broken:
            raise ValueError('visual chain already broken')
        try:
            p = np.asarray(motion.translation_previous_body_m, dtype=float)
            r = np.asarray(motion.rotation_previous_from_current_body, dtype=float)
            valid = (motion.epoch == self.epoch and math.isfinite(motion.previous_stamp)
                     and math.isfinite(motion.current_stamp) and abs(motion.previous_stamp-self.stamp) <= 1e-6
                     and 0 < motion.current_stamp - motion.previous_stamp <= self.max_step_s
                     and p.shape == (3,) and r.shape == (3,3)
                     and np.isfinite(p).all() and np.isfinite(r).all()
                     and np.allclose(r.T @ r, np.eye(3), rtol=0, atol=1e-5)
                     and abs(np.linalg.det(r)-1.) <= 1e-5)
        except (TypeError, ValueError, AttributeError):
            valid = False
        if not valid:
            self.broken = True
            raise ValueError('invalid or discontinuous relative motion')
        position = self.position + self.rotation @ p
        orientation = self.rotation @ r
        if not np.isfinite(position).all() or not np.isfinite(orientation).all():
            self.broken = True
            raise ValueError('visual trajectory numerical range')
        self.position, self.rotation = position, orientation
        self.stamp = motion.current_stamp; self.steps += 1
        return self.snapshot()

    def snapshot(self):
        return LocalVisualPose(self.stamp, self.epoch, tuple(self.position),
                               tuple(tuple(float(v) for v in row) for row in self.rotation), self.steps)
