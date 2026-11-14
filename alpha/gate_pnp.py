"""Gate relative pose from 4 corners + known gate size (monocular PnP).

MonoRace-style: gate inner opening size is officially known.
TODO(P3): cv2.solvePnP(IPPE), reject oblique views, return covariance-ish score.
"""

import numpy as np


def gate_object_points(width: float, height: float) -> np.ndarray:
    """(4,3) object points of gate opening corners in gate frame (z out of plane)."""
    w, h = width / 2.0, height / 2.0
    return np.array(
        [[-w, +h, 0.0], [+w, +h, 0.0], [+w, -h, 0.0], [-w, -h, 0.0]],
        dtype=np.float64,
    )


def solve(corners_px: np.ndarray, k: np.ndarray, width: float, height: float):
    """TODO(P3): return (R, t) gate->camera. Reject if re-projection error > thresh."""
    raise NotImplementedError
