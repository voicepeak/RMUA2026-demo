"""HUD overlay: gate boxes, target, speed, stage, score into the video stream.

Input: image frame + state + gate observations + mission status.
Output: annotated frame; optionally mux to mp4 (ffmpeg) -- one file per run.
"""

import numpy as np


def draw(
    frame: np.ndarray,
    state=None,
    observations=None,
    status: dict | None = None,
) -> np.ndarray:
    """TODO(P0): draw gate quads, target marker, speed/stage text."""
    return frame
