"""Monocular gate corner detection.

Start from beta/ros_ws/src/rmua_gate_vision (YOLOv8s best.pt).
TODO(P3): corners instead of plain bbox (MonoRace QuAdGate style).
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class GateObs:
    px: np.ndarray      # (4,2) corner pixels, order: TL,TR,BR,BL
    score: float
    gate_id: int | None = None


class GateDetector:
    def __init__(self, weights: str, conf: float = 0.4) -> None:
        self.weights = weights
        self.conf = conf

    def detect(self, image: np.ndarray) -> list[GateObs]:
        """TODO(P3): bbox -> refine corners from mask/edges; RANSAC on quad shape."""
        raise NotImplementedError
