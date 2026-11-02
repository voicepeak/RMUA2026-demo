"""State estimation: GPS(10Hz) + IMU(100Hz) EKF.

Reuse main/estimation/imu_gps.py as the starting point; keep this module thin.
TODO(P2): implement proper EKF.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class State:
    t: float          # seconds, monotonic
    pos: np.ndarray   # world NED [3]
    vel: np.ndarray   # world NED [3]
    quat: np.ndarray  # body->world, wxyz [4]


class Estimator:
    def __init__(self, initial_pose: np.ndarray | None = None) -> None:
        self._state = State(0.0, np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]))

    def update_imu(self, t: float, acc: np.ndarray, gyro: np.ndarray) -> None:
        """TODO(P2): propagate with IMU; guard saturation like MonoRace."""
        raise NotImplementedError

    def update_gps(self, t: float, pos: np.ndarray, quat: np.ndarray) -> None:
        """TODO(P2): GPS with-attitude correction."""
        raise NotImplementedError

    @property
    def state(self) -> State:
        return self._state
