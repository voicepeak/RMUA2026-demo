"""Route planner: gate sequence -> straight waypoints + speed profile.

Keep it dumb on purpose: line between gate centers, trapezoid speed,
brake distance from a measured deceleration envelope.
TODO(P4): speed envelope from logged runs, gate approach offsets.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Setpoint:
    pos: np.ndarray   # world NED
    vel: np.ndarray   # world NED
    t: float          # relative time


def plan_leg(start: np.ndarray, target: np.ndarray, v_max: float, a_max: float) -> list[Setpoint]:
    """TODO(P4): trapezoid profile along the segment; returns dense setpoints."""
    raise NotImplementedError


def waypoints_from_map(gates) -> list[np.ndarray]:
    """TODO(P3): gate centers in order; optional per-gate approach offset."""
    return [np.asarray(g.center, dtype=float) for g in gates]
