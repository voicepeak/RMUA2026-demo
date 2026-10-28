"""Immutable sparse occupancy. Missing, stale or other-epoch cells are UNKNOWN."""
from dataclasses import dataclass
from enum import Enum
import math
from typing import Tuple
from interfaces.contracts import Metadata, Vec3, finite_vec3


class Occupancy(str, Enum):
    FREE = "FREE"
    OCCUPIED = "OCCUPIED"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class Cell:
    key: Tuple[int, int, int]
    status: Occupancy
    metadata: Metadata

    def __post_init__(self):
        if (not isinstance(self.key, tuple) or len(self.key) != 3
                or not all(isinstance(k, int) for k in self.key)
                or not isinstance(self.status, Occupancy)):
            raise ValueError("invalid occupancy cell")


@dataclass(frozen=True)
class OccupancySnapshot:
    metadata: Metadata
    resolution_m: float
    cells: Tuple[Cell, ...]

    def __post_init__(self):
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError("positive resolution required")
        if not isinstance(self.cells, tuple):
            raise ValueError("cells must be immutable")
        if len({c.key for c in self.cells}) != len(self.cells):
            raise ValueError("duplicate voxel keys")

    def query(self, position: Vec3, now: float, epoch: int) -> Occupancy:
        finite_vec3(position)
        if not self.metadata.current(now, epoch):
            return Occupancy.UNKNOWN
        key = tuple(math.floor(x / self.resolution_m) for x in position)
        for cell in self.cells:
            if cell.key == key:
                return (cell.status if cell.metadata.frame_id == self.metadata.frame_id
                        and cell.metadata.current(now, epoch) else Occupancy.UNKNOWN)
        return Occupancy.UNKNOWN
