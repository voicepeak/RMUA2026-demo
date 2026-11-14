"""Gate map: world-frame gate poses + projection consistency check.

Format (gates_<seed>.yaml), aligned with beta gates_seed123_recorded.yaml:
  seed: 123
  gates:
    - id: 1
      center: [x, y, z]       # world NED
      normal: [nx, ny, nz]    # gate plane normal (world)
      size: [width, height]   # inner opening, meters
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass
class Gate:
    id: int
    center: np.ndarray
    normal: np.ndarray
    size: tuple[float, float]


def load(path: str | Path) -> list[Gate]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return [
        Gate(
            id=int(g["id"]),
            center=np.asarray(g["center"], dtype=float),
            normal=np.asarray(g["normal"], dtype=float),
            size=tuple(float(s) for s in g.get("size", (1.5, 1.5)))[:2],
        )
        for g in data["gates"]
    ]


def save(path: str | Path, seed: int, gates: list[Gate]) -> None:
    payload = {
        "seed": seed,
        "gates": [
            {
                "id": g.id,
                "center": [float(x) for x in g.center],
                "normal": [float(x) for x in g.normal],
                "size": [float(g.size[0]), float(g.size[1])],
            }
            for g in gates
        ],
    }
    Path(path).write_text(
        yaml.safe_dump(payload, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )


def reprojection_iou(gate: Gate, corners_px: np.ndarray, k: np.ndarray, r: np.ndarray, t: np.ndarray) -> float:
    """TODO(P1): project known gate quad with (R,t) and compare against observed mask IoU."""
    raise NotImplementedError
