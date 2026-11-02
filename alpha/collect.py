"""Scout run: collect gate geometry for a seed using pose_gt (development only).

Procedure (TODO P1):
  1. run simulator with GameConfig debug switches (ignore collision/timeout)
  2. fly along the route (pose_gt as reference input; DEV ONLY, never in race code)
  3. for each detected gate: PnP relative pose + pose_gt -> world gate pose
  4. refine with multi-frame averaging; dump gates_<seed>.yaml
  5. validate: re-project gates with recorded poses, IoU > 0.8

Compliance note: pose_gt is a debug topic. This script never runs in race mode;
its output yaml is static data consumed by gate_map.load().
"""

from pathlib import Path


def scout(seed: int, out_dir: str | Path) -> Path:
    """TODO(P1): return path to gates_<seed>.yaml."""
    raise NotImplementedError
