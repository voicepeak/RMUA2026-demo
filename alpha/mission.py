"""Alpha mission: single entry point.

Flow (to implement):
  1. start simulator (reuse main tools/run_sim.sh with seed)
  2. wait sensors ready (IMU)
  3. mode == collect: collect.py scout run -> gates_<seed>.yaml
     mode == race:    mission state machine (takeoff -> gates -> stages)
  4. append result line to scoreboard.jsonl
  5. always stop simulator and clean up
"""

import argparse
import json
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCOREBOARD = HERE / "scoreboard.jsonl"


def parse() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="alpha mission")
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--mode", choices=["race", "collect"], default="race")
    ap.add_argument("--conservative", action="store_true", default=True)
    return ap.parse_args()


def run_race(seed: int) -> dict:
    """TODO: state -> gate_detect -> gate_pnp -> planner -> control loop.

    Return score dict:
      {"stage": int, "score": number, "time_s": float, "fail": str|null}
    """
    raise NotImplementedError("race loop: P2/P3/P4 work")


def run_collect(seed: int) -> dict:
    """TODO: pose_gt driven scout run, dump gates_<seed>.yaml (see collect.py)."""
    raise NotImplementedError("collect run: P1 work")


def main() -> None:
    args = parse()
    t0 = time.time()
    try:
        result = run_collect(args.seed) if args.mode == "collect" else run_race(args.seed)
    except NotImplementedError as exc:
        result = {"stage": 0, "score": 0, "time_s": -1.0, "fail": str(exc)}
    result.update(
        {
            "ts": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "seed": args.seed,
            "mode": args.mode,
            "wall_s": round(time.time() - t0, 1),
        }
    )
    with SCOREBOARD.open("a", encoding="utf-8") as fp:
        fp.write(json.dumps(result, ensure_ascii=False) + "\n")
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
