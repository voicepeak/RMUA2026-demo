#!/usr/bin/env python3
"""Offline inventory of route/gate coverage and controller logs; not a race scorer.

Requires PyYAML and numpy. Does not import ROS or modify flight configuration.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def project_gates(route, gates):
    route = np.asarray(route, dtype=float)
    if route.ndim != 2 or route.shape[1] != 3 or len(route) < 2:
        raise ValueError("route must contain at least two XYZ points")
    if not np.isfinite(route).all():
        raise ValueError("route contains nonfinite coordinates")
    delta = np.diff(route[:, :2], axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    if np.any(lengths <= 1e-9):
        raise ValueError("route contains a zero-length XY segment")
    progress = np.r_[0.0, np.cumsum(lengths)]
    rows = []
    for g in gates:
        xyz = np.array([g[k] for k in ("x", "y", "z")], dtype=float)
        if not np.isfinite(xyz).all():
            raise ValueError("gate contains nonfinite coordinates")
        t = np.clip(np.sum((xyz[:2] - route[:-1, :2]) * delta, axis=1)
                    / lengths ** 2, 0, 1)
        centers = route[:-1, :2] + t[:, None] * delta
        errors = np.linalg.norm(centers - xyz[:2], axis=1)
        i = int(np.argmin(errors))
        rows.append({"id": g.get("id"), "valid": g.get("valid", True),
                     "projected_s_m": float(progress[i] + t[i] * lengths[i]),
                     "route_snap_displacement_m": float(errors[i]),
                     "z_m": float(xyz[2]), "source": g.get("source", "unknown")})
    return float(progress[-1]), sorted(rows, key=lambda g: g["projected_s_m"])


def source_diff(source, runtime):
    def inventory(root):
        entries = {}
        for p in root.rglob("*"):
            if "__pycache__" in p.parts or p.suffix == ".pyc":
                continue
            if p.is_symlink():
                entries[str(p.relative_to(root))] = "symlink:" + os.readlink(p)
            elif p.is_file():
                entries[str(p.relative_to(root))] = hashlib.sha256(p.read_bytes()).hexdigest()
        return entries
    a, b = inventory(source), inventory(runtime)
    return {"changed": sorted(k for k in a.keys() & b.keys() if a[k] != b[k]),
            "only_source": sorted(a.keys() - b.keys()),
            "only_runtime": sorted(b.keys() - a.keys())}


def summarize_log(path):
    log = path.read_text(errors="replace")
    states = []
    for line in log.splitlines():
        match = re.search(r"PATH s=([\d.+-]+).*?noFG=(True|False)", line)
        if match:
            states.append({"s_m": float(match[1]), "no_future_gate": match[2] == "True"})
    return {"path": str(path),
            "controller_pass_events": re.findall(r"GATE (\S+) PASSED", log),
            "controller_miss_events": re.findall(r"GATE (\S+) MISSED", log),
            "controller_skip_or_behind_events": re.findall(r"(?:SKIP|MISSED) gate (\S+)", log),
            "controller_goal_reached": "GOAL_REACHED" in log,
            "last_path_state": states[-1] if states else None,
            "max_logged_s_m": max((s["s_m"] for s in states), default=None),
            "official_result": "unknown; controller logs are not independent scoring"}


def main():
    repo = Path(__file__).resolve().parents[1]
    config = repo / "ros_ws/src/route_follower/config"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route-file", type=Path, default=config / "route_1_3.yaml")
    parser.add_argument("--route-name", default="route_1_3")
    parser.add_argument("--gates-file", type=Path, default=config / "gates_vision_1_3.yaml")
    parser.add_argument("--launch-file", type=Path,
                        default=repo / "ros_ws/src/route_follower/launch/route_follower.launch")
    parser.add_argument("--runtime-src", type=Path, default=repo.parent / "rmua_ws/src")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--output", type=Path, help="optional JSON output; stdout otherwise")
    args = parser.parse_args()
    try:
        route = yaml.safe_load(args.route_file.read_text())["routes"][args.route_name]
        gates = yaml.safe_load(args.gates_file.read_text())["gates"]
        length, rows = project_gates(route, gates)
        defaults = {a.attrib["name"]: a.attrib.get("default")
                    for a in ET.parse(args.launch_file).getroot().findall("arg")}
        valid = [g for g in rows if g["valid"]]
        last_s = valid[-1]["projected_s_m"] if valid else 0.0
        report = {
            "scope": "offline inputs and launch defaults; runtime overrides are not inspected",
            "route_file": str(args.route_file), "gates_file": str(args.gates_file),
            "route_name": args.route_name, "route_points": len(route),
            "route_xy_length_m": length, "valid_static_gate_count": len(valid),
            "last_static_gate_s_m": last_s,
            "route_fraction_at_last_static_gate": last_s / length,
            "remaining_route_after_last_static_gate_m": length - last_s,
            "coverage_note": "distance fraction is not gate coverage or race completion",
            "launch_defaults": {k: defaults.get(k) for k in (
                "use_gate_map", "snap_gate_to_route", "cruise_speed",
                "gate_pass_half_width", "gate_pass_half_height", "guides_file")},
            "gates": rows,
            "source_diff": source_diff(repo / "ros_ws/src", args.runtime_src)
            if args.runtime_src.is_dir() else {"error": "runtime source directory missing"},
            "log": summarize_log(args.log) if args.log else None,
        }
        result = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        if args.output:
            args.output.write_text(result)
        else:
            print(result, end="")
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError, ET.ParseError) as exc:
        parser.exit(2, "audit error: %s\n" % exc)


if __name__ == "__main__":
    main()
