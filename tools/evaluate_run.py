#!/usr/bin/env python3
"""Independent offline gate-plane evaluator. No controller route or snapped gates.

Pose CSV columns: stamp,x,y,z (ROS header time in seconds, NED metres).
Gate JSON schema and limitations: docs/17_run_recording_and_scoring.md.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def vector(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError("expected a finite XYZ vector")
    return result


def positive(value):
    result = float(value)
    if not np.isfinite(result) or result <= 0:
        raise ValueError("expected a finite positive value")
    return result


def evaluate(spec, samples, margin=0.25, max_gap=0.25, max_speed=30.0):
    """Score first forward crossings; reject interpolation over gaps/reset/teleports.

    margin is an isotropic body + estimation reserve in each aperture axis.
    Missing evidence stays UNKNOWN; a controller SKIP never becomes a geometric PASS.
    """
    margin = float(margin)
    if not np.isfinite(margin) or margin < 0:
        raise ValueError("margin must be finite and nonnegative")
    max_gap, max_speed = positive(max_gap), positive(max_speed)
    if spec.get("frame") != "NED":
        raise ValueError("gate geometry must explicitly use frame=NED")
    gates, rows, ids = [], [], set()
    for raw in spec["gates"]:
        uid = raw["uid"]
        if not isinstance(uid, str) or not uid or uid in ids:
            raise ValueError("gate UID must be a unique nonempty string")
        ids.add(uid)
        row = {"uid": uid, "status": "UNKNOWN", "reason": "no_forward_crossing"}
        rows.append(row)
        if raw.get("verified") is not True or not raw.get("verification"):
            row["reason"] = "unverified_geometry"
            gates.append(None)
            continue
        center = vector(raw["center"])
        axes = np.array([vector(raw[k]) for k in ("normal", "axis_u", "axis_v")])
        if not np.allclose(axes @ axes.T, np.eye(3), atol=1e-5):
            raise ValueError("gate %s axes must be orthonormal" % uid)
        hw, hh = positive(raw["half_width"]), positive(raw["half_height"])
        if min(hw, hh) <= margin:
            raise ValueError("gate %s has no usable aperture after margin" % uid)
        gates.append((center, axes, hw - margin, hh - margin))
    count = spec.get("expected_gate_count")
    if count is not None and (type(count) is not int or count < len(rows)):
        raise ValueError("expected_gate_count must be an integer >= known gate count")
    complete = (spec.get("complete") is True and count == len(rows) and bool(rows))
    events, issues, previous = [], [], None
    seen, next_index, order_ok, sample_count, last_index = set(), 0, True, 0, -1
    for sample in samples:
        stamp = float(sample["stamp"])
        point = vector([sample[k] for k in ("x", "y", "z")])
        if not np.isfinite(stamp):
            raise ValueError("nonfinite pose timestamp")
        sample_count += 1
        if previous is not None:
            old_stamp, old_point = previous
            dt = stamp - old_stamp
            reason = ("time_reset_or_duplicate" if dt <= 0 else
                      "pose_gap" if dt > max_gap else
                      "pose_jump" if np.linalg.norm(point - old_point) / dt > max_speed else None)
            if reason:
                issues.append({"stamp": stamp, "reason": reason})
            else:
                crossings = []
                for i, gate in enumerate(gates):
                    if gate is None or i in seen:
                        continue
                    center, axes, hw, hh = gate
                    before = float((old_point - center) @ axes[0])
                    after = float((point - center) @ axes[0])
                    if before < 0 <= after:
                        alpha = -before / (after - before)
                        crossing = old_point + alpha * (point - old_point)
                        u, v = (axes @ (crossing - center))[1:]
                        clearance = float(min(hw - abs(u), hh - abs(v)))
                        crossings.append((alpha, i, crossing, float(u), float(v), clearance))
                for alpha, i, crossing, u, v, clearance in sorted(crossings, key=lambda c: c[:2]):
                    seen.add(i)
                    status = "PASS" if clearance > 0 else "MISS"
                    in_order = i == next_index
                    # Missing earlier evidence is UNKNOWN, not proof of a wrong order.
                    order_ok = order_ok and i > last_index
                    last_index = i
                    if in_order:
                        next_index += 1
                    event = {"uid": rows[i]["uid"], "status": status,
                             "stamp": old_stamp + alpha * dt,
                             "intersection": crossing.tolist(), "u": u, "v": v,
                             "clearance_m": clearance, "in_order": in_order}
                    events.append(event)
                    rows[i].update(status=status, reason="forward_plane_crossing")
        previous = stamp, point
    all_pass = bool(rows) and all(r["status"] == "PASS" for r in rows)
    # A broken recording cannot prove a full ordered trajectory, even if later points cross.
    verdict = "UNKNOWN"
    if not issues and sample_count >= 2:
        if any(r["status"] == "MISS" for r in rows) or not order_ok:
            verdict = "FAIL"
        elif complete and all_pass:
            verdict = "PASS"
    return {"schema_version": 1, "geometric_verdict": verdict,
            "official_result": "UNKNOWN", "race_success": "UNKNOWN",
            "complete_gate_inventory": complete, "expected_gate_count": count,
            "pose_samples": sample_count, "margin_m": margin,
            "max_gap_s": max_gap, "max_speed_mps": max_speed,
            "ordered_crossings": order_ok, "data_issues": issues,
            "gates": rows, "events": events,
            "note": "Geometry is independent of controller events; official finish/collision/bounds remain unverified."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gates", type=Path, required=True)
    parser.add_argument("--poses", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--margin", type=float, default=0.25)
    parser.add_argument("--max-gap", type=float, default=0.25)
    parser.add_argument("--max-speed", type=float, default=30.0)
    args = parser.parse_args()
    try:
        with args.poses.open() as fh:
            report = evaluate(json.loads(args.gates.read_text()), csv.DictReader(fh),
                              args.margin, args.max_gap, args.max_speed)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, "evaluation error: %s\n" % exc)


if __name__ == "__main__":
    main()
