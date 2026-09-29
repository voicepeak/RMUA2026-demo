#!/usr/bin/env python3
"""Export synchronized evidence from a completed run's ROS bag (ROS environment)."""
import argparse
import csv
import json
from pathlib import Path

from evaluate_run import evaluate
from run_experiment import write_json


def export(run, gates_path=None):
    import rosbag
    events, pose_count = [], 0
    with rosbag.Bag(str(run / "telemetry.bag")) as bag, \
            (run / "poses.csv").open("w") as poses, \
            (run / "controller_events.jsonl").open("w") as event_file, \
            (run / "controller_telemetry.jsonl").open("w") as telemetry:
        writer = csv.writer(poses)
        writer.writerow(["stamp", "x", "y", "z", "qx", "qy", "qz", "qw", "bag_stamp", "frame_id"])
        counts = {topic: info.message_count for topic, info in bag.get_type_and_topic_info().topics.items()}
        for topic, msg, stamp in bag.read_messages(topics=[
                "/airsim_node/drone_1/debug/pose_gt", "/rmua/controller/events", "/rmua/controller/telemetry"]):
            if topic == "/airsim_node/drone_1/debug/pose_gt":
                p, q = msg.pose.position, msg.pose.orientation
                writer.writerow([msg.header.stamp.to_sec(), p.x, p.y, p.z,
                                 q.x, q.y, q.z, q.w, stamp.to_sec(), msg.header.frame_id])
                pose_count += 1
            elif topic in ("/rmua/controller/events", "/rmua/controller/telemetry"):
                payload = json.loads(msg.data)
                payload["bag_stamp"] = stamp.to_sec()
                if topic.endswith("/events"):
                    events.append(payload)
                    event_file.write(json.dumps(payload, allow_nan=False) + "\n")
                else:
                    telemetry.write(json.dumps(payload, allow_nan=False) + "\n")
    controller = {state: [e for e in events if e.get("kind") == "GATE" and e.get("status") == state]
                  for state in ("PASS", "MISS", "SKIP", "UNKNOWN")}
    summary = {"topic_message_counts": counts, "pose_samples": pose_count,
               "controller_events": controller,
               "controller_terminations": [e for e in events if e.get("kind") == "TERMINATION"],
               "official_result": "UNKNOWN", "race_success": "UNKNOWN",
               "note": "Raw official/custom topics stay in the bag; no result adapter is verified yet."}
    if gates_path:
        with (run / "poses.csv").open() as fh:
            summary["independent_geometry"] = evaluate(json.loads(gates_path.read_text()), csv.DictReader(fh))
    else:
        summary["independent_geometry"] = {"geometric_verdict": "UNKNOWN", "reason": "no_verified_gate_inventory"}
    write_json(run / "summary.json", summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--gates", type=Path, help="independently verified geometry JSON")
    args = parser.parse_args()
    export(args.run_dir, args.gates)


if __name__ == "__main__":
    main()
