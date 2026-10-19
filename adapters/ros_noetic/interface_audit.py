"""Read-only whitelist audit. Never publishes control or calls task services."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import statistics
import threading
import time

def audit(config, duration):
    import rosgraph
    import roslib.message
    import rospy
    import rosservice

    rospy.init_node("rmua_next_interface_audit", anonymous=True, disable_signals=True)
    master = rosgraph.Master(rospy.get_name())
    types = dict(master.getTopicTypes())
    publishers, subscribers, _ = master.getSystemState()
    pub_map, sub_map = dict(publishers), dict(subscribers)
    samples = {topic: deque(maxlen=10000) for topic in config["sensors"]}
    listeners = []
    lock = threading.Lock()
    observed = {}

    def receive(topic, msg):
        arrival = time.monotonic()
        header = getattr(msg, "header", None)
        stamp = header.stamp.to_sec() if header else None
        # ROS now - stamp is diagnostic clock difference, not proven actuation latency.
        ros_now = rospy.Time.now().to_sec()
        age = ros_now - stamp if stamp is not None and stamp > 0 else None
        with lock:
            samples[topic].append((arrival, stamp, age))

    for topic in config["sensors"]:
        if "/debug/" in topic:
            raise ValueError("debug topics are excluded from algorithm inputs")
        type_name = types.get(topic)
        cls = roslib.message.get_message_class(type_name) if type_name else None
        observed[topic] = {"type": type_name, "publishers": pub_map.get(topic, []),
                           "message_class_available": cls is not None}
        if cls is not None:
            listeners.append(rospy.Subscriber(topic, cls, lambda msg, t=topic: receive(t, msg),
                                              queue_size=1))
    controls = {topic: {"type": types.get(topic), "publishers": pub_map.get(topic, []),
                        "subscribers": sub_map.get(topic, [])} for topic in config["control_topics"]}
    services = {name: rosservice.get_service_type(name) for name in config["services"]}
    try:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and not rospy.is_shutdown():
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
    finally:
        for listener in listeners:
            listener.unregister()
    with lock:
        captured = {topic: list(rows) for topic, rows in samples.items()}
    for topic, rows in captured.items():
        stamps = [r[1] for r in rows if r[1] is not None]
        ages = [r[2] for r in rows if r[2] is not None and math.isfinite(r[2])]
        span = rows[-1][0] - rows[0][0] if len(rows) > 1 else 0
        observed[topic].update({
            "received_count": len(rows),
            "receiver_hz": (len(rows) - 1) / span if span > 0 else None,
            "stamp_regressions": sum(b < a for a, b in zip(stamps, stamps[1:])),
            "ros_clock_difference_median_s": statistics.median(ages) if ages else None,
        })
    return {"topics": observed, "controls": controls, "services": services,
            "missing_topics": [t for t in config["sensors"] if not observed[t]["received_count"]],
            "formal_server_permissions_verified": False,
            "pwm_range_and_motor_signs_verified": False,
            "clock_note": "receiver rate is wall time; ROS clock difference is not end-to-end control latency"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--run-dir", type=Path, required=True,
                        help="existing exclusive run prepared on the host")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 120:
        parser.error("duration must be in (0, 120] seconds")
    run = args.run_dir
    manifest = json.loads((run / "manifest.json").read_text())
    config = manifest["config"]
    print("Run directory:", run, flush=True)
    code = 1
    try:
        report = audit(config, args.duration)
        (run / "interface_audit.json").write_text(json.dumps(report, indent=2) + "\n")
        manifest["topic_manifest"] = "interface_audit.json"
        manifest["audit_state"] = "INCOMPLETE" if report["missing_topics"] else "OBSERVED"
        code = 2 if report["missing_topics"] else 0
    except Exception as error:
        manifest["audit_state"] = "FAILED"
        manifest["error"] = str(error)
        print("Audit failed:", error, flush=True)
    finally:
        (run / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
