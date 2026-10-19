"""LOCAL EVALUATION ONLY. Separate process, no publishers or controller imports."""
import argparse
import json
from pathlib import Path
import threading
import time


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    run = args.run_dir
    rospy.init_node('rmua_hidden_truth_evaluator', anonymous=True, disable_signals=True)
    lock = threading.Lock()
    rows = []
    def receive(msg):
        p, q = msg.pose.position, msg.pose.orientation
        row = {'stamp': msg.header.stamp.to_sec(), 'position': (p.x, p.y, p.z),
               'orientation': (q.w, q.x, q.y, q.z)}
        with lock:
            if len(rows) < 30000:
                rows.append(row)
    listener = rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt', PoseStamped,
                                receive, queue_size=1)
    deadline = time.monotonic()+40
    try:
        while time.monotonic() < deadline and not rospy.is_shutdown() and not (run/'truth_stop').exists():
            with lock:
                if rows and not (run/'truth_ready').exists():
                    (run/'truth_ready').touch()
            time.sleep(.02)
    finally:
        listener.unregister()
        with lock:
            (run/'evaluation_truth.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    return 0 if rows else 2


if __name__ == '__main__':
    raise SystemExit(main())
