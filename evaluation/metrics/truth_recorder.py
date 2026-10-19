"""LOCAL EVALUATION ONLY. Separate process, no publishers or controller imports."""
import argparse
import json
from pathlib import Path
import threading
import time


def main():
    import rospy
    import roslib.message
    from geometry_msgs.msg import PoseStamped
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    run = args.run_dir
    rospy.init_node('rmua_hidden_truth_evaluator', anonymous=True, disable_signals=True)
    lock = threading.Lock()
    rows = []
    actuation=[]
    def receive(msg):
        p, q = msg.pose.position, msg.pose.orientation
        row = {'stamp': msg.header.stamp.to_sec(), 'position': (p.x, p.y, p.z),
               'orientation': (q.w, q.x, q.y, q.z),'arrival_wall_monotonic':time.monotonic()}
        with lock:
            if len(rows) < 30000:
                rows.append(row)
    listener = rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt', PoseStamped,
                                receive, queue_size=1)
    def plain(value):
        if hasattr(value,'__slots__'):
            return {slot:plain(getattr(value,slot)) for slot in value.__slots__}
        if isinstance(value,(list,tuple)):
            return [plain(v) for v in value]
        return value
    def receive_actuation(msg):
        header=msg._connection_header
        cls=roslib.message.get_message_class(header['type'])
        row={'arrival_wall_monotonic':time.monotonic(),'type':header['type'],
             'md5sum':header['md5sum']}
        if cls is not None and cls._md5sum==header['md5sum']:
            decoded=cls();decoded.deserialize(msg._buff)
            row['data']=plain(decoded)
        else:
            row['raw_hex']=msg._buff.hex()
        with lock:
            if len(actuation)<30000:actuation.append(row)
    actuator_listener=rospy.Subscriber('/airsim_node/drone_1/debug/rotor_pwm',rospy.AnyMsg,
                                       receive_actuation,queue_size=1)
    deadline = time.monotonic()+40
    try:
        while time.monotonic() < deadline and not rospy.is_shutdown() and not (run/'truth_stop').exists():
            with lock:
                if rows and not (run/'truth_ready').exists():
                    (run/'truth_ready').touch()
            time.sleep(.02)
    finally:
        listener.unregister()
        actuator_listener.unregister()
        with lock:
            (run/'evaluation_truth.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
            (run/'evaluation_actuation.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in actuation))
    return 0 if rows else 2


if __name__ == '__main__':
    raise SystemExit(main())
