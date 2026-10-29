"""Optional independent PWM wire observer; no control or truth publication."""
import argparse
import json
from pathlib import Path
import threading
import time


def main():
    import rospy
    from airsim_ros.msg import RotorPWM

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True, type=Path)
    args = parser.parse_args()
    run = args.run_dir
    rospy.init_node('rmua_second_wire_observer', anonymous=True, disable_signals=True)
    lock = threading.Lock()
    rows = []
    def receive(msg):
        arrival = time.monotonic()
        stamp = msg.header.stamp
        row = {'arrival_wall_monotonic': arrival, 'type': 'airsim_ros/RotorPWM',
               'md5sum': msg._md5sum,
               'data': {'header': {'seq': msg.header.seq,
                                   'stamp': {'secs': stamp.secs, 'nsecs': stamp.nsecs},
                                   'frame_id': msg.header.frame_id},
                        **{'rotorPWM'+str(i): getattr(msg, 'rotorPWM'+str(i)) for i in range(4)}}}
        with lock:
            rows.append(row)
    listener = rospy.Subscriber('/airsim_node/drone_1/rotor_pwm_cmd', RotorPWM, receive, queue_size=1000)
    (run/'second_wire_ready').touch(exist_ok=False)
    deadline = time.monotonic()+60
    try:
        while not rospy.is_shutdown() and not (run/'second_wire_stop').exists() and time.monotonic() < deadline:
            time.sleep(.02)
    finally:
        listener.unregister()
        with lock:
            with (run/'second_wire_commands.jsonl').open('x') as output:
                for row in rows:
                    output.write(json.dumps(row)+'\n')
    return 0 if (run/'second_wire_stop').exists() else 2


if __name__ == '__main__':
    raise SystemExit(main())
