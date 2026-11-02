"""Read-only front camera HUD recorder; never feeds flight control."""
import argparse
import json
import math
from pathlib import Path
import os
import subprocess
import time


def main():
    import cv2
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    parser = argparse.ArgumentParser()
    parser.add_argument('output')
    args = parser.parse_args()
    rospy.init_node('alpha_hud', anonymous=True)
    bridge = CvBridge()
    writer = None
    count = 0
    last = 0.0

    def frame(msg):
        nonlocal writer, count, last
        now = time.monotonic()
        if now-last < .1:
            return
        last = now
        img = bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]
        if writer is None:
            writer = subprocess.Popen([os.environ.get('ALPHA_FFMPEG', 'ffmpeg'), '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{w}x{h}', '-r', '10', '-i', '-', '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', args.output], stdin=subprocess.PIPE)
        try:
            status = json.loads((Path(args.output).parent/'hud_state.json').read_text())
            velocity = status['velocity_ned']
            speed = math.sqrt(sum(v*v for v in velocity))
            text_status = 'state: %s | speed: %.2f m/s | phase: %s' % (status['health'], speed, status['phase'])
        except (OSError, ValueError, KeyError, TypeError):
            text_status = 'state/speed/phase: unavailable'
        # Gate geometry is shown only when an actual detector supplies it.
        try:
            gates = json.loads((Path(args.output).parent/'hud_gates.json').read_text())
            for quad in gates:
                import numpy as np
                cv2.polylines(img, [np.asarray(quad, dtype=np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
            gate_status = 'gates: %d' % len(gates)
        except (OSError, ValueError, TypeError):
            gate_status = 'gates: unavailable'
        for i, text in enumerate(('P0 PWM | official stage: unknown | score: unknown', 'target: placeholder | '+gate_status, text_status)):
            cv2.putText(img, text, (12, 26+25*i), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 1, cv2.LINE_AA)
        writer.stdin.write(img.tobytes())
        count += 1
    rospy.Subscriber('/airsim_node/drone_1/front_left/Scene', Image, frame, queue_size=1, buff_size=2**24)
    try:
        rospy.spin()
    finally:
        if writer:
            writer.stdin.close()
            writer.wait(timeout=15)
        print('HUD frames:', count, flush=True)


if __name__ == '__main__':
    main()
