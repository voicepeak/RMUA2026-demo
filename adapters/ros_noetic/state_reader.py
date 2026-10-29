"""Read-only ESKF observer. Subscribes to official initial pose, GPS and IMU only."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import threading
import time

from estimation.clock import SensorClock
from estimation.imu_gps import ImuGpsEstimator


def pose_values(pose):
    p, q = pose.position, pose.orientation
    return (p.x, p.y, p.z), (q.w, q.x, q.y, q.z)


def observe(run, duration):
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Imu

    rospy.init_node("rmua_next_state_reader", anonymous=True, disable_signals=True)
    lock = threading.RLock()
    clock = SensorClock()
    estimator = ImuGpsEstimator()
    events = Counter()
    initial = None
    health = Counter()
    count = 0
    with (run / "state.jsonl").open("x") as output:
        def initial_pose(msg):
            nonlocal initial
            with lock:
                try:
                    initial = pose_values(msg.pose)
                except ValueError:
                    events["INVALID_INITIAL"] += 1

        def imu(msg):
            nonlocal estimator, initial, count
            with lock:
                try:
                    stamp = msg.header.stamp.to_sec()
                    previous = clock.stamp
                    if clock.observe(stamp, time.monotonic()):
                        estimator = ImuGpsEstimator(clock.epoch)
                        initial = None  # Require a newly received initial pose after reset.
                        events["CLOCK_RESET"] += 1
                    elif previous == stamp:
                        events["IMU_DUPLICATE"] += 1
                        return
                    if initial is None:
                        events["IMU_BEFORE_INITIAL"] += 1
                        return
                    if initial is not None and not estimator.initialized:
                        estimator.initialize(*initial)
                    a, w = msg.linear_acceleration, msg.angular_velocity
                    estimator.propagate(stamp, (a.x, a.y, a.z), (w.x, w.y, w.z))
                    if estimator.initialized:
                        state = estimator.snapshot()
                        output.write(json.dumps(asdict(state), allow_nan=False)+"\n")
                        health[state.health.value] += 1
                        count += 1
                except (ValueError, ArithmeticError) as error:
                    events["INVALID_IMU:"+str(error)] += 1

        def gps(msg):
            with lock:
                try:
                    stamp = msg.header.stamp.to_sec()
                    if stamp <= 0 or not clock.fresh(time.monotonic()):
                        events["GPS_NO_VALID_CLOCK"] += 1
                        return
                    estimator.update_gps(stamp, *pose_values(msg.pose))
                except (ValueError, ArithmeticError) as error:
                    events["INVALID_GPS:"+str(error)] += 1

        subscribers = [
            rospy.Subscriber("/airsim_node/initial_pose", PoseStamped, initial_pose, queue_size=1),
            rospy.Subscriber("/airsim_node/drone_1/imu/imu", Imu, imu, queue_size=1),
            rospy.Subscriber("/airsim_node/drone_1/gps", PoseStamped, gps, queue_size=1),
        ]
        deadline = time.monotonic()+duration
        try:
            while time.monotonic() < deadline and not rospy.is_shutdown():
                time.sleep(.02)
        finally:
            for subscriber in subscribers:
                subscriber.unregister()
        with lock:
            result = {"state_count": count, "health_counts": dict(health),
                      "events": dict(events), "gps_rejections": estimator.rejections,
                      "epoch": clock.epoch, "control_published": False,
                      "noise_calibrated": False, "official_result": "UNKNOWN"}
            if count:
                result["last_state"] = asdict(estimator.snapshot())
            (run / "state_summary.json").write_text(json.dumps(result, indent=2)+"\n")
    return 0 if count else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=10.)
    args = parser.parse_args()
    if not 0 < args.duration <= 120:
        parser.error("duration must be in (0,120]")
    return observe(args.run_dir, args.duration)


if __name__ == "__main__":
    raise SystemExit(main())
