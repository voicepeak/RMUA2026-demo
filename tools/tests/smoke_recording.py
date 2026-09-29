#!/usr/bin/env python3
"""ROS integration check on a private master with synthetic poses; no simulator."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time


def main():
    repo = Path(__file__).resolve().parents[2]
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%s" % port
    os.environ["ROS_HOSTNAME"] = "127.0.0.1"
    os.environ["ROS_PACKAGE_PATH"] = str(repo / "ros_ws/src") + ":/opt/ros/noetic/share"
    os.environ["CMAKE_PREFIX_PATH"] = "/opt/ros/noetic"
    folder = Path(tempfile.mkdtemp(prefix="rmua_recording_smoke_"))
    done = threading.Event()
    with (folder / "master.log").open("w") as log:
        master = subprocess.Popen(["roscore", "-p", str(port)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            import rosgraph
            deadline = time.monotonic() + 15
            while not rosgraph.is_master_online():
                if time.monotonic() > deadline:
                    raise RuntimeError("private master did not start")
                time.sleep(.1)
            import rospy
            from geometry_msgs.msg import PoseStamped
            rospy.init_node("synthetic_pose_fixture", disable_signals=True)
            pub = rospy.Publisher("/airsim_node/drone_1/debug/pose_gt", PoseStamped, queue_size=10)
            def publish():
                while not done.is_set():
                    msg = PoseStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "world_ned"
                    msg.pose.position.x = 2.5
                    msg.pose.position.z = -1
                    msg.pose.orientation.w = 1
                    pub.publish(msg)
                    done.wait(.05)
            thread = threading.Thread(target=publish, daemon=True)
            thread.start()
            result = subprocess.run([
                "python3", str(repo / "tools/run_experiment.py"), "--scene", "SYNTHETIC_TEST_ONLY",
                "--seed", "0", "--runtime-src", str(repo / "ros_ws/src"),
                "--runs-dir", str(folder / "runs"), "--timeout", "5", "--execute"],
                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
            print(result.stdout)
            assert result.returncode == 0, result.stdout
            run = next((folder / "runs").iterdir())
            manifest = json.loads((run / "manifest.json").read_text())
            summary = json.loads((run / "summary.json").read_text())
            assert manifest["termination_reason"] == "WALL_TIMEOUT", manifest["termination_reason"]
            assert manifest["bag_finalized"], manifest
            assert (run / "params_running.yaml").is_file()
            assert summary["pose_samples"] > 10, summary
            for topic in ("/rmua/controller/telemetry", "/airsim_node/drone_1/vel_body_cmd"):
                assert summary["topic_message_counts"].get(topic, 0) > 0, summary
            assert summary["race_success"] == "UNKNOWN"
            assert any(event["reason"] == "ROS_SHUTDOWN" for event in summary["controller_terminations"])
            import rosbag
            with rosbag.Bag(str(run / "telemetry.bag")) as bag:
                commands = [message for _, message, _ in bag.read_messages(
                    topics=["/airsim_node/drone_1/vel_body_cmd"])]
            assert commands and (commands[-1].vx, commands[-1].vy, commands[-1].vz, commands[-1].yawRate) == (0, 0, 0, 0)
            print("ROS recording smoke PASS:", run)
        finally:
            done.set()
            os.killpg(master.pid, signal.SIGINT)
            master.wait(timeout=15)


if __name__ == "__main__":
    main()
