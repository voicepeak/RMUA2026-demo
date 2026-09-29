#!/usr/bin/env python3
"""Prepare a reproducible run; --execute records all ROS topics then starts control.

Run inside the sourced ROS environment. Seed/scene are operator declarations,
not a command to reset or start the simulator. Never silently synchronizes sources.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid

from audit_course import source_diff


def write_json(path, data):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def command_output(argv, cwd=None):
    return subprocess.check_output(argv, cwd=cwd, text=True, stderr=subprocess.STDOUT).strip()


def snapshot(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(str(source), str(destination))
    return hashlib.sha256(destination.read_bytes()).hexdigest()


def prepare(args):
    repo = Path(__file__).resolve().parents[1]
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid.uuid4().hex[:8]
    run = args.runs_dir.resolve() / run_id
    run.mkdir(parents=True)
    hashes = {}
    for root in (repo / "ros_ws/src", repo / "tools"):
        for path in sorted(root.rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
                relative = str(path.relative_to(repo))
                hashes[relative] = snapshot(path, run / "inputs" / relative)
    for name, path in (("model", args.model), ("simulator_config", args.simulator_config),
                       ("evaluation_gates", args.evaluation_gates)):
        if path:
            hashes[name] = snapshot(path, run / "inputs" / (name + path.suffix))
    # Preserve overridden route/gate/guide/capability files as well as repository defaults.
    for assignment in args.launch_arg:
        key, value = assignment.split(":=", 1)
        if key in ("route_file", "gates_file", "guides_file", "vz_capability_file"):
            path = Path(value)
            hashes["override_" + key] = snapshot(path, run / "inputs" / ("override_" + key + path.suffix))
    drift = source_diff(repo / "ros_ws/src", args.runtime_src)
    drift["only_runtime"] = [p for p in drift["only_runtime"] if p != "CMakeLists.txt"]
    # roslaunch rejects duplicate arg declarations; overrides replace defaults here.
    overrides = {"cruise_speed": "3.0", "normal_speed_floor": "3.0"}
    overrides.update(dict(value.split(":=", 1) for value in args.launch_arg))
    launch = ["roslaunch", str(args.runtime_src.resolve() / "route_follower/launch/route_follower.launch")]
    launch += [key + ":=" + value for key, value in overrides.items()]
    manifest = {"schema_version": 1, "run_id": run_id, "state": "PREPARED",
                "scene": args.scene, "seed": args.seed,
                "scene_seed_evidence": "operator_declared_unverified",
                "source_commit": command_output(["git", "rev-parse", "HEAD"], repo) if shutil.which("git") else None,
                "source_status": command_output(["git", "status", "--short"], repo) if shutil.which("git") else "git_unavailable; use input_sha256",
                "input_sha256": hashes, "runtime_source_diff": drift,
                "runtime_src": str(args.runtime_src.resolve()), "launch_argv": launch,
                "runner_argv": list(os.sys.argv), "timeout_wall_s": args.timeout,
                "official_result": "UNKNOWN", "termination_reason": None}
    write_json(run / "manifest.json", manifest)
    return run, manifest


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def execute(args, run, manifest):
    recorder = controller = None
    handles = []
    old_handlers = {}
    def interrupted(signum, _frame):
        raise KeyboardInterrupt("signal %s" % signum)
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, interrupted)
        if any(manifest["runtime_source_diff"].values()):
            raise RuntimeError("runtime source differs; review and synchronize before flight")
        for binary in ("rosbag", "roslaunch", "rosparam", "rostopic", "rosnode", "rospack"):
            if not shutil.which(binary):
                raise RuntimeError("missing %s; source ROS and runtime workspace" % binary)
        expected_package = (args.runtime_src / "route_follower").resolve()
        if Path(command_output(["rospack", "find", "route_follower"])).resolve() != expected_package:
            raise RuntimeError("ROS resolves a different route_follower workspace")
        import rosgraph
        import rospy
        from std_msgs.msg import String
        master = rosgraph.Master("/run_experiment")
        topics = dict(master.getPublishedTopics("/"))
        pose_topic = "/airsim_node/drone_1/debug/pose_gt"
        if topics.get(pose_topic) != "geometry_msgs/PoseStamped":
            raise RuntimeError("simulator pose topic is not available")
        publishers, _, _ = master.getSystemState()
        if any(topic == "/airsim_node/drone_1/vel_body_cmd" and nodes for topic, nodes in publishers):
            raise RuntimeError("an existing velocity publisher is active")
        rospy.init_node("run_experiment", anonymous=True, disable_signals=True)
        terminations = []
        def controller_event(message):
            payload = json.loads(message.data)
            if payload.get("kind") == "TERMINATION":
                terminations.append(payload)
        event_subscription = rospy.Subscriber("/rmua/controller/events", String, controller_event)
        write_json(run / "topics_before.json", topics)
        subprocess.check_call(["rosparam", "dump", str(run / "params_before.yaml")])
        recorder_name = "rmua_record_" + manifest["run_id"].lower()
        bag_argv = ["rosbag", "record", "-a", "--lz4", "--buffsize=512", "-O",
                    str(run / "telemetry.bag"), "__name:=" + recorder_name]
        manifest["record_argv"] = bag_argv
        handle = (run / "recorder.log").open("w")
        handles.append(handle)
        recorder = subprocess.Popen(bag_argv, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 15
        while True:
            _, subscribers, _ = master.getSystemState()
            if any(topic == pose_topic and "/" + recorder_name in nodes for topic, nodes in subscribers):
                break
            if recorder.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("recorder failed to subscribe before controller start")
            time.sleep(0.1)
        handle = (run / "controller.log").open("w")
        handles.append(handle)
        controller = subprocess.Popen(manifest["launch_argv"], stdout=handle,
                                      stderr=subprocess.STDOUT, start_new_session=True)
        manifest.update(state="RUNNING", started_utc=datetime.now(timezone.utc).isoformat())
        write_json(run / "manifest.json", manifest)
        deadline = time.monotonic() + args.timeout
        saved_params = False
        while controller.poll() is None:
            if recorder.poll() is not None:
                raise RuntimeError("recorder exited during control")
            if not saved_params and master.hasParam("/route_follower/control_rate"):
                subprocess.check_call(["rosparam", "dump", str(run / "params_running.yaml")])
                saved_params = True
            if terminations:
                manifest["termination_reason"] = "CONTROLLER_" + terminations[0]["reason"]
                break
            if time.monotonic() > deadline:
                manifest["termination_reason"] = "WALL_TIMEOUT"
                break
            time.sleep(0.2)
        else:
            manifest["termination_reason"] = "CONTROLLER_EXIT"
            manifest["controller_exit_code"] = controller.returncode
            raise RuntimeError("controller launch exited without an observed completion event (code %s)" % controller.returncode)
        manifest["state"] = "FINISHED"
    except KeyboardInterrupt:
        manifest.update(state="INTERRUPTED", termination_reason="OPERATOR_INTERRUPT")
    except Exception as exc:
        manifest.update(state="ERROR", error=str(exc))
        if not manifest.get("termination_reason"):
            manifest["termination_reason"] = "RUNNER_ERROR"
    finally:
        # roslaunch propagates SIGINT; recorder stops last so final commands are captured.
        for signum in old_handlers:
            signal.signal(signum, signal.SIG_IGN)
        stop(controller)
        stop(recorder)
        for handle in handles:
            handle.close()
        manifest["finished_utc"] = datetime.now(timezone.utc).isoformat()
        manifest["bag_finalized"] = (run / "telemetry.bag").is_file()
        if manifest["bag_finalized"]:
            try:
                from export_run import export
                export(run, args.evaluation_gates)
            except Exception as exc:
                manifest["export_error"] = str(exc)
                manifest["state"] = "ERROR"
        write_json(run / "manifest.json", manifest)
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
    return 1 if manifest["state"] == "ERROR" else 0


def main():
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--seed", required=True, type=int)
    parser.add_argument("--runtime-src", type=Path, default=repo.parent / "rmua_ws/src")
    parser.add_argument("--runs-dir", type=Path, default=repo.parent / "runs")
    parser.add_argument("--model", type=Path)
    parser.add_argument("--simulator-config", type=Path)
    parser.add_argument("--evaluation-gates", type=Path)
    parser.add_argument("--timeout", type=float, default=600)
    parser.add_argument("--launch-arg", action="append", default=[])
    parser.add_argument("--execute", action="store_true", help="start recording and control; default prepares only")
    args = parser.parse_args()
    if not 0 < args.timeout < float("inf"):
        parser.error("timeout must be finite and positive")
    if any(":=" not in value or not value.split(":=", 1)[0] for value in args.launch_arg):
        parser.error("--launch-arg expects name:=value")
    try:
        run, manifest = prepare(args)
        print(run, flush=True)
        if args.execute:
            result = execute(args, run, manifest)
            print(manifest["termination_reason"] + ": " + manifest.get("error", ""))
            return result
        return 0
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        parser.exit(2, "run preparation error: %s\n" % exc)


if __name__ == "__main__":
    raise SystemExit(main())
