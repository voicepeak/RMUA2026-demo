"""Host provenance wrapper for read-only formal-input state observation."""
import argparse
import json
import os
from pathlib import Path
import subprocess
from evaluation.seed_runner.manifest import prepare_run, source_hashes


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--duration", type=float, default=10.)
    args = parser.parse_args()
    if not 0 < args.duration <= 120:
        parser.error("duration must be in (0,120]")
    container = os.environ.get("RMUA_CONTAINER", "rmua_noetic")
    digest = subprocess.check_output(["docker", "inspect", container, "--format", "{{.Image}}"], text=True).strip()
    config = json.loads((root/"configs/official_baseline.yaml").read_text())
    run, manifest = prepare_run(root, root/"runs", args.seed, digest, config)
    manifest["stage"] = "P0_FORMAL_STATE_OBSERVATION"
    (run/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print(run, flush=True)
    code = 1
    try:
        code = subprocess.call([str(root/"tools/enter_env.sh"), "python3", "-m",
                                "adapters.ros_noetic.state_reader", "--run-dir",
                                "/workspace/main/"+str(run.relative_to(root)),
                                "--duration", str(args.duration)])
    finally:
        manifest["source_sha256_after"] = source_hashes(root)
        manifest["source_unchanged"] = manifest["source_sha256_before"] == manifest["source_sha256_after"]
        manifest["observer_exit_code"] = code
        manifest["telemetry"] = ["state.jsonl", "state_summary.json"]
        (run/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
