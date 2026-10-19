"""Host provenance + existing-container read-only observation, no dependency install."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess

from evaluation.seed_runner.manifest import prepare_run, source_hashes


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--duration", type=float, default=10.)
    parser.add_argument("--output", type=Path, default=root / "runs")
    parser.add_argument("--config", type=Path, default=root / "configs/official_baseline.yaml")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 120:
        parser.error("duration must be in (0, 120] seconds")
    try:
        args.output.resolve().relative_to(root)
    except ValueError:
        parser.error("output must be inside main so the existing container can access it")
    config = json.loads(args.config.read_text())
    container = os.environ.get("RMUA_CONTAINER", "rmua_noetic")
    digest = subprocess.check_output(["docker", "inspect", container, "--format", "{{.Image}}"], text=True).strip()
    run, manifest = prepare_run(root, args.output, args.seed, digest, config)
    container_run = Path("/workspace/main") / run.relative_to(root)
    code = 1
    try:
        code = subprocess.call([str(root / "tools/enter_env.sh"), "python3", "-m",
                                "adapters.ros_noetic.interface_audit", "--run-dir", str(container_run),
                                "--duration", str(args.duration)])
    finally:
        manifest = json.loads((run / "manifest.json").read_text())
        manifest["source_sha256_after"] = source_hashes(root)
        manifest["source_unchanged"] = manifest["source_sha256_before"] == manifest["source_sha256_after"]
        manifest["observer_exit_code"] = code
        (run / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
