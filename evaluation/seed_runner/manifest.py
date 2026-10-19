"""Exclusive run directories and complete source/config hashes; no flight execution."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import uuid


def file_hash(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def source_hashes(root):
    names = subprocess.check_output(
        ["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"], cwd=root)
    result = {}
    for name in sorted(set(n.decode() for n in names.split(b"\0") if n)):
        path = Path(root) / name
        if path.is_file():
            result[name] = file_hash(path)
    return result


def prepare_run(root, output, seed, image_digest, config):
    root = Path(root).resolve()
    run = Path(output).resolve() / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_")
                                  + uuid.uuid4().hex)
    run.mkdir(parents=True, exist_ok=False)
    status = subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True)
    manifest = {
        "schema_version": 1, "seed": seed, "seed_evidence": "operator_declared",
        "git_sha": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        "git_status": status, "image_digest": image_digest,
        "config": config, "source_sha256_before": source_hashes(root),
        "model_hashes": {}, "telemetry": [], "safety_events": [],
        "official_result": "UNKNOWN", "stage": "P0_READ_ONLY_AUDIT",
    }
    (run / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return run, manifest
