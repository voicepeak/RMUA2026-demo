"""Prepare reviewed simulation data; publish deterministic, verified public shards.

Raw source archives are read only. Credentials are read by huggingface_hub, never
accepted as command arguments, logged, or included in the publication directory.
"""
import argparse
from collections import Counter
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tarfile
import time

ROOTS = {"experiments", "frames", "runs", "tracked_iteration_artifacts"}
SKIP_DIRS = {
    ".git", "__pycache__", ".cache", ".venv", "venv", "node_modules",
    "build", "devel", "install", "checkouts", "controller_sources",
    "vision_sources", "tool_sources", "sim_launch_sources", "driver_root",
    "runtime_before_demo", "native", "weights",
}
SUFFIXES = {
    ".jpg", ".jpeg", ".png", ".webp", ".bmp", ".tif", ".tiff", ".xwd",
    ".npz", ".npy", ".pcd", ".bag", ".csv", ".json", ".jsonl",
    ".yaml", ".yml", ".txt", ".log", ".md",
}
TEXT = {".csv", ".json", ".jsonl", ".yaml", ".yml", ".txt", ".log", ".md"}
SECRET = re.compile(
    rb"hf_[A-Za-z0-9]{20,}|gh[pousr]_[A-Za-z0-9]{20,}|"
    rb"github_pat_[A-Za-z0-9_]{20,}|AKIA[A-Z0-9]{16}|"
    rb"-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----|"
    rb"(?i:authorization\s*[:=]\s*[\"']?bearer\s+[A-Za-z0-9._-]{16,})|"
    rb"https?://[^\s/:@]{1,100}:[^\s/@]{1,200}@"
)


def write_json(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    tmp.replace(path)


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def contains_secret(path):
    tail = b""
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            combined = tail + block
            if SECRET.search(combined):
                return True
            tail = combined[-4096:]
    return False


def scan(root):
    root = Path(root).resolve()
    accepted, excluded = [], []
    reasons = Counter()
    last = time.monotonic()
    for top in sorted(ROOTS):
        base = root / top
        if not base.is_dir():
            continue
        for directory, dirs, files in os.walk(base, followlinks=False):
            current = Path(directory)
            kept = []
            for name in sorted(dirs):
                p = current / name
                if p.is_symlink() or name.lower() in SKIP_DIRS:
                    reasons["excluded_directory"] += 1
                else:
                    kept.append(name)
            dirs[:] = kept
            for name in sorted(files):
                p = current / name
                rel = p.relative_to(root).as_posix()
                reason = None
                if p.is_symlink():
                    reason = "symlink"
                elif p.suffix.lower() not in SUFFIXES:
                    reason = "not_observation_or_metadata"
                elif name.startswith(".") or any(v in name.lower() for v in (
                        "credential", "private_key", "id_rsa", "id_ed25519", ".env")):
                    reason = "credential_filename"
                elif p.suffix.lower() in TEXT and contains_secret(p):
                    reason = "possible_credential_content"
                if reason:
                    reasons[reason] += 1
                    excluded.append({"path": rel, "reason": reason})
                    continue
                info = p.stat()
                accepted.append({"path": rel, "size": info.st_size, "mtime_ns": info.st_mtime_ns})
            if time.monotonic() - last > 20:
                print("Scanned eligible files:", len(accepted), flush=True)
                last = time.monotonic()
    return sorted(accepted, key=lambda r: r["path"]), excluded, dict(reasons)


def prepare(root, output, target):
    output.mkdir(parents=True, exist_ok=True)
    plan_path = output / "plan.json"
    if plan_path.exists():
        raise FileExistsError("Existing plan: use pack/upload to resume, or choose a new output")
    entries, excluded, reasons = scan(root)
    if not entries:
        raise ValueError("No eligible observations")
    shards, group, size = [], [], 0
    for entry in entries:
        # Estimate tar header, PAX header, padded payload; bounded near target.
        estimate = ((entry["size"] + 511) // 512 + 4) * 512
        if group and size + estimate > target:
            shards.append({"name": "data-%05d.tar" % len(shards), "entries": group})
            group, size = [], 0
        group.append(entry)
        size += estimate
    if group:
        shards.append({"name": "data-%05d.tar" % len(shards), "entries": group})
    plan = {"schema_version": 1, "source_root": str(root.resolve()),
            "file_count": len(entries), "source_bytes": sum(e["size"] for e in entries),
            "target_shard_bytes": target, "excluded_counts": reasons, "shards": shards}
    write_json(plan_path, plan)
    write_json(output / "exclusions.local.json", excluded)
    public = output / "public"
    public.mkdir(exist_ok=True)
    write_json(public / "dataset_summary.json", {
        k: v for k, v in plan.items() if k not in {"source_root", "shards"}})
    write_json(public / "shard_plan.json", [
        {"name": s["name"], "file_count": len(s["entries"]),
         "source_bytes": sum(e["size"] for e in s["entries"])} for s in shards])
    (public / "README.md").write_text("""---
pretty_name: RMUA 2026 Legacy Simulation Observations
language:
- zh
tags:
- robotics
- drone
- simulation
- stereo
- lidar
---

# RMUA 2026 legacy simulation observations

Historical development observations and telemetry from RMUA simulator experiments.
This dataset contains successful and failed development runs. Directory names,
local gate ledgers and screenshots are not certified official competition results.
Seeds and configurations are recorded where present in each run's original metadata;
missing values remain unknown. These historical runs are development data, not an
independent held-out evaluation split. Do not randomly split adjacent video frames.

Images, NPZ point/trace arrays, telemetry, configurations and related records retain
their original relative paths inside deterministic tar shards. This is a file archive,
not a claim of synchronized, calibrated or uniformly labeled training samples.
Original source code is maintained separately in voicepeak/RMUA2026-demo on GitHub.
Generated data and source references do not relicense any third-party materials.

## Layout and integrity

- `data/data-NNNNN.tar`: observations with original relative paths.
- `indices/data-NNNNN.tar.files.jsonl.gz`: each member's path, size and SHA256.
- `shard_plan.json`: complete planned set of shards.
- `shards.json`: committed and checksum-verified shards; compare with the plan.
- `dataset_summary.json`: selected data volume and exclusion counts.

Uploads are incremental. The dataset is complete only when `publication_status.json`
reports `COMPLETE`; do not assume a newly created repository is a complete archive.
Download selected shards from Hugging Face, verify their SHA256 from `shards.json`,
and extract into a new directory with `tar -xf data-NNNNN.tar -C <destination>`.
No symlinks, absolute paths or parent-directory members are included.

Git repositories, repeated code snapshots, model weights, drivers, executable packages,
caches, credential filenames and text containing recognized credential patterns are
excluded. Automated pattern scanning is not a guarantee of exhaustive detection.
Raw local archives are retained; publishing never deletes source observations.
""")
    write_json(public / "publication_status.json", {"state": "PREPARED", "shards_expected": len(shards)})
    print(json.dumps({k: v for k, v in plan.items() if k not in {"source_root", "shards"}}), flush=True)
    print("Planned shards:", len(shards), flush=True)
    return plan


class HashingReader:
    def __init__(self, source):
        self.source, self.digest = source, hashlib.sha256()

    def read(self, size=-1):
        value = self.source.read(size)
        self.digest.update(value)
        return value


def pack(plan, output, shard):
    staging = output / "staging"
    staging.mkdir(exist_ok=True)
    target = staging / shard["name"]
    index = staging / (shard["name"] + ".files.jsonl.gz")
    receipt = staging / (shard["name"] + ".receipt.json")
    if target.exists() and index.exists() and receipt.exists():
        info = json.loads(receipt.read_text())
        if sha256(target) == info["sha256"] and sha256(index) == info["index_sha256"]:
            return target, index, info
        raise ValueError("Existing staging shard does not match its receipt")
    required = sum(e["size"] + 2048 for e in shard["entries"])
    if shutil.disk_usage(output).free < required + 1024 ** 3:
        raise RuntimeError("Insufficient free disk space for staging one shard")
    root = Path(plan["source_root"])
    tmp = target.with_suffix(".tar.tmp")
    index_tmp = index.with_suffix(".gz.tmp")
    with tarfile.open(tmp, "w", format=tarfile.PAX_FORMAT) as tar, \
            index_tmp.open("wb") as raw, gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as z:
        for entry in shard["entries"]:
            rel = Path(entry["path"])
            if rel.is_absolute() or ".." in rel.parts:
                raise ValueError("Invalid archive member path")
            source = root / rel
            before = source.stat()
            if source.is_symlink() or (before.st_size, before.st_mtime_ns) != (entry["size"], entry["mtime_ns"]):
                raise RuntimeError("Source changed since review: " + str(rel))
            info = tarfile.TarInfo(entry["path"])
            info.size, info.mode, info.mtime = entry["size"], 0o644, 0
            with source.open("rb") as handle:
                reader = HashingReader(handle)
                tar.addfile(info, reader)
            after = source.stat()
            if (after.st_size, after.st_mtime_ns) != (before.st_size, before.st_mtime_ns):
                raise RuntimeError("Source changed during packing: " + str(rel))
            z.write((json.dumps({"path": entry["path"], "size": entry["size"],
                                "sha256": reader.digest.hexdigest()}) + "\n").encode())
    tmp.replace(target)
    index_tmp.replace(index)
    result = {"name": shard["name"], "size": target.stat().st_size,
              "sha256": sha256(target), "file_count": len(shard["entries"]),
              "index_size": index.stat().st_size,
              "index_sha256": sha256(index)}
    write_json(receipt, result)
    return target, index, result


def verify_remote(api, repo_id, path, size, digest, revision="main"):
    from huggingface_hub import hf_hub_url
    rows = api.get_paths_info(repo_id, [path], repo_type="dataset", revision=revision)
    if len(rows) != 1 or rows[0].size != size:
        raise RuntimeError("Remote size mismatch: " + path)
    lfs = rows[0].lfs
    if lfs is not None:
        if lfs.sha256 != digest:
            raise RuntimeError("Remote SHA256 mismatch: " + path)
        return
    # Small files may be regular Git blobs. Read them back to verify their content.
    import urllib.request
    h = hashlib.sha256()
    url = hf_hub_url(repo_id, path, repo_type="dataset", revision=revision)
    with urllib.request.urlopen(url, timeout=60) as response:
        for block in iter(lambda: response.read(1024 * 1024), b""):
            h.update(block)
    if h.hexdigest() != digest:
        raise RuntimeError("Remote content mismatch: " + path)


def publish(output, plan, repo_id):
    from huggingface_hub import CommitOperationAdd, HfApi, get_token
    if not get_token():
        raise RuntimeError("Hugging Face login required; run hf auth login locally")
    api = HfApi()
    who = api.whoami()
    owner = repo_id.split("/")[0]
    if owner.lower() != who["name"].lower():
        raise RuntimeError("Logged-in account does not match requested owner: " + who["name"])
    # Use authenticated canonical casing; never change another existing private repo.
    repo_id = who["name"] + "/" + repo_id.split("/", 1)[1]
    api.create_repo(repo_id, repo_type="dataset", private=False, exist_ok=True)
    if api.repo_info(repo_id, repo_type="dataset").private:
        raise RuntimeError("Existing repository is private; refusing to change its visibility implicitly")
    public = output / "public"
    state_file = output / "upload_state.local.json"
    state = json.loads(state_file.read_text()) if state_file.exists() else {"repo_id": repo_id, "verified": {}}
    if state["repo_id"] != repo_id:
        raise RuntimeError("Existing upload state belongs to another repository")
    write_json(public / "shards.json", list(state["verified"].values()))
    write_json(public / "publication_status.json", {"state": "UPLOADING", "shards_expected": len(plan["shards"])})
    api.upload_folder(repo_id=repo_id, repo_type="dataset", folder_path=public,
                      commit_message="Document public legacy simulation dataset and shard plan")
    for shard in plan["shards"]:
        name = shard["name"]
        if name in state["verified"]:
            existing = state["verified"][name]
            verify_remote(api, repo_id, "data/" + name, existing["size"], existing["sha256"])
            verify_remote(api, repo_id, "indices/" + name + ".files.jsonl.gz",
                          existing["index_size"], existing["index_sha256"])
            print("Already verified:", name, flush=True)
            continue
        target, index, receipt = pack(plan, output, shard)
        commit = api.create_commit(repo_id, repo_type="dataset", operations=[
            CommitOperationAdd(path_in_repo="data/" + name, path_or_fileobj=target),
            CommitOperationAdd(path_in_repo="indices/" + index.name, path_or_fileobj=index),
        ], commit_message="Add observation shard " + name)
        verify_remote(api, repo_id, "data/" + name, receipt["size"], receipt["sha256"], commit.oid)
        verify_remote(api, repo_id, "indices/" + index.name, index.stat().st_size, receipt["index_sha256"], commit.oid)
        receipt["revision"] = commit.oid
        state["verified"][name] = receipt
        write_json(state_file, state)
        write_json(public / "shards.json", list(state["verified"].values()))
        api.upload_file(repo_id=repo_id, repo_type="dataset", path_or_fileobj=public / "shards.json",
                        path_in_repo="shards.json", commit_message="Record verified shard " + name)
        # Only disposable staging copies are removed. Original archived data is untouched.
        target.unlink()
        index.unlink()
        print("Verified remote shard:", name, flush=True)
    api.upload_file(repo_id=repo_id, repo_type="dataset", path_or_fileobj=public / "shards.json",
                    path_in_repo="shards.json", commit_message="Finalize verified shard index")
    write_json(public / "publication_status.json", {"state": "COMPLETE", "shards_expected": len(plan["shards"]),
                                                   "shards_verified": len(state["verified"])})
    api.upload_file(repo_id=repo_id, repo_type="dataset", path_or_fileobj=public / "publication_status.json",
                    path_in_repo="publication_status.json", commit_message="Complete verified archive publication")
    print("Published:", "https://huggingface.co/datasets/" + repo_id, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("prepare", "pack", "upload"))
    parser.add_argument("--source", type=Path, default=Path("/home/tianbot/RUMA-legacy-archive-20261019"))
    parser.add_argument("--output", type=Path, default=Path("/home/tianbot/RUMA-hf-public-20261019"))
    parser.add_argument("--shard-gib", type=float, default=1.0)
    parser.add_argument("--shard-index", type=int, default=0)
    parser.add_argument("--repo-id", default="Voicepeak/rmua2026-legacy-data")
    args = parser.parse_args()
    if not 0 < args.shard_gib <= 4:
        parser.error("shard-gib must be in (0, 4]")
    if args.action == "prepare":
        prepare(args.source, args.output, int(args.shard_gib * 1024 ** 3))
    else:
        plan = json.loads((args.output / "plan.json").read_text())
        if args.action == "pack":
            _, _, receipt = pack(plan, args.output, plan["shards"][args.shard_index])
            print(json.dumps(receipt, indent=2))
        else:
            publish(args.output, plan, args.repo_id)


if __name__ == "__main__":
    main()
