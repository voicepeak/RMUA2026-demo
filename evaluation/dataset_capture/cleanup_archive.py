"""Delete only unchanged, publicly backed-up archive members after full verification."""
import argparse
import fcntl
import gzip
import hashlib
import itertools
import json
import os
from pathlib import Path
import shutil
import stat
import tempfile
import time
import urllib.request

from evaluation.dataset_capture.publish_archive import sha256, write_json


class PublicationPending(ValueError):
    pass


class PublicArchive:
    def __init__(self, repo_id):
        for name in ("ALL_PROXY", "all_proxy"):
            if os.environ.get(name, "").startswith("socks://"):
                os.environ.pop(name)
        from huggingface_hub import HfApi
        self.api = HfApi(token=False)
        self.repo_id = repo_id
        info = self.api.repo_info(repo_id, repo_type="dataset")
        if info.private:
            raise ValueError("Expected the authorized public dataset")
        self.revision = info.sha

    def read(self, path):
        from huggingface_hub import hf_hub_url
        url = hf_hub_url(self.repo_id, path, repo_type="dataset", revision=self.revision)
        with urllib.request.urlopen(url, timeout=60) as response:
            return response.read()

    def metadata(self, paths):
        rows = self.api.get_paths_info(self.repo_id, paths, repo_type="dataset", revision=self.revision)
        return {r.path: {"size": r.size, "sha256": r.lfs.sha256 if r.lfs else None}
                for r in rows}


def verify_publication(plan, state, remote, output, shard_names=None):
    expected = {s["name"] for s in plan["shards"]}
    partial = shard_names is not None
    selected = set(shard_names) if partial else expected
    if not selected or not selected <= expected or not selected <= set(state["verified"]):
        raise ValueError("Requested shards are not locally verified")
    if len(expected) != len(plan["shards"]) or (not partial and set(state["verified"]) != expected):
        raise ValueError("Local upload receipts are incomplete")
    status = json.loads(remote.read("publication_status.json"))
    if not partial and (status.get("state") != "COMPLETE" or status.get("shards_expected") != len(expected)
                        or status.get("shards_verified") != len(expected)):
        raise PublicationPending("Remote publication is not COMPLETE")
    summary = json.loads(remote.read("dataset_summary.json"))
    for key in ("file_count", "source_bytes"):
        if summary.get(key) != plan[key]:
            raise ValueError("Remote summary does not match reviewed plan")
    records = json.loads(remote.read("shards.json"))
    by_name = {r["name"]: r for r in records}
    if len(by_name) != len(records) or not set(by_name) <= expected:
        raise ValueError("Remote shard index is incomplete")
    if not selected <= set(by_name):
        raise PublicationPending("Remote shard index has not committed requested receipts yet")
    paths = []
    for name in sorted(selected):
        paths.extend(("data/" + name, "indices/" + name + ".files.jsonl.gz"))
    metadata = remote.metadata(paths)
    indices = output / "cleanup_indices"
    indices.mkdir(exist_ok=True)
    for shard in plan["shards"]:
        name = shard["name"]
        if name not in selected:
            continue
        receipt, published = state["verified"][name], by_name[name]
        for key in ("sha256", "size", "index_sha256", "index_size", "file_count"):
            if receipt[key] != published[key]:
                raise ValueError("Published receipt differs from local receipt: " + name)
        for path, size, digest in (
                ("data/" + name, receipt["size"], receipt["sha256"]),
                ("indices/" + name + ".files.jsonl.gz", receipt["index_size"], receipt["index_sha256"])):
            info = metadata.get(path)
            if info is None or info["size"] != size:
                raise ValueError("Remote artifact missing or size mismatch: " + path)
            if info["sha256"] is not None and info["sha256"] != digest:
                raise ValueError("Remote artifact hash mismatch: " + path)
            if path.startswith("data/") and info["sha256"] is None:
                raise ValueError("Missing content-addressed hash for remote data shard")
        data = remote.read("indices/" + name + ".files.jsonl.gz")
        if hashlib.sha256(data).hexdigest() != receipt["index_sha256"] or len(data) != receipt["index_size"]:
            raise ValueError("Downloaded index checksum mismatch: " + name)
        rows = (json.loads(line) for line in gzip.decompress(data).decode().splitlines())
        for entry, row in itertools.zip_longest(shard["entries"], rows):
            if (entry is None or row is None or entry["path"] != row["path"]
                    or entry["size"] != row["size"] or len(row["sha256"]) != 64):
                raise ValueError("Per-file index differs from reviewed plan: " + name)
        (indices / (name + ".files.jsonl.gz")).write_bytes(data)
    return {"repo_id": state["repo_id"], "revision": remote.revision,
            "shards_verified": len(selected),
            "file_count": sum(len(s["entries"]) for s in plan["shards"] if s["name"] in selected)}


def parent_fd(root_fd, relative):
    path = Path(relative)
    if path.is_absolute() or not path.parts or any(p in (".", "..") for p in path.parts):
        raise ValueError("Unsafe archive member path")
    descriptor = os.dup(root_fd)
    try:
        for name in path.parts[:-1]:
            next_fd = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = next_fd
        return descriptor, path.name
    except BaseException:
        os.close(descriptor)
        raise


def fingerprint(info):
    return [info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns]


def delete_verified_members(plan, output, progress=lambda **kwargs: None):
    root = Path(plan["source_root"])
    root_fd = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    report = {"deleted_files": 0, "deleted_bytes": 0, "preserved_changed_files": 0,
              "already_missing_files": 0, "removed_empty_directories": 0,
              "disk_free_bytes_before": shutil.disk_usage(root).free}
    empty_candidates = set()
    last_update = time.monotonic()
    try:
        # All remote artifacts for this selection have already been verified.
        # Hash all members of the selected shard before deleting its first member.
        with tempfile.TemporaryFile(mode="w+t", dir=output) as candidates, \
                (output / "cleanup_preserved.local.jsonl").open("a") as preserved:
            inspected = 0
            for shard in plan["shards"]:
                index = output / "cleanup_indices" / (shard["name"] + ".files.jsonl.gz")
                with gzip.open(index, "rt") as stream:
                    for line in stream:
                        row = json.loads(line)
                        directory = None
                        try:
                            directory, name = parent_fd(root_fd, row["path"])
                            fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory)
                            with os.fdopen(fd, "rb") as source:
                                before = os.fstat(source.fileno())
                                if not stat.S_ISREG(before.st_mode):
                                    raise ValueError("Not a regular file")
                                digest = hashlib.sha256()
                                for block in iter(lambda: source.read(1024 * 1024), b""):
                                    digest.update(block)
                                after = os.fstat(source.fileno())
                            current = os.stat(name, dir_fd=directory, follow_symlinks=False)
                            if (fingerprint(before) != fingerprint(after) or fingerprint(after) != fingerprint(current)
                                    or after.st_size != row["size"] or digest.hexdigest() != row["sha256"]):
                                raise ValueError("Local data changed after backup")
                            candidates.write(json.dumps({"path": row["path"], "fingerprint": fingerprint(after)}) + "\n")
                        except FileNotFoundError:
                            report["already_missing_files"] += 1
                        except (OSError, ValueError) as error:
                            report["preserved_changed_files"] += 1
                            preserved.write(json.dumps({"path": row["path"], "reason": str(error)}) + "\n")
                        finally:
                            if directory is not None:
                                os.close(directory)
                        inspected += 1
                        if time.monotonic() - last_update > 20:
                            progress(phase="HASHING_LOCAL", inspected_files=inspected, **report)
                            last_update = time.monotonic()
            if inspected != plan["file_count"]:
                raise ValueError("Local candidate count mismatch; nothing deleted")
            candidates.flush()
            candidates.seek(0)
            progress(phase="DELETING_VERIFIED_MEMBERS", inspected_files=inspected, **report)
            for line in candidates:
                candidate = json.loads(line)
                directory = None
                try:
                    directory, name = parent_fd(root_fd, candidate["path"])
                    current = os.stat(name, dir_fd=directory, follow_symlinks=False)
                    if fingerprint(current) != candidate["fingerprint"]:
                        raise ValueError("File changed between verification and removal")
                    os.unlink(name, dir_fd=directory)
                    report["deleted_files"] += 1
                    report["deleted_bytes"] += current.st_size
                    parts = Path(candidate["path"]).parts
                    empty_candidates.update("/".join(parts[:i]) for i in range(1, len(parts)))
                except FileNotFoundError:
                    report["already_missing_files"] += 1
                except (OSError, ValueError) as error:
                    report["preserved_changed_files"] += 1
                    preserved.write(json.dumps({"path": candidate["path"], "reason": str(error)}) + "\n")
                finally:
                    if directory is not None:
                        os.close(directory)
                if time.monotonic() - last_update > 20:
                    progress(phase="DELETING_VERIFIED_MEMBERS", **report)
                    last_update = time.monotonic()
        for relative in sorted(empty_candidates, key=lambda p: len(Path(p).parts), reverse=True):
            directory = None
            try:
                directory, name = parent_fd(root_fd, relative)
                os.rmdir(name, dir_fd=directory)
                report["removed_empty_directories"] += 1
            except OSError:
                pass  # Nonempty, changed or already missing directories stay untouched.
            finally:
                if directory is not None:
                    os.close(directory)
    finally:
        os.close(root_fd)
    report["disk_free_bytes_after"] = shutil.disk_usage(root).free
    return report


def incremental_cleanup(plan, output, update, plan_digest):
    journal_path = output / "per_shard_cleanup_state.local.json"
    journal = json.loads(journal_path.read_text()) if journal_path.exists() else {
        "plan_sha256": plan_digest, "cleaned": {}}
    if journal["plan_sha256"] != plan_digest:
        raise ValueError("Cleanup journal belongs to another reviewed plan")
    shards = {s["name"]: s for s in plan["shards"]}

    def totals():
        keys = ("deleted_files", "deleted_bytes", "preserved_changed_files", "already_missing_files")
        records = list(journal["cleaned"].values()) + list(journal.get("retry_previous", {}).values())
        return {key: sum(r.get(key, 0) for r in records) for key in keys}

    while len(journal["cleaned"]) < len(shards):
        if sha256(output / "plan.json") != plan_digest:
            raise ValueError("Reviewed upload plan changed")
        state_path = output / "upload_state.local.json"
        state = json.loads(state_path.read_text()) if state_path.exists() else {"verified": {}}
        pending = sorted(set(state["verified"]) - set(journal["cleaned"]))
        if not pending:
            update(state="WAITING_FOR_NEXT_SHARD", error=None, cleaned_shards=len(journal["cleaned"]), **totals())
            time.sleep(10)
            continue
        for name in pending:
            update(state="VERIFYING_SHARD", current_shard=name, cleaned_shards=len(journal["cleaned"]), **totals())
            for attempt in range(1, 13):
                try:
                    remote = PublicArchive(state["repo_id"])
                    verified = verify_publication(plan, state, remote, output, {name})
                    break
                except PublicationPending:
                    if attempt == 12:
                        raise
                    time.sleep(5)
                except ValueError:
                    raise
                except Exception as error:
                    if attempt == 12:
                        raise
                    update(state="RETRYING_SHARD_VERIFICATION", error=str(error), current_shard=name)
                    time.sleep(10)
            subset = {"source_root": plan["source_root"], "file_count": len(shards[name]["entries"]),
                      "shards": [shards[name]]}
            base = totals()

            def progress(**values):
                for key in base:
                    if key in values:
                        values[key] += base[key]
                update(state="CLEANING_SHARD", current_shard=name, cleaned_shards=len(journal["cleaned"]), **values)

            result = delete_verified_members(subset, output, progress)
            previous = journal.get("retry_previous", {}).pop(name, None)
            if previous:
                result["already_missing_files"] = max(previous.get("already_missing_files", 0),
                    result["already_missing_files"] - previous.get("deleted_files", 0))
                result["deleted_files"] += previous.get("deleted_files", 0)
                result["deleted_bytes"] += previous.get("deleted_bytes", 0)
            result["remote_revision"] = verified["revision"]
            journal["cleaned"][name] = result
            write_json(journal_path, journal)
            update(state="SHARD_CLEANED", error=None, current_shard=name, cleaned_shards=len(journal["cleaned"]), **totals())
            print(json.dumps({"shard_cleaned": name, "deleted_bytes": result["deleted_bytes"],
                              "cleaned_shards": len(journal["cleaned"])}), flush=True)
    update(state="COMPLETE", cleaned_shards=len(journal["cleaned"]), **totals())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/home/tianbot/RUMA-hf-public-20261019"))
    parser.add_argument("--authorize-cleanup", action="store_true", required=True)
    parser.add_argument("--per-shard", action="store_true",
                        help="remove each verified shard immediately while the rest uploads")
    args = parser.parse_args()
    output = args.output.resolve()
    with (output / "cleanup_service.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        plan_path = output / "plan.json"
        plan_digest = sha256(plan_path)
        plan = json.loads(plan_path.read_text())
        source = Path(plan["source_root"])
        if (source != Path("/home/tianbot/RUMA-legacy-archive-20261019") or source.is_symlink()
                or source.resolve() == output or source in output.parents):
            raise ValueError("Refusing cleanup outside the explicitly authorized historical archive")
        status_path = output / "local_cleanup_status.local.json"
        status = {"pid": os.getpid(), "source_root": str(source), "plan_sha256": plan_digest,
                  "authorized": True, "per_shard": args.per_shard,
                  "state": "WAITING_FOR_UPLOAD", "updated_at_unix": time.time()}

        def update(**values):
            status.update(values, updated_at_unix=time.time())
            write_json(status_path, status)

        update()
        if args.per_shard:
            try:
                incremental_cleanup(plan, output, update, plan_digest)
            except Exception as error:
                update(state="FAILED", error=str(error))
                raise
            return
        while True:
            if sha256(plan_path) != plan_digest:
                update(state="FAILED", error="Reviewed upload plan changed; no cleanup attempted")
                return
            state_path = output / "upload_state.local.json"
            completion = json.loads((output / "public/publication_status.json").read_text())
            state = json.loads(state_path.read_text()) if state_path.exists() else None
            if completion.get("state") == "COMPLETE" and state and len(state["verified"]) == len(plan["shards"]):
                break
            time.sleep(30)
        try:
            update(state="VERIFYING_REMOTE")
            for attempt in range(1, 13):
                try:
                    remote = PublicArchive(state["repo_id"])
                    verified = verify_publication(plan, state, remote, output)
                    break
                except PublicationPending:
                    if attempt == 12:
                        raise
                    update(state="WAITING_FOR_REMOTE_COMPLETION", verification_attempt=attempt)
                    time.sleep(30)
                except ValueError:
                    raise  # An integrity mismatch must never authorize deletion.
                except Exception as error:
                    if attempt == 12:
                        raise
                    update(state="RETRYING_REMOTE_VERIFICATION", error=str(error), verification_attempt=attempt)
                    time.sleep(30)
            write_json(output / "cleanup_remote_verification.local.json", verified)
            result = delete_verified_members(plan, output, lambda **values: update(state="VERIFYING_AND_CLEANING", **values))
            update(state="COMPLETE", **result, remote_revision=verified["revision"])
            print(json.dumps(status), flush=True)
        except Exception as error:
            update(state="FAILED", error=str(error))
            raise


if __name__ == "__main__":
    main()
