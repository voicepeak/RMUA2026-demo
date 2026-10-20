"""Exclusive run directories and complete source/config hashes; no flight execution."""
from datetime import datetime, timezone
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import stat
import subprocess
import tarfile
import tempfile
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


def archive_sources(root, run, expected):
    """Retain the exact bytes, including dirty files, before starting a scene.

    Only regular, repository-relative sources are accepted. Atomic exclusive
    publication prevents either a partial archive or replacement of old evidence.
    Metadata is normalized so the same bytes and modes produce the same archive.
    """
    root, run = Path(root).resolve(), Path(run)
    destination = run / "source_snapshot.tar.gz"
    temporary = None
    total = 0
    try:
        with tempfile.NamedTemporaryFile(dir=run, suffix=".tmp", delete=False) as output:
            temporary = Path(output.name)
            with gzip.GzipFile(filename="", mode="wb", fileobj=output, mtime=0) as compressed:
                with tarfile.open(fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT) as archive:
                    for name, expected_hash in sorted(expected.items()):
                        relative = Path(name)
                        path = root / relative
                        if (relative.is_absolute() or ".." in relative.parts
                                or relative.as_posix() != name or path.resolve() != path
                                or path.is_symlink()):
                            raise ValueError("source must be a regular repository-relative file: " + name)
                        with path.open("rb") as source:
                            metadata = os.fstat(source.fileno())
                            if not stat.S_ISREG(metadata.st_mode):
                                raise ValueError("nonregular source: " + name)
                            data = source.read()
                        if hashlib.sha256(data).hexdigest() != expected_hash:
                            raise RuntimeError("source changed before archival: " + name)
                        member = tarfile.TarInfo(name)
                        member.size = len(data)
                        member.mode = stat.S_IMODE(metadata.st_mode)
                        archive.addfile(member, io.BytesIO(data))
                        total += len(data)
            output.flush()
            os.fsync(output.fileno())
        if source_hashes(root) != expected:
            raise RuntimeError("source set changed during archival")
        temporary.chmod(0o444)
        os.link(str(temporary), str(destination))
        return {"path": destination.name, "sha256": file_hash(destination),
                "files": len(expected), "uncompressed_bytes": total,
                "scope": "workspace_sources_not_container_or_simulator_binaries"}
    finally:
        if temporary is not None:
            temporary.unlink()


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
    manifest["source_archive"] = archive_sources(root, run, manifest["source_sha256_before"])
    (run / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return run, manifest
