"""Detached, resumable archive upload. Token arrives through stdin and stays in memory."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

from evaluation.dataset_capture.publish_archive import publish, write_json


class RedactedLog:
    def __init__(self, handle, credential):
        self.handle, self.credential = handle, credential

    def write(self, value):
        return self.handle.write(value.replace(self.credential, "[REDACTED]"))

    def flush(self):
        self.handle.flush()

    def isatty(self):
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/home/tianbot/RUMA-hf-public-20261019"))
    parser.add_argument("--repo-id", default="Vociepeak/rmua2026-legacy-data")
    args = parser.parse_args()
    credential = sys.stdin.readline().strip()
    if not credential:
        raise RuntimeError("Supply the credential through interactive stdin, never command arguments")
    # Keep the existing HTTP proxy; the SDK rejects this host's obsolete socks:// alias.
    for name in ("ALL_PROXY", "all_proxy"):
        if os.environ.get(name, "").startswith("socks://"):
            os.environ.pop(name)
    os.environ["HF_TOKEN"] = credential
    os.environ["HF_HUB_DISABLE_PROGRESS_BARS"] = "1"
    os.environ["HF_XET_HIGH_PERFORMANCE"] = "1"
    from huggingface_hub import HfApi
    owner = HfApi().whoami()["name"]
    if owner.lower() != args.repo_id.split("/", 1)[0].lower():
        raise RuntimeError("Authenticated account does not match repository owner")
    plan = json.loads((args.output / "plan.json").read_text())
    import fcntl
    lock = (args.output / "upload_service.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    read_fd, write_fd = os.pipe()
    child = os.fork()
    if child:
        os.close(write_fd)
        with os.fdopen(read_fd) as pipe:
            print(pipe.read().strip(), flush=True)
        return
    os.close(read_fd)
    os.setsid()
    log_path = args.output / "upload_service.log"
    descriptor = os.open(log_path, os.O_CREAT | os.O_WRONLY | os.O_APPEND, 0o600)
    for target in (1, 2):
        os.dup2(descriptor, target)
    if descriptor > 2:
        os.close(descriptor)
    null = os.open(os.devnull, os.O_RDONLY)
    os.dup2(null, 0)
    if null > 2:
        os.close(null)
    sys.stdout = RedactedLog(os.fdopen(os.dup(1), "w", buffering=1), credential)
    sys.stderr = RedactedLog(os.fdopen(os.dup(2), "w", buffering=1), credential)
    state_path = args.output / "upload_service_status.local.json"
    state = {"pid": os.getpid(), "repo_id": args.repo_id, "started_at_unix": time.time(),
             "shards_expected": len(plan["shards"]), "log": str(log_path)}

    def status(phase, **kwargs):
        state.update(state=phase, updated_at_unix=time.time(), **kwargs)
        write_json(state_path, state)

    def stop(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    status("UPLOADING")
    os.write(write_fd, (json.dumps({"service_started": True, "pid": os.getpid(),
                                  "repo_id": args.repo_id}) + "\n").encode())
    os.close(write_fd)
    try:
        for attempt in range(1, 13):
            status("UPLOADING", attempt=attempt)
            try:
                publish(args.output, plan, args.repo_id)
                status("COMPLETE")
                break
            except Exception as error:
                message = str(error).replace(credential, "[REDACTED]")
                code = getattr(getattr(error, "response", None), "status_code", None)
                print(json.dumps({"attempt_failed": attempt, "error": message}), flush=True)
                if code in (401, 403, 404) or attempt == 12:
                    status("FAILED", error=message)
                    break
                delay = min(60, 5 * 2 ** min(attempt, 4))
                status("RETRYING", error=message, retry_delay_s=delay)
                time.sleep(delay)
    except KeyboardInterrupt:
        status("STOPPED")
    finally:
        os.environ.pop("HF_TOKEN", None)
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
