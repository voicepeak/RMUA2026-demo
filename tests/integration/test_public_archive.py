from contextlib import redirect_stdout
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import tarfile
import tempfile
import unittest

from evaluation.dataset_capture.publish_archive import prepare, pack, scan


class PublicArchiveTests(unittest.TestCase):
    def fixture(self, base):
        source = base / "source"
        run = source / "experiments/run001"
        run.mkdir(parents=True)
        (run / "frame.jpg").write_bytes(b"simulated image")
        (run / "telemetry.json").write_text('{"seed": 456}\n')
        return source, run

    def test_credentials_snapshots_and_symlinks_are_excluded(self):
        with tempfile.TemporaryDirectory() as temp:
            source, run = self.fixture(Path(temp))
            (run / "secret.log").write_text("hf_" + "a" * 30)
            (run / "controller_sources").mkdir()
            (run / "controller_sources/manifest.json").write_text("{}")
            (run / "private_key.txt").write_text("sensitive")
            (run / "outside.jpg").symlink_to("/etc/passwd")
            files, excluded, _ = scan(source)
            self.assertEqual({Path(row["path"]).name for row in files}, {"frame.jpg", "telemetry.json"})
            self.assertIn("possible_credential_content", {r["reason"] for r in excluded})

    def test_tar_and_per_file_index_are_deterministic_and_match_content(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            source, _ = self.fixture(base)
            first, second = base / "a", base / "b"
            with redirect_stdout(io.StringIO()):
                plan = prepare(source, first, 1024 ** 2)
            second.mkdir()
            a, index, receipt = pack(plan, first, plan["shards"][0])
            _, _, other = pack(plan, second, plan["shards"][0])
            self.assertEqual(receipt["sha256"], other["sha256"])
            self.assertEqual(receipt["index_sha256"], other["index_sha256"])
            with gzip.open(index, "rt") as f:
                rows = [json.loads(line) for line in f]
            with tarfile.open(a) as tar:
                self.assertEqual(len(tar.getmembers()), 2)
                for row in rows:
                    member = tar.getmember(row["path"])
                    self.assertFalse(member.issym())
                    self.assertFalse(Path(member.name).is_absolute())
                    self.assertEqual(hashlib.sha256(tar.extractfile(member).read()).hexdigest(), row["sha256"])

    def test_changed_source_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            source, run = self.fixture(base)
            output = base / "out"
            with redirect_stdout(io.StringIO()):
                plan = prepare(source, output, 1024 ** 2)
            (run / "frame.jpg").write_bytes(b"changed")
            with self.assertRaisesRegex(RuntimeError, "Source changed"):
                pack(plan, output, plan["shards"][0])

    def test_existing_plan_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            source, _ = self.fixture(base)
            with redirect_stdout(io.StringIO()):
                prepare(source, base / "out", 1024 ** 2)
                with self.assertRaises(FileExistsError):
                    prepare(source, base / "out", 1024 ** 2)


if __name__ == "__main__":
    unittest.main()
