from contextlib import redirect_stdout
import gzip
import io
import json
import os
from pathlib import Path
import tempfile
import unittest

from evaluation.dataset_capture.publish_archive import prepare, pack
from evaluation.dataset_capture.cleanup_archive import (
    PublicationPending, delete_verified_members, verify_publication,
)


class FakeRemote:
    revision = "frozen-test-revision"

    def __init__(self, plan, receipt, index):
        name = receipt["name"]
        self.data = {
            "publication_status.json": json.dumps({"state": "COMPLETE", "shards_expected": 1,
                                                   "shards_verified": 1}).encode(),
            "dataset_summary.json": json.dumps({k: plan[k] for k in ("file_count", "source_bytes")}).encode(),
            "shards.json": json.dumps([receipt]).encode(),
            "indices/" + index.name: index.read_bytes(),
        }
        self.info = {
            "data/" + name: {"size": receipt["size"], "sha256": receipt["sha256"]},
            "indices/" + index.name: {"size": receipt["index_size"], "sha256": receipt["index_sha256"]},
        }

    def read(self, path):
        return self.data[path]

    def metadata(self, paths):
        return {p: self.info[p] for p in paths if p in self.info}


class ArchiveCleanupTests(unittest.TestCase):
    def fixture(self, base):
        source = base / "archive"
        run = source / "experiments/run1"
        run.mkdir(parents=True)
        (run / "a.jpg").write_bytes(b"original camera frame")
        (run / "b.npz").write_bytes(b"original point cloud")
        (run / "unpublished.bin").write_bytes(b"keep this excluded file")
        output = base / "upload"
        with redirect_stdout(io.StringIO()):
            plan = prepare(source, output, 1024 ** 2)
        _, index, receipt = pack(plan, output, plan["shards"][0])
        state = {"repo_id": "test/archive", "verified": {receipt["name"]: receipt}}
        remote = FakeRemote(plan, receipt, index)
        return source, run, output, plan, state, remote

    def test_incomplete_publication_never_authorizes_cleanup(self):
        with tempfile.TemporaryDirectory() as temp:
            _, run, output, plan, state, remote = self.fixture(Path(temp))
            remote.data["publication_status.json"] = b'{"state":"UPLOADING"}'
            with self.assertRaises(PublicationPending):
                verify_publication(plan, state, remote, output)
            self.assertTrue((run / "a.jpg").exists())

    def test_missing_remote_artifact_prevents_any_deletion(self):
        with tempfile.TemporaryDirectory() as temp:
            _, run, output, plan, state, remote = self.fixture(Path(temp))
            del remote.info["data/data-00000.tar"]
            with self.assertRaises(ValueError):
                verify_publication(plan, state, remote, output)
            self.assertTrue((run / "a.jpg").exists())

    def test_remote_index_corruption_prevents_cleanup(self):
        with tempfile.TemporaryDirectory() as temp:
            _, run, output, plan, state, remote = self.fixture(Path(temp))
            remote.data["indices/data-00000.tar.files.jsonl.gz"] = b"corrupt"
            with self.assertRaises(ValueError):
                verify_publication(plan, state, remote, output)
            self.assertTrue((run / "b.npz").exists())

    def test_only_backed_up_unchanged_files_are_removed(self):
        with tempfile.TemporaryDirectory() as temp:
            _, run, output, plan, state, remote = self.fixture(Path(temp))
            verify_publication(plan, state, remote, output)
            # Same byte count, changed contents: stat size alone must not authorize deletion.
            original = run / "b.npz"
            original.write_bytes(b"x" * original.stat().st_size)
            result = delete_verified_members(plan, output)
            self.assertEqual(result["deleted_files"], 1)
            self.assertEqual(result["preserved_changed_files"], 1)
            self.assertFalse((run / "a.jpg").exists())
            self.assertTrue(original.exists())
            self.assertTrue((run / "unpublished.bin").exists())

    def test_new_files_and_symlinks_are_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            _, run, output, plan, state, remote = self.fixture(base)
            verify_publication(plan, state, remote, output)
            outside = base / "outside.jpg"
            outside.write_bytes(b"original camera frame")
            (run / "a.jpg").unlink()
            (run / "a.jpg").symlink_to(outside)
            (run / "new.jpg").write_bytes(b"new observations after backup")
            result = delete_verified_members(plan, output)
            self.assertEqual(result["deleted_files"], 1)
            self.assertTrue(outside.exists())
            self.assertTrue((run / "a.jpg").is_symlink())
            self.assertTrue((run / "new.jpg").exists())

    def test_changes_between_hashing_and_removal_are_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            _, run, output, plan, state, remote = self.fixture(Path(temp))
            verify_publication(plan, state, remote, output)

            def mutate(**progress):
                if progress.get("phase") == "DELETING_VERIFIED_MEMBERS":
                    (run / "a.jpg").write_bytes(b"changed after hashing")

            result = delete_verified_members(plan, output, mutate)
            self.assertEqual(result["deleted_files"], 1)
            self.assertTrue((run / "a.jpg").exists())

    def test_non_regular_replacement_is_preserved_without_blocking(self):
        with tempfile.TemporaryDirectory() as temp:
            _, run, output, plan, state, remote = self.fixture(Path(temp))
            verify_publication(plan, state, remote, output)
            (run / "a.jpg").unlink()
            os.mkfifo(run / "a.jpg")
            result = delete_verified_members(plan, output)
            self.assertEqual(result["deleted_files"], 1)
            self.assertEqual(result["preserved_changed_files"], 1)
            self.assertTrue((run / "a.jpg").exists())


if __name__ == "__main__":
    unittest.main()
