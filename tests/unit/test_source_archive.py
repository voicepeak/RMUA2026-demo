import hashlib
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import unittest

from evaluation.seed_runner.manifest import archive_sources, file_hash, prepare_run, source_hashes


@unittest.skipUnless(shutil.which('git'), 'host provenance tests require git')
class SourceArchiveTests(unittest.TestCase):
    def repository(self, root):
        subprocess.check_call(['git', 'init', '-q', str(root)])
        (root / '.gitignore').write_text('runs/\nignored_secret\n')
        (root / 'control.py').write_text('original\n')
        subprocess.check_call(['git', 'add', '.'], cwd=root)
        subprocess.check_call(['git', '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid',
                               'commit', '-qm', 'initial'], cwd=root)
        (root / 'control.py').write_text('dirty bytes\n')
        (root / 'new.sh').write_text('#!/bin/sh\nexit 0\n')
        (root / 'new.sh').chmod(0o755)
        (root / 'ignored_secret').write_text('excluded\n')

    def test_dirty_and_untracked_bytes_restore_exactly_with_executable_mode(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            self.repository(root)
            run, manifest = prepare_run(root, root / 'runs', 1, 'test-digest', {})
            snapshot = manifest['source_archive']
            with tarfile.open(run / snapshot['path']) as archive:
                self.assertEqual(set(archive.getnames()), set(manifest['source_sha256_before']))
                for member in archive:
                    data = archive.extractfile(member).read()
                    self.assertEqual(data, (root / member.name).read_bytes())
                    self.assertEqual(hashlib.sha256(data).hexdigest(),
                                     manifest['source_sha256_before'][member.name])
                    self.assertEqual(member.uid, 0)
                    self.assertEqual(member.mtime, 0)
                self.assertEqual(archive.getmember('new.sh').mode, 0o755)
            other = root / 'runs/second'
            other.mkdir()
            same = archive_sources(root, other, manifest['source_sha256_before'])
            self.assertEqual(snapshot['sha256'], file_hash(other / same['path']))
            with self.assertRaises(FileExistsError):
                archive_sources(root, other, manifest['source_sha256_before'])
            self.assertEqual(snapshot['sha256'], same['sha256'])
            self.assertFalse(list(other.glob('*.tmp')))

    def test_changed_source_or_source_set_aborts_before_publication(self):
        for change in ('content', 'new_file'):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as directory:
                root = Path(directory).resolve()
                self.repository(root)
                expected = source_hashes(root)
                run = root / 'runs/case'
                run.mkdir(parents=True)
                if change == 'content':
                    (root / 'control.py').write_text('changed again\n')
                else:
                    (root / 'added.py').write_text('added\n')
                with self.assertRaises(RuntimeError):
                    archive_sources(root, run, expected)
                self.assertFalse(list(run.iterdir()))

    def test_symlink_and_escaping_names_are_not_archived(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            self.repository(root)
            run = root / 'runs/case'
            run.mkdir(parents=True)
            (root / 'link').symlink_to(root / 'control.py')
            for name in ('link', '../outside', str(root / 'control.py')):
                with self.subTest(name=name), self.assertRaises(ValueError):
                    archive_sources(root, run, {name: 'irrelevant'})
                self.assertFalse(list(run.iterdir()))


if __name__ == '__main__':
    unittest.main()
