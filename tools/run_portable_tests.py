"""Run the unit suite with explicit Windows exclusions; Linux runs discovery unchanged."""
import importlib
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    loader = unittest.TestLoader()
    if sys.platform != 'win32':
        suite = loader.discover(str(ROOT / 'tests' / 'unit'), top_level_dir=str(ROOT))
    else:
        print('Windows exclusions: test_control_guard (fcntl), reference worker '
              'process lifetime (Linux prctl), and source archive executable-mode '
              'publication (Windows file permissions).', file=sys.stderr, flush=True)
        excluded = {'test_control_guard', 'test_reference_worker', 'test_source_archive'}
        suite = unittest.TestSuite()
        for path in sorted((ROOT / 'tests' / 'unit').glob('test_*.py')):
            if path.stem not in excluded:
                suite.addTests(loader.loadTestsFromModule(
                    importlib.import_module('tests.unit.' + path.stem)))
        for name in (
            'tests.unit.test_reference_worker.ReferenceIsolationTests.test_recovered_reference_does_not_automatically_resume_after_hold',
            'tests.unit.test_source_archive.SourceArchiveTests.test_changed_source_or_source_set_aborts_before_publication',
            'tests.unit.test_source_archive.SourceArchiveTests.test_symlink_and_escaping_names_are_not_archived',
        ):
            suite.addTests(loader.loadTestsFromName(name))
    return 0 if unittest.TextTestRunner(verbosity=1).run(suite).wasSuccessful() else 1


if __name__ == '__main__':
    sys.exit(main())
