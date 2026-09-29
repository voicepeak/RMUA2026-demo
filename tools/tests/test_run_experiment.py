import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_experiment import execute, prepare


class RunTests(unittest.TestCase):
    def test_preparation_snapshots_inputs_and_preserves_argv(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = Path(__file__).resolve().parents[2] / "ros_ws/src"
            model = root / "model with spaces.pt"
            model.write_bytes(b"synthetic model")
            args = argparse.Namespace(scene="test", seed=123, runtime_src=source,
                                      runs_dir=root / "runs", model=model, simulator_config=None,
                                      evaluation_gates=None, timeout=3,
                                      launch_arg=["cruise_speed:=5.0", "trace_file:=/tmp/a $(literal).csv"])
            with patch("run_experiment.command_output", return_value="synthetic-git-metadata"), \
                    patch("run_experiment.subprocess.Popen") as popen:
                run, manifest = prepare(args)
                popen.assert_not_called()
            self.assertEqual(manifest["state"], "PREPARED")
            self.assertFalse(any(manifest["runtime_source_diff"].values()))
            self.assertIn("cruise_speed:=5.0", manifest["launch_argv"])
            self.assertNotIn("cruise_speed:=3.0", manifest["launch_argv"])
            self.assertIn("trace_file:=/tmp/a $(literal).csv", manifest["launch_argv"])
            self.assertEqual(manifest["input_sha256"]["model"], hashlib.sha256(model.read_bytes()).hexdigest())
            self.assertEqual((run / "inputs/model.pt").read_bytes(), model.read_bytes())

    def test_source_drift_aborts_before_launch(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            manifest = {"runtime_source_diff": {"changed": ["controller.py"]}, "termination_reason": None}
            with patch("run_experiment.subprocess.Popen") as popen:
                result = execute(argparse.Namespace(), run, manifest)
                popen.assert_not_called()
            self.assertEqual(result, 1)
            self.assertEqual(json.loads((run / "manifest.json").read_text())["state"], "ERROR")


if __name__ == "__main__":
    unittest.main()
