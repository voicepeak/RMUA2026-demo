import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "ros_ws" / "src" / "route_follower" / "scripts"))
from z_capability import VzCapability, load_capability  # noqa: E402


class VzCapabilityTests(unittest.TestCase):
    def test_up_and_down_are_separate(self):
        cap = VzCapability(up_table=[(0.0, 4.0), (10.0, 2.0)],
                           down_table=[(0.0, 1.0), (10.0, 3.0)])
        self.assertAlmostEqual(cap.up(5.0), 3.0)
        self.assertAlmostEqual(cap.down(5.0), 2.0)

    def test_down_defaults_to_up_when_missing(self):
        cap = VzCapability(up_table=[(0.0, 4.0), (10.0, 2.0)])
        self.assertAlmostEqual(cap.down(5.0), 3.0)

    def test_out_of_range_uses_endpoints(self):
        cap = VzCapability(up_table=[(2.0, 4.0), (8.0, 2.0)])
        self.assertAlmostEqual(cap.up(0.0), 4.0)
        self.assertAlmostEqual(cap.up(20.0), 2.0)

    def test_load_new_format(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "cap.yaml"
            path.write_text(
                "vz_available:\n"
                "  up:\n"
                "    - {vxy: 0.0, vz: 5.0}\n"
                "    - {vxy: 10.0, vz: 2.5}\n"
                "  down:\n"
                "    - {vxy: 0.0, vz: 3.0}\n"
                "    - {vxy: 10.0, vz: 4.0}\n")
            cap = load_capability(str(path))
            self.assertAlmostEqual(cap.up(5.0), 3.75)
            self.assertAlmostEqual(cap.down(5.0), 3.5)

    def test_load_legacy_format(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "cap.yaml"
            path.write_text("vz_available:\n  - {vxy: 0.0, vz_up: 4.0}\n"
                            "  - {vxy: 10.0, vz_up: 2.0}\n")
            cap = load_capability(str(path))
            self.assertAlmostEqual(cap.up(5.0), 3.0)
            self.assertAlmostEqual(cap.down(5.0), 3.0)


if __name__ == "__main__":
    unittest.main()
