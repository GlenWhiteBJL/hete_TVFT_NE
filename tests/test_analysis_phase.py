"""Regression checks for the future-t0 RUNNING commit in recorded ROS data."""
import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_experiment.py"
SPEC = importlib.util.spec_from_file_location("analysis_phase", SCRIPT)
ANALYSIS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYSIS)


class FormalPhaseTests(unittest.TestCase):
    def test_future_commit_waiting_prefix_is_excluded(self):
        # Actual ROS nodes publish RUNNING and tau=0 during the two-second
        # commit barrier. Only the samples at/after physical t0 are formal.
        records = [{"phase": "RUNNING", "stamp": stamp, "tau": 0.0}
                   for stamp in (98.0, 99.98, 100.0, 100.02)]
        elapsed = [ANALYSIS.formal_elapsed(record, 100.0) for record in records]
        self.assertEqual(elapsed[:3], [None, None, 0.0])
        self.assertAlmostEqual(elapsed[3], .02)

    def test_elapsed_time_comes_from_stamp_instead_of_reported_tau(self):
        for tau in (0.0, 999.0, -3.0, None, float("nan")):
            with self.subTest(tau=tau):
                record = {"phase": "RUNNING", "stamp": 101.25, "tau": tau}
                self.assertEqual(ANALYSIS.formal_elapsed(record, 100.0), 1.25)
        self.assertEqual(ANALYSIS.formal_elapsed({"phase": "RUNNING", "stamp": 100.5}, 100.0), .5)
        self.assertIsNone(ANALYSIS.formal_elapsed({"phase": "RUNNING", "stamp": 99.0, "tau": 999.0}, 100.0))

    def test_other_phases_cannot_enter_formal_statistics(self):
        for phase in ("IDLE", "SCHEDULED", "STOPPING", "STOPPED", "LANDED", "FAULT", None):
            with self.subTest(phase=phase):
                self.assertIsNone(ANALYSIS.formal_elapsed({"phase": phase, "stamp": 101.0, "tau": 1.0}, 100.0))

    def test_invalid_or_nonfinite_clock_values_are_excluded(self):
        for stamp in (None, "invalid", float("nan"), float("inf"), -float("inf")):
            with self.subTest(stamp=stamp):
                self.assertIsNone(ANALYSIS.formal_elapsed({"phase": "RUNNING", "stamp": stamp, "tau": 0.0}, 100.0))
        self.assertIsNone(ANALYSIS.formal_elapsed({"phase": "RUNNING", "tau": 0.0}, 100.0))
        for t0 in (None, "invalid", float("nan"), float("inf"), -float("inf")):
            with self.subTest(t0=t0):
                self.assertIsNone(ANALYSIS.formal_elapsed({"phase": "RUNNING", "stamp": 100.0}, t0))


if __name__ == "__main__":
    unittest.main()
