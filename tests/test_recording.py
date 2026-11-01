"""Offline storage and measurement-alignment tests (no ROS required)."""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock
import numpy as np
from ne_tvft.recording import SessionWriter, decode_packet, validate_session_id


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_experiment.py"
SPEC = importlib.util.spec_from_file_location("experiment_analysis", SCRIPT)
ANALYSIS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYSIS)
RECORDER_SPEC = importlib.util.spec_from_file_location("experiment_recorder", SCRIPT.with_name("recorder_node.py"))
RECORDER = importlib.util.module_from_spec(RECORDER_SPEC)
RECORDER_SPEC.loader.exec_module(RECORDER)


def temporary_directory():
    # Keep test artifacts inside the permitted workspace on Windows as well.
    parent = Path(__file__).resolve().parent
    temporary = tempfile.TemporaryDirectory(dir=str(parent))
    if Path(temporary.name).resolve().parent != parent:
        raise RuntimeError("Unexpected temporary test directory")
    return temporary


class RecordingTests(unittest.TestCase):
    def test_roundtrip_preserves_missing_and_controller(self):
        with temporary_directory() as directory:
            writer = SessionWriter(directory, "session_001", {"config": {"groups": []}})
            record = {"id": "uav1", "stamp": 1.0, "session_id": "session_001", "phase": "RUNNING",
                      "position": [1, 2, 3], "acceleration_estimate": None, "acceleration_valid": False,
                      "controller": {"beta": 2.5, "beta_hat": 2.0}}
            self.assertTrue(writer.write(record))
            record["controller"]["beta"] = 99
            self.assertTrue(writer.event({"type": "stop", "stamp": 2}))
            self.assertTrue(writer.close())
            saved = json.loads((writer.directory / "telemetry.jsonl").read_text())
            self.assertIsNone(saved["acceleration_estimate"])
            self.assertFalse(saved["acceleration_valid"])
            self.assertEqual(saved["controller"]["beta"], 2.5)
            with (writer.directory / "telemetry.csv").open(newline="") as handle:
                row = next(csv.DictReader(handle))
            self.assertEqual(json.loads(row["controller"])["beta"], 2.5)
            self.assertEqual(writer.written, 1)
            self.assertEqual(writer.events_written, 1)

    def test_session_path_and_exclusive_creation(self):
        for name in ["../escape", "a/b", "a\\b", "", ".", "/tmp", "a b"]:
            with self.assertRaises(ValueError):
                validate_session_id(name)
        with temporary_directory() as directory:
            writer = SessionWriter(directory, "run", {})
            writer.close()
            with self.assertRaises(FileExistsError):
                SessionWriter(directory, "run", {})

    def test_invalid_json_and_nonfinite(self):
        for value in ["{bad", "[]", '{"version":2}', '{"version":1,"x":NaN}']:
            self.assertIsNone(decode_packet(value))
        self.assertEqual(decode_packet('{"version":1,"x":null}')["x"], None)
        with temporary_directory() as directory:
            writer = SessionWriter(directory, "run", {})
            self.assertFalse(writer.write({"x": float("nan")}))
            self.assertTrue(writer.failed)
            self.assertEqual(writer.dropped, 1)
            self.assertFalse(writer.close())

    def test_disk_write_failure_is_not_success(self):
        class FailingWriter(SessionWriter):
            def _write_item(self, kind, encoded):
                raise OSError("disk full")
        with temporary_directory() as directory:
            writer = FailingWriter(directory, "run", {})
            writer.write({"id": "ugv1"})
            self.assertFalse(writer.close())
            self.assertTrue(writer.failed)
            self.assertIn("disk full", writer.error)
            self.assertFalse(json.loads((writer.directory / "recording_status.json").read_text())["complete"])

    def test_queue_overflow_is_sticky(self):
        gate, entered = threading.Event(), threading.Event()
        class SlowWriter(SessionWriter):
            def _write_item(self, kind, encoded):
                entered.set()
                gate.wait(3)
                super()._write_item(kind, encoded)
        with temporary_directory() as directory:
            writer = SlowWriter(directory, "run", {}, queue_size=1)
            writer.write({"id": "a"})
            self.assertTrue(entered.wait(2))
            self.assertTrue(writer.write({"id": "b"}))
            self.assertFalse(writer.write({"id": "c"}))
            self.assertFalse(writer.healthy)
            gate.set()
            self.assertFalse(writer.close())
            self.assertEqual(writer.written, 2)

    def test_nearest_alignment_never_fills_stale_measurements(self):
        indices = ANALYSIS.nearest_indices([0, .04, .3, 2.0], [0, .05, .5], max_gap=.06)
        np.testing.assert_array_equal(indices, [0, 1, -1, -1])
        with self.assertRaises(ValueError):
            ANALYSIS.nearest_indices([0], [.2, .1])

    def test_missing_stats_do_not_claim_settling(self):
        result = ANALYSIS.statistics([0, 1, 2], [1, np.nan, .01], threshold=.1)
        self.assertEqual(result["missing_samples"], 1)
        self.assertIsNone(result["sampled_settling_time_s"])
        self.assertIsNone(ANALYSIS.clean_json(float("nan")))

    def test_bag_topics_follow_mock_remapping_and_include_events(self):
        node = RECORDER.RecorderNode.__new__(RECORDER.RecorderNode)
        node.mock = True
        node.config = {"robots": {"uav1": {"pose_topic": "/uav1/pose", "command_topic": "/uav1/command",
                                               "pose_type": "PoseStamped"},
                                     "ugv1": {"imu_topic": "ugv1/imu"}}}
        topics = node.bag_topics()
        self.assertIn("/mock/uav1/pose", topics)
        self.assertIn("/mock/ugv1/imu", topics)
        self.assertIn("/ne_tvft/events", topics)
        self.assertIn("/ne_tvft/signal/uav1", topics)
        self.assertNotIn("/uav1/pose", topics)
        self.assertNotIn("PoseStamped", topics)

    def test_required_bag_cannot_be_disabled(self):
        node = RECORDER.RecorderNode.__new__(RECORDER.RecorderNode)
        node.config = {"logging": {"rosbag": False, "bag_required": True}}
        node.bag_ready = False
        with self.assertRaises(RuntimeError):
            node.start_bag()
        self.assertFalse(node.bag_ready)

    def test_bag_ack_waits_for_live_process_and_active_file(self):
        with temporary_directory() as directory:
            node = RECORDER.RecorderNode.__new__(RECORDER.RecorderNode)
            node.mock = True
            node.config = {"logging": {"rosbag": True, "bag_required": True}, "robots": {}}
            node.writer = type("Writer", (), {"directory": Path(directory)})()
            node._shutdown = threading.Event()
            node.bag_ready = False
            fake_process = mock.Mock()
            fake_process.poll.return_value = None
            def start_process(argv, **kwargs):
                self.assertIn("/ne_tvft/events", argv)
                self.assertFalse(kwargs.get("shell", False))
                (Path(directory) / "raw.bag.active").touch()
                return fake_process
            with mock.patch.object(RECORDER.subprocess, "Popen", side_effect=start_process):
                node.start_bag()
            self.assertTrue(node.bag_ready)
            node.bag_log.close()


if __name__ == "__main__":
    unittest.main()
