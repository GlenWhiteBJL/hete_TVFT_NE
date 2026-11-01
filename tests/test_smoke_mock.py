"""Pure validation tests; these do not claim a real ROS/catkin integration run."""
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("smoke_mock", ROOT / "scripts" / "smoke_mock.py")
smoke = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(smoke)


class MasterGuardTests(unittest.TestCase):
    def test_loopback_only_master(self):
        for uri in ("http://127.0.0.1:11311", "http://localhost:12345/", "http://[::1]:11311"):
            self.assertEqual(smoke.check_master_uri(uri), uri)
        for uri in ("http://robot.local:11311", "http://127.0.0.1.example.org:11311",
                    "http://user@localhost:11311", "https://localhost:11311",
                    "http://localhost:99999", "http://localhost:abc", "http://localhost/path",
                    "http://localhost:11311?target=robot", "http://127.0.0.2:11311"):
            with self.subTest(uri=uri), self.assertRaises(smoke.SmokeFailure):
                smoke.check_master_uri(uri)

    def fixture(self):
        config = yaml.safe_load((ROOT / "config" / "experiment.yaml").read_text(encoding="utf-8"))
        config.update(yaml.safe_load((ROOT / "config" / "topics.yaml").read_text(encoding="utf-8")))
        params = {"/recorder/mock": True}
        pubs = {"/command": ["/ne_tvft_smoke_mock_123"]}
        subs = {"/command": ["/supervisor", "/recorder"]}
        services = {}
        for rid in smoke.agent_ids(config):
            node = "/agent_" + rid
            params.update({node + "/mock": True, node + "/allow_hardware": False, node + "/agent_id": rid})
            robot = config["robots"][rid]
            for key, topic in robot.items():
                if key.endswith("_topic") and topic:
                    topic = smoke.mock_topic(topic)
                    pubs[topic] = [node] if key == "command_topic" else ["/mock_fleet"]
                    subs[topic] = ["/mock_fleet"] if key == "command_topic" else [node]
                if key.endswith("_service"):
                    services[smoke.mock_topic(topic)] = ["/mock_fleet"]
        graph = [list(pubs.items()), list(subs.items()), list(services.items())]
        return config, params, graph

    def test_complete_mock_graph_is_accepted(self):
        self.assertEqual(len(smoke.check_mock_graph(*self.fixture())), 9)

    def test_mock_parameter_is_not_enough_with_a_hardware_actuator_publisher(self):
        config, params, graph = self.fixture()
        graph[0].append(("/uav1/mavros/setpoint_raw/local", ["/agent_uav1"]))
        with self.assertRaises(smoke.SmokeFailure):
            smoke.check_mock_graph(config, params, graph)

    def test_nonmock_flag_or_missing_service_rejected(self):
        config, params, graph = self.fixture()
        params["/agent_uav3/mock"] = False
        with self.assertRaises(smoke.SmokeFailure):
            smoke.check_mock_graph(config, params, graph)
        params["/agent_uav3/mock"] = True
        graph[2] = graph[2][1:]
        with self.assertRaises(smoke.SmokeFailure):
            smoke.check_mock_graph(config, params, graph)

    def test_global_command_cannot_reach_an_unrelated_hardware_controller(self):
        config, params, graph = self.fixture()
        for topic, nodes in graph[1]:
            if topic == "/command":
                nodes.append("/real_flight_supervisor")
        with self.assertRaises(smoke.SmokeFailure):
            smoke.check_mock_graph(config, params, graph)


class RecordingValidationTests(unittest.TestCase):
    def fixture(self, directory):
        ids = ["ugv1", "ugv2", "ugv3"] + ["uav" + str(i) for i in range(1, 7)]
        rows = []
        for stamp in (100.0, 101.0):
            for rid in ids:
                rows.append({"version": 1, "id": rid, "session_id": "test_session", "t0": 100.0,
                             "stamp": stamp, "tau": stamp - 100, "phase": "RUNNING", "healthy": True,
                             "position": [0., 0., 0.], "velocity": [0., 0., 0.],
                             "input_raw": [0., 0., 0.], "input_applied": [0., 0., 0.],
                             "start_offset_s": 0.0})
        (directory / "metadata.json").write_text(json.dumps({"mock": True, "session_id": "test_session", "t0": 100.0}), encoding="utf-8")
        (directory / "recording_status.json").write_text(json.dumps({"complete": True, "failed": False, "dropped": 0, "telemetry_records": len(rows)}), encoding="utf-8")
        for name in ("config_used.yaml", "telemetry.csv", "events.jsonl", "raw.bag"):
            (directory / name).write_text("unit-test-fixture-only\n", encoding="utf-8")
        self.write_rows(directory, rows)
        return ids, rows

    @staticmethod
    def write_rows(directory, rows):
        (directory / "telemetry.jsonl").write_text("".join(json.dumps(p) + "\n" for p in rows), encoding="utf-8")

    @staticmethod
    def fake_bag_factory(rows):
        class FixtureBag:
            def __init__(self, _filename, mode):
                if mode != "r":
                    raise ValueError("read only")

            def __enter__(self):
                return self

            def __exit__(self, *args):
                pass

            def get_message_count(self):
                return len(rows)

            def read_messages(self, topics):
                for row in rows:
                    yield "/ne_tvft/telemetry", SimpleNamespace(data=json.dumps(row)), None
        return FixtureBag

    def validate(self, directory, ids, rows):
        return smoke.validate_recording(directory, "test_session", 100.0, ids, 1.0, 2,
                                        bag_factory=self.fake_bag_factory(rows))

    def test_validation_checks_all_agents_in_json_and_bag(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            ids, rows = self.fixture(directory)
            result = self.validate(directory, ids, rows)
            self.assertEqual(result["bag_formal_agents"], sorted(ids))
            self.assertEqual(result["telemetry_records"], 18)

    def test_other_session_or_prestart_only_rows_cannot_pass(self):
        for other_session in (True, False):
            with self.subTest(other_session=other_session), tempfile.TemporaryDirectory() as temp:
                directory = Path(temp)
                ids, rows = self.fixture(directory)
                rows[0]["session_id" if other_session else "start_offset_s"] = "another_session" if other_session else None
                self.write_rows(directory, rows)
                with self.assertRaises(smoke.SmokeFailure):
                    self.validate(directory, ids, rows)

    def test_missing_bag_agent_or_active_bag_rejected(self):
        for active in (False, True):
            with self.subTest(active=active), tempfile.TemporaryDirectory() as temp:
                directory = Path(temp)
                ids, rows = self.fixture(directory)
                if active:
                    (directory / "raw.bag.active").touch()
                    bag_rows = rows
                else:
                    bag_rows = [p for p in rows if p["id"] != "uav6"]
                with self.assertRaises(smoke.SmokeFailure):
                    self.validate(directory, ids, bag_rows)

    def test_short_or_nonmonotonic_trace_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            ids, rows = self.fixture(directory)
            rows[9]["stamp"] = 100.1
            rows[9]["tau"] = .1
            self.write_rows(directory, rows)
            with self.assertRaises(smoke.SmokeFailure):
                self.validate(directory, ids, rows)

    def test_late_control_start_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            ids, rows = self.fixture(directory)
            rows[0]["start_offset_s"] = .5
            self.write_rows(directory, rows)
            with self.assertRaises(smoke.SmokeFailure):
                self.validate(directory, ids, rows)

    def test_remote_master_fails_before_any_ros_import_and_records_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve()
            with patch.dict("os.environ", {"ROS_MASTER_URI": "http://flight-controller.local:11311"}):
                exit_code = smoke.main(["--report", str(root / "result.json"), "--output-root", str(root / "output")])
            report = json.loads((root / "result.json").read_text(encoding="utf-8"))
            self.assertEqual(exit_code, 1)
            self.assertFalse(report["success"])
            self.assertFalse(report["ros_runtime_executed"])
            self.assertIn("loopback", report["error"])


if __name__ == "__main__":
    unittest.main()
