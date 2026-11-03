#!/usr/bin/env python3
"""Exercise an already launched, isolated ROS mock fleet and inspect its files.

This is an integration test of the ideal ROS emulator, not PX4 SITL. No ROS
imports occur until main(), so the validation helpers can be tested without ROS.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from ne_tvft.protocol import agent_ids, decode
from ne_tvft.recording import validate_session_id


class SmokeFailure(RuntimeError):
    pass


def check_master_uri(uri):
    """Require an explicitly local HTTP master before initializing any ROS node."""
    parsed = urlsplit(uri)
    try:
        valid_port = parsed.port is None or 0 < parsed.port < 65536
    except ValueError:
        valid_port = False
    if (parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost", "::1"}
            or parsed.username or parsed.password or parsed.query or parsed.fragment
            or parsed.path not in {"", "/"} or not valid_port):
        raise SmokeFailure("ROS_MASTER_URI must address a loopback HTTP master")
    return uri


def mock_topic(topic):
    if not isinstance(topic, str) or not topic.startswith("/"):
        raise SmokeFailure("Configured external topics must be absolute ROS names")
    return topic if topic.startswith("/mock/") else "/mock" + topic


def check_mock_graph(config, parameters, graph):
    """Check node parameters AND actual master registrations before commanding."""
    ids = agent_ids(config)
    required = {"ugv1", "ugv2", "ugv3"} | {"uav" + str(i) for i in range(1, 7)}
    if (set(ids) != required or len(ids) != 9 or len(config["groups"]) != 3
            or any(len(g["followers"]) != 2 for g in config["groups"])):
        raise SmokeFailure("Expected the configured three squads of three robots")
    if parameters.get("/recorder/mock") is not True:
        raise SmokeFailure("Recorder must have mock=true")
    publishers, subscribers, services = [dict((topic, set(nodes)) for topic, nodes in rows)
                                          for rows in graph]
    if any(node not in {"/supervisor", "/recorder"} and not node.startswith("/record_")
           for node in subscribers.get("/command", set())):
        raise SmokeFailure("An unexpected node subscribes to the global /command topic")
    if any(not node.startswith("/ne_tvft_smoke_mock_") for node in publishers.get("/command", set())):
        raise SmokeFailure("Another publisher owns the global /command topic")
    for rid in ids:
        node = "/agent_" + rid
        if (parameters.get(node + "/mock") is not True
                or parameters.get(node + "/allow_hardware") is not False
                or parameters.get(node + "/agent_id") != rid):
            raise SmokeFailure("Agent parameters are not the isolated mock launch: " + rid)
        robot = config["robots"][rid]
        command = mock_topic(robot["command_topic"])
        if node not in publishers.get(command, set()) or "/mock_fleet" not in subscribers.get(command, set()):
            raise SmokeFailure("Mock command path is incomplete: " + rid)
        for topic, nodes in publishers.items():
            if node in nodes and topic != command and topic != "/rosout" and not topic.startswith("/ne_tvft/"):
                raise SmokeFailure("An agent publishes outside the mock/internal graph")
        sensor_topics = set()
        for key, topic in robot.items():
            if key.endswith("_topic") and key != "command_topic" and topic:
                expected = mock_topic(topic)
                sensor_topics.add(expected)
                if "/mock_fleet" not in publishers.get(expected, set()) or node not in subscribers.get(expected, set()):
                    raise SmokeFailure("Mock sensor path is incomplete: " + rid)
        for topic, nodes in subscribers.items():
            if node in nodes and topic not in sensor_topics and not topic.startswith("/ne_tvft/"):
                raise SmokeFailure("An agent subscribes outside the mock/internal graph")
        if robot["kind"] == "uav":
            for key in ("arming_service", "mode_service"):
                if "/mock_fleet" not in services.get(mock_topic(robot[key]), set()):
                    raise SmokeFailure("Mock MAVROS service is missing: " + rid)
    return ids


def finite_number(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def formal_packet(packet, sid, t0):
    return (packet.get("session_id") == sid and packet.get("phase") == "RUNNING"
            and finite_number(packet.get("stamp")) and packet["stamp"] >= t0
            and finite_number(packet.get("start_offset_s")))


def validate_recording(directory, sid, t0, ids, duration, rate_hz, bag_factory=None, backend="mock"):
    """Read actual closed recordings; a syntactically present bag is insufficient."""
    directory = Path(directory)
    metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    status = json.loads((directory / "recording_status.json").read_text(encoding="utf-8"))
    if (metadata.get("mock") is not (backend == "mock") or metadata.get("session_id") != sid
            or (backend != "mock" and metadata.get("backend") != backend)):
        raise SmokeFailure("Recording metadata does not identify this backend/session")
    if not finite_number(metadata.get("t0")) or abs(metadata["t0"] - t0) > 1e-6:
        raise SmokeFailure("Recording metadata has a different start time")
    if status.get("complete") is not True or status.get("failed") is not False or status.get("dropped") != 0:
        raise SmokeFailure("Recorder did not flush cleanly without dropped data")
    for filename in ("config_used.yaml", "telemetry.csv", "events.jsonl"):
        if not (directory / filename).is_file() or not (directory / filename).stat().st_size:
            raise SmokeFailure("Missing or empty recording artifact: " + filename)
    samples = {rid: [] for rid in ids}
    start_offsets = {}
    runtime = metadata.get("config", {}).get("runtime", {})
    start_tolerance = float(runtime.get("start_tolerance", runtime.get("max_dt", .06)))
    total = 0
    with (directory / "telemetry.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            packet = decode(line)
            total += 1
            if packet.get("session_id") != sid or packet.get("id") not in samples:
                raise SmokeFailure("Telemetry contains another session or unknown agent")
            if not formal_packet(packet, sid, t0):
                continue
            if (not packet.get("healthy") or packet.get("fault_reason")
                    or not finite_number(packet.get("t0")) or abs(packet["t0"] - t0) > 1e-6):
                raise SmokeFailure("Unhealthy RUNNING telemetry or inconsistent start time")
            offset = packet["start_offset_s"]
            rid = packet["id"]
            if not 0 <= offset <= start_tolerance or abs(start_offsets.setdefault(rid, offset) - offset) > 1e-9:
                raise SmokeFailure("Agent did not retain an on-time, unique control start: " + rid)
            for name in ("position", "velocity", "input_raw", "input_applied"):
                value = packet.get(name)
                if not isinstance(value, list) or len(value) != 3 or not all(finite_number(x) for x in value):
                    raise SmokeFailure("Missing or nonfinite RUNNING state/input")
            if not finite_number(packet.get("tau")) or abs(packet["tau"] - (packet["stamp"] - t0)) > 1e-6:
                raise SmokeFailure("Telemetry elapsed time disagrees with its ROS clock")
            samples[packet["id"]].append(packet["stamp"])
    if total != status.get("telemetry_records"):
        raise SmokeFailure("Written telemetry count disagrees with recording status")
    minimum = max(2, int(duration * rate_hz * .5))
    spans = {}
    for rid, stamps in samples.items():
        if len(stamps) < minimum or any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise SmokeFailure("Missing, insufficient, or unordered formal samples: " + rid)
        spans[rid] = stamps[-1] - stamps[0]
        if spans[rid] < max(0.0, duration - .5):
            raise SmokeFailure("Formal recording is shorter than requested: " + rid)
    bag_path = directory / "raw.bag"
    if not bag_path.is_file() or not bag_path.stat().st_size or list(directory.glob("raw.bag*.active")):
        raise SmokeFailure("No finalized rosbag exists")
    if bag_factory is None:
        import rosbag
        bag_factory = rosbag.Bag
    bag_samples = {rid: [] for rid in ids}
    with bag_factory(str(bag_path), "r") as bag:
        bag_count = bag.get_message_count()
        if bag_count <= 0:
            raise SmokeFailure("The rosbag contains no messages")
        for _topic, msg, _stamp in bag.read_messages(topics=["/ne_tvft/telemetry"]):
            packet = decode(msg.data)
            if formal_packet(packet, sid, t0):
                if packet.get("id") not in bag_samples or not finite_number(packet.get("t0")) or abs(packet["t0"] - t0) > 1e-6:
                    raise SmokeFailure("The rosbag contains an unexpected agent/start time")
                bag_samples[packet["id"]].append(packet["stamp"])
        for rid, stamps in bag_samples.items():
            if len(stamps) < minimum or max(stamps) - min(stamps) < max(0.0, duration - .5):
                raise SmokeFailure("The rosbag lacks full-duration formal telemetry: " + rid)
    return {"telemetry_records": total, "formal_counts": {rid: len(v) for rid, v in samples.items()},
            "formal_span_s": spans, "bag_message_count": bag_count,
            "start_offset_s": start_offsets, "max_start_offset_s": max(start_offsets.values()),
            "bag_formal_counts": {rid: len(v) for rid, v in bag_samples.items()},
            "bag_formal_agents": sorted(bag_samples), "recording_complete": True}


class Probe:
    """A bounded latest-packet view; ROS callbacks never wait for stage changes."""
    def __init__(self, rospy, message_type):
        self.rospy = rospy
        self.lock = threading.RLock()
        self.session = None
        self.recorder = None
        self.telemetry = {}
        self.subscribers = [rospy.Subscriber("/ne_tvft/session", message_type, self.on_session, queue_size=30),
                            rospy.Subscriber("/ne_tvft/telemetry", message_type, self.on_agent, queue_size=2000),
                            rospy.Subscriber("/ne_tvft/recorder_status", message_type, self.on_recorder, queue_size=30)]

    def receive(self, target, msg):
        try:
            packet = decode(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            received = (packet, time.monotonic())
            if target == "telemetry":
                self.telemetry[packet.get("id")] = received
            else:
                setattr(self, target, received)

    def on_session(self, msg):
        self.receive("session", msg)

    def on_agent(self, msg):
        self.receive("telemetry", msg)

    def on_recorder(self, msg):
        self.receive("recorder", msg)

    def snapshot(self):
        with self.lock:
            return self.session, self.recorder, self.telemetry.copy()

    def wait(self, label, predicate, timeout, ignore_fault=False, require_fresh=False, ids=()):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self.rospy.is_shutdown():
            session, recorder, telemetry = self.snapshot()
            if not ignore_fault:
                if session and session[0].get("phase") == "FAULT":
                    raise SmokeFailure("Supervisor FAULT: " + str(session[0].get("reason", "")))
                for rid in ids:
                    packet = telemetry.get(rid, ({}, 0))[0]
                    if packet.get("phase") == "FAULT" or packet.get("fault_reason"):
                        raise SmokeFailure("Agent fault: " + rid + ": " + str(packet.get("fault_reason", packet.get("reason", ""))))
            if require_fresh:
                wall = time.monotonic()
                now = self.rospy.Time.now().to_sec()
                if not session or wall - session[1] > .5 or not finite_number(session[0].get("stamp")) or not -.1 <= now - session[0]["stamp"] <= .5:
                    raise SmokeFailure("Supervisor heartbeat became stale")
                for rid in ids:
                    p, received = telemetry.get(rid, ({}, 0))
                    if (wall - received > .3 or not p.get("healthy")
                            or not finite_number(p.get("stamp")) or not -.1 <= now - p["stamp"] <= .3):
                        raise SmokeFailure("Missing, stale, or unhealthy telemetry: " + rid)
            if predicate(session, recorder, telemetry):
                return session[0] if session else None
            time.sleep(.02)
        phase = self.session[0].get("phase") if self.session else "no heartbeat"
        raise SmokeFailure("Timed out waiting for " + label + "; current phase=" + str(phase))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--transition-timeout", type=float, default=65.0)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    report = {"success": False, "ros_runtime_executed": False, "steps": [],
              "verification_scope": "ROS Noetic mock fleet with ideal integrators; not PX4 SITL or hardware",
              "started_at": datetime.now(timezone.utc).isoformat(),
              "requested_duration_s": args.duration if finite_number(args.duration) else None}
    probe = publisher = rospy = None
    commands_started = False
    ids = []
    try:
        if not args.output_root.is_absolute() or not args.report.is_absolute():
            raise SmokeFailure("--output-root and --report must be absolute paths")
        if not all(finite_number(v) and v > 0 for v in (args.duration, args.transition_timeout, args.startup_timeout)):
            raise SmokeFailure("Durations and timeouts must be finite and positive")
        check_master_uri(os.environ.get("ROS_MASTER_URI", "http://localhost:11311"))
        import rospy as ros
        import rosgraph
        from std_msgs.msg import Int32, String
        rospy = ros
        rospy.init_node("ne_tvft_smoke_mock", anonymous=True, disable_signals=True)
        probe = Probe(rospy, String)
        publisher = rospy.Publisher("/command", Int32, queue_size=5)
        probe.wait("initial session heartbeat", lambda s, r, a: s is not None and publisher.get_num_connections() > 0,
                   args.startup_timeout)
        report["ros_runtime_executed"] = True
        config = rospy.get_param("/ne_tvft/config")
        ids = agent_ids(config)
        if args.duration + 2 >= config["runtime"].get("max_run_seconds", 120):
            raise SmokeFailure("Duration must leave time before the configured automatic stop")
        configured_root = Path(config["logging"]["output_root"]).expanduser()
        if not configured_root.is_absolute() or configured_root.resolve() != args.output_root.resolve():
            raise SmokeFailure("Launch output_root differs from --output-root")
        if config["logging"].get("rosbag", True) is not True or config["logging"].get("bag_required", True) is not True:
            raise SmokeFailure("This test requires rosbag recording and its startup barrier")
        params = {"/recorder/mock": rospy.get_param("/recorder/mock", None)}
        for rid in ids:
            for key in ("mock", "allow_hardware", "agent_id"):
                name = "/agent_" + rid + "/" + key
                params[name] = rospy.get_param(name, None)
        master = rosgraph.Master(rospy.get_name())

        def graph_ready(_session, _recorder, _telemetry):
            try:
                check_mock_graph(config, params, master.getSystemState())
                return True
            except SmokeFailure:
                return False

        probe.wait("registered mock-only ROS graph", graph_ready, args.startup_timeout, ids=ids)
        report["mock_graph_verified"] = True
        phase = probe.snapshot()[0][0]["phase"]
        if phase not in {"IDLE", "LANDED"}:
            raise SmokeFailure("Use a fresh mock launch; initial phase must be IDLE or LANDED")
        probe.wait("all nine healthy sensor streams", lambda s, r, a: all(
            rid in a and a[rid][0].get("healthy") and time.monotonic() - a[rid][1] < .3 for rid in ids),
            args.startup_timeout, ids=ids)

        def command(number, target):
            nonlocal commands_started
            # Revalidate actuator registrations immediately before sending.
            check_mock_graph(config, params, master.getSystemState())
            commands_started = True
            publisher.publish(Int32(data=number))
            status = probe.wait(target, lambda s, r, a: s is not None and s[0].get("phase") == target,
                                args.transition_timeout, ids=ids)
            report["steps"].append({"command": number, "completed_phase": target,
                                    "ros_stamp": status["stamp"], "session_id": status.get("session_id", "")})
            print("command {} -> {}".format(number, target), flush=True)
            return status

        for number, target in ((1, "READY"), (2, "ARMED"), (3, "AIRBORNE"), (4, "POSITIONED")):
            command(number, target)
        running = command(5, "RUNNING")
        sid, t0 = validate_session_id(running["session_id"]), running["t0"]
        if not finite_number(t0) or t0 <= 0:
            raise SmokeFailure("Supervisor returned an invalid start time")
        directory = args.output_root.resolve() / sid
        report.update(session_id=sid, t0=t0, session_directory=str(directory))

        def run_complete(session, recorder, telemetry):
            if session[0].get("phase") != "RUNNING" or session[0].get("session_id") != sid:
                raise SmokeFailure("Session left RUNNING before the requested duration")
            if (not recorder or recorder[0].get("session_id") != sid or not recorder[0].get("healthy")
                    or not recorder[0].get("ready") or time.monotonic() - recorder[1] > .5):
                raise SmokeFailure("Recorder did not stay healthy during RUNNING")
            for rid in ids:
                p = telemetry[rid][0]
                if p.get("session_id") != sid or p.get("phase") != "RUNNING" or abs(p.get("t0", 0) - t0) > 1e-6:
                    raise SmokeFailure("Agent phase/session/start mismatch: " + rid)
            return (rospy.Time.now().to_sec() >= t0 + args.duration
                    and all(formal_packet(telemetry[rid][0], sid, t0) for rid in ids))

        # RUNNING is committed before t0. Allow one session heartbeat for agents
        # to observe the commit, then require fresh, consistent active samples.
        probe.wait("nine agents acknowledging RUNNING", lambda s, r, a: all(
            rid in a and a[rid][0].get("phase") == "RUNNING" and a[rid][0].get("session_id") == sid for rid in ids),
            max(1.0, t0 - rospy.Time.now().to_sec()), ids=ids)
        probe.wait("requested formal control duration", run_complete,
                   args.duration + max(0.0, t0 - rospy.Time.now().to_sec()) + 10,
                   require_fresh=True, ids=ids)
        command(6, "STOPPED")
        probe.wait("recorder flush and finalized bag", lambda s, r, a: r is not None
                   and r[0].get("session_id") == sid and r[0].get("closed") is True,
                   config["logging"].get("tail_seconds", 3) + 30, ids=ids)
        command(7, "LANDED")
        report["recording_validation"] = validate_recording(directory, sid, t0, ids, args.duration,
                                                             config["runtime"]["rate_hz"])
        report["success"] = True
    except (Exception, KeyboardInterrupt) as exc:
        report["error"] = "{}: {}".format(type(exc).__name__, exc)
        print(report["error"], file=sys.stderr, flush=True)
    finally:
        if commands_started and not report["success"] and probe and publisher and not rospy.is_shutdown():
            # Only executed after all pre-command mock checks passed. Recovery
            # never turns a failed integration test into a reported success.
            cleanup = []
            for number, target in ((6, "STOPPED"), (7, "LANDED")):
                try:
                    publisher.publish(Int32(data=number))
                    probe.wait("cleanup " + target, lambda s, r, a: s is not None and s[0].get("phase") == target,
                               15, ignore_fault=True)
                    cleanup.append(target)
                except Exception as exc:
                    cleanup.append(str(exc))
            report["failure_cleanup"] = cleanup
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
        args.report.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.report.with_suffix(args.report.suffix + ".tmp")
        temporary.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
        temporary.replace(args.report)
        if rospy is not None:
            rospy.signal_shutdown("mock integration smoke test finished")
    return 0 if report["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
