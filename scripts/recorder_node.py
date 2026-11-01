#!/usr/bin/env python3
"""ROS recorder with a worker thread and required rosbag readiness handshake."""
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from ne_tvft.recording import SessionWriter, decode_packet


class RecorderNode:
    def __init__(self):
        import rospy
        from std_msgs.msg import String
        self.rospy, self.String = rospy, String
        self.config = rospy.get_param("~config", rospy.get_param("/ne_tvft/config", {}))
        if not self.config:
            raise ValueError("No experiment configuration at ~config or /ne_tvft/config")
        self.mock = bool(rospy.get_param("~mock", False))
        log = self.config.get("logging", {})
        self.root = Path(log.get("output_root", "output")).expanduser()
        if not self.root.is_absolute():
            try:
                import rospkg
                package_root = Path(rospkg.RosPack().get_path("ne_tvft_experiment"))
            except Exception:
                package_root = Path(__file__).resolve().parents[1]
            self.root = package_root / self.root
        self.writer = None
        self.bag = None
        self.bag_log = None
        self.error = ""
        self.session_id = ""
        self.phase = "IDLE"
        self.close_at = None
        self.bag_ready = False
        self.t0_saved = None
        self._lock = threading.RLock()
        self._commands = queue.Queue(maxsize=1000)
        self._shutdown = threading.Event()
        self._latest_session = None
        self.publisher = rospy.Publisher("/ne_tvft/recorder_status", String, queue_size=5)
        self.session_sub = rospy.Subscriber("/ne_tvft/session", String, self.on_session, queue_size=20)
        self.telemetry_sub = rospy.Subscriber("/ne_tvft/telemetry", String, self.on_telemetry, queue_size=2000)
        self.event_sub = rospy.Subscriber("/ne_tvft/events", String, self.on_event, queue_size=100)
        self.command_sub = rospy.Subscriber("/command", __import__("std_msgs.msg", fromlist=["Int32"]).Int32,
                                            self.on_command, queue_size=20)
        self.worker = threading.Thread(target=self.run_worker, name="recording_manager", daemon=True)
        self.worker.start()
        self.timer = rospy.Timer(rospy.Duration(.1), self.publish_status)
        rospy.on_shutdown(self.shutdown)

    def enqueue(self, command, payload):
        try:
            self._commands.put_nowait((command, payload))
        except queue.Full:
            self.error = "Recorder manager queue overflow"

    def on_session(self, message):
        packet = decode_packet(message.data)
        if packet is None or not isinstance(packet.get("phase"), str):
            self.rospy.logwarn_throttle(2, "Ignoring malformed session JSON")
            return
        # Keep the session's latest phase; repeated heartbeat traffic never blocks control.
        self._latest_session = packet
        if (packet.get("session_id"), packet.get("phase"), packet.get("t0")) != (
                self.session_id, self.phase, self.t0_saved):
            self.enqueue("session", packet)

    def on_command(self, message):
        with self._lock:
            if self.writer and self.writer.healthy:
                self.writer.event({"stamp": self.rospy.Time.now().to_sec(), "type": "command", "command": message.data})

    def on_event(self, message):
        packet = decode_packet(message.data)
        if packet is None:
            return
        with self._lock:
            if self.writer and self.writer.healthy and packet.get("session_id", self.session_id) == self.session_id:
                self.writer.event(packet)

    def on_telemetry(self, message):
        packet = decode_packet(message.data)
        if packet is None or "id" not in packet or "stamp" not in packet:
            return
        with self._lock:
            writer = self.writer
            if writer is not None and packet.get("session_id") == self.session_id:
                writer.write(packet)

    def bag_topics(self):
        topics = {"/command", "/ne_tvft/session", "/ne_tvft/telemetry", "/ne_tvft/recorder_status", "/ne_tvft/events"}
        for name, robot in self.config.get("robots", {}).items():
            topics.add("/ne_tvft/signal/" + name)
            for key, value in robot.items():
                if key.endswith("_topic") and isinstance(value, str) and value:
                    if self.mock and not value.startswith("/mock/"):
                        value = "/mock" + (value if value.startswith("/") else "/" + value)
                    topics.add(value)
        return sorted(topics)

    def start_bag(self):
        if not self.config.get("logging", {}).get("rosbag", True):
            if self.config.get("logging", {}).get("bag_required", True):
                raise RuntimeError("bag_required is true but rosbag is disabled")
            self.bag_ready = True
            return
        self.bag_log = (self.writer.directory / "rosbag.log").open("w", encoding="utf-8")
        output = self.writer.directory / "raw.bag"
        self.bag = subprocess.Popen(["rosbag", "record", "--lz4", "-O", str(output)] + self.bag_topics(),
                                    stdout=self.bag_log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        started = time.monotonic()
        while not self._shutdown.is_set() and time.monotonic() - started < 8:
            if self.bag.poll() is not None:
                raise RuntimeError("rosbag exited during startup; inspect rosbag.log")
            if any(self.writer.directory.glob("raw.bag*.active")):
                self.bag_ready = True
                return
            time.sleep(.05)
        raise RuntimeError("rosbag did not create its active output file before timeout")

    def close_session(self):
        self.bag_ready = False
        if self.bag is not None:
            if self.bag.poll() is None:
                try:
                    if os.name == "posix":
                        os.killpg(self.bag.pid, signal.SIGINT)
                    else:
                        self.bag.send_signal(signal.SIGINT)
                    self.bag.wait(timeout=8)
                except (OSError, subprocess.TimeoutExpired):
                    self.bag.kill()
                    self.bag.wait(timeout=3)
                    self.error = "rosbag required forced termination; bag may be incomplete"
            elif self.bag.returncode not in (0, -signal.SIGINT):
                self.error = "rosbag exited unexpectedly: {}".format(self.bag.returncode)
            self.bag = None
        if self.bag_log:
            self.bag_log.close()
            self.bag_log = None
        with self._lock:
            if self.writer:
                self.writer.event({"stamp": self.rospy.Time.now().to_sec(), "type": "recording_end", "reason": self.error})
                self.writer.close()
                if self.writer.failed:
                    self.error = self.writer.error
        self.close_at = None

    def handle_session(self, packet):
        sid, phase = packet.get("session_id", ""), packet["phase"]
        if phase == "RECORDING" and sid and sid != self.session_id:
            if self.writer and not self.writer.closed:
                self.close_session()
            self.session_id, self.error = sid, ""
            self.bag_ready = False
            with self._lock:
                self.writer = SessionWriter(self.root, sid, {"config": self.config, "session": packet,
                                             "mock": self.mock, "firmware": self.config.get("hardware", {})},
                                             self.config.get("logging", {}).get("queue_size", 10000))
            self.start_bag()
        self.phase = phase
        if self.writer and not self.writer.closed and sid == self.session_id:
            self.writer.event({"stamp": self.rospy.Time.now().to_sec(), "type": "session", "session": packet})
            self.t0_saved = packet.get("t0")
            self.writer.update_metadata({"t0": self.t0_saved, "last_phase": phase,
                                         "session": packet, "bag_topics": self.bag_topics()})
            if phase in ("STOPPED", "FAULT", "LANDED") and self.close_at is None:
                self.close_at = time.monotonic() + float(self.config.get("logging", {}).get("tail_seconds", 3))

    def run_worker(self):
        while not self._shutdown.is_set():
            try:
                command, payload = self._commands.get(timeout=.05)
                try:
                    if command == "session":
                        self.handle_session(payload)
                except Exception as exc:
                    self.error = "Recorder failure: {}".format(exc)
                    self.rospy.logerr(self.error)
                    self.close_session()
                finally:
                    self._commands.task_done()
            except queue.Empty:
                pass
            if self.bag and self.bag.poll() is not None and not self.error:
                self.error = "rosbag process died while recording"
            if self.close_at is not None and time.monotonic() >= self.close_at:
                self.close_session()
        self.close_session()

    def publish_status(self, _event):
        with self._lock:
            writer = self.writer
            healthy = bool(writer and writer.healthy and not self.error)
            payload = {"version": 1, "session_id": self.session_id, "phase": self.phase,
                       "stamp": self.rospy.Time.now().to_sec(),
                       "ready": healthy and self.bag_ready,
                       "healthy": healthy, "reason": self.error or (writer.error if writer else "No open session"),
                       "dropped": writer.dropped if writer else 0,
                       "closed": bool(writer and writer.closed)}
        self.publisher.publish(self.String(data=json.dumps(payload, allow_nan=False)))

    def shutdown(self):
        self._shutdown.set()
        if self.worker.is_alive():
            self.worker.join(timeout=25)


if __name__ == "__main__":
    import rospy
    rospy.init_node("ne_tvft_recorder")
    RecorderNode()
    rospy.spin()
