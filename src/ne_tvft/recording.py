"""ROS-independent, bounded asynchronous experiment storage.

Missing measurements remain null and carry validity flags. Queue overflow or a
write failure makes the writer unhealthy; neither is silently treated as success.
"""
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import re
import threading
import time


CSV_FIELDS = ["stamp", "tau", "session_id", "id", "group", "role", "phase",
              "healthy", "ready", "reason", "pose_stamp", "velocity_stamp",
              "position", "velocity", "acceleration_estimate", "acceleration_valid",
              "imu_specific_force_body", "input_raw", "input_applied", "limited",
              "start_offset_s", "controller", "backend", "communication", "communication_clock"]


def _default(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    raise TypeError("Not JSON serializable: {}".format(type(value).__name__))


def json_text(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, default=_default)


def validate_session_id(session_id):
    if not isinstance(session_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", session_id):
        raise ValueError("Session ID must contain only letters, digits, underscore and hyphen.")
    return session_id


class SessionWriter:
    """Create an exclusive session directory and write telemetry outside control.

    `write` and `event` return False after overflow, serialization or I/O failure.
    `close` flushes pending data and returns False if any data was lost.
    """
    def __init__(self, root, session_id, metadata, queue_size=10000):
        validate_session_id(session_id)
        if int(queue_size) < 1:
            raise ValueError("queue_size must be positive")
        self.directory = Path(root).expanduser().resolve() / session_id
        self.directory.mkdir(parents=True, exist_ok=False)
        self.session_id = session_id
        self.failed = False
        self.dropped = 0
        self.error = ""
        self.closed = False
        self.written = 0
        self.events_written = 0
        self._lock = threading.Lock()
        self._queue = queue.Queue(maxsize=int(queue_size))
        self._sentinel = object()
        self._last_flush = time.monotonic()
        self._handles = []
        self._metadata = dict(metadata)
        self._metadata.update(session_id=session_id, recording_format_version=1,
                              created_at=self._metadata.get("created_at", datetime.now(timezone.utc).isoformat()))
        try:
            self._save_metadata()
            try:
                import yaml
                (self.directory / "config_used.yaml").write_text(
                    yaml.safe_dump(self._metadata.get("config", {}), allow_unicode=True, sort_keys=False), encoding="utf-8")
            except ImportError:
                # JSON is a valid YAML subset and preserves the full config.
                (self.directory / "config_used.yaml").write_text(json_text(self._metadata.get("config", {})), encoding="utf-8")
            self._telemetry = (self.directory / "telemetry.jsonl").open("x", encoding="utf-8")
            self._handles.append(self._telemetry)
            self._events = (self.directory / "events.jsonl").open("x", encoding="utf-8")
            self._handles.append(self._events)
            self._csv_handle = (self.directory / "telemetry.csv").open("x", encoding="utf-8", newline="")
            self._handles.append(self._csv_handle)
            self._csv = csv.DictWriter(self._csv_handle, fieldnames=CSV_FIELDS)
            self._csv.writeheader()
        except Exception:
            for handle in self._handles:
                handle.close()
            raise
        self._thread = threading.Thread(target=self._worker, name="experiment_writer", daemon=True)
        self._thread.start()

    @property
    def healthy(self):
        return not self.failed and not self.closed

    def _fail(self, error, lost=0):
        with self._lock:
            self.failed = True
            self.error = str(error)
            self.dropped += lost

    def _save_metadata(self):
        # Used only by the storage thread or before it starts.
        temporary = self.directory / "metadata.json.tmp"
        temporary.write_text(json_text(self._metadata) + "\n", encoding="utf-8")
        temporary.replace(self.directory / "metadata.json")

    def _enqueue(self, kind, value):
        if not self.healthy:
            with self._lock:
                self.dropped += 1
            return False
        try:
            # JSON serialization gives a detached, immutable snapshot to the worker.
            encoded = json_text(value)
            self._queue.put_nowait((kind, encoded))
            return True
        except queue.Full:
            self._fail("Recording queue overflow", lost=1)
        except (TypeError, ValueError) as exc:
            self._fail("Invalid telemetry: {}".format(exc), lost=1)
        return False

    def write(self, record):
        return self._enqueue("telemetry", record)

    def event(self, record):
        return self._enqueue("event", record)

    def update_metadata(self, values):
        return self._enqueue("metadata", values)

    def _write_item(self, kind, encoded):
        value = json.loads(encoded)
        if kind == "telemetry":
            self._telemetry.write(encoded + "\n")
            row = {}
            for key in CSV_FIELDS:
                field = value.get(key)
                row[key] = json_text(field) if isinstance(field, (list, dict, tuple)) else field
            self._csv.writerow(row)
            self.written += 1
        elif kind == "event":
            self._events.write(encoded + "\n")
            self.events_written += 1
        elif kind == "metadata":
            self._metadata.update(value)
            self._save_metadata()
        if time.monotonic() - self._last_flush > .5:
            for handle in self._handles:
                handle.flush()
            self._last_flush = time.monotonic()

    def _worker(self):
        try:
            while True:
                item = self._queue.get()
                try:
                    if item is self._sentinel:
                        break
                    try:
                        self._write_item(*item)
                    except Exception as exc:
                        self._fail("Write failure: {}".format(exc), lost=1)
                finally:
                    self._queue.task_done()
        finally:
            for handle in self._handles:
                try:
                    handle.flush()
                    handle.close()
                except Exception as exc:
                    self._fail("Flush failure: {}".format(exc))

    def close(self, timeout=15.0):
        if self.closed:
            return not self.failed
        self.closed = True
        deadline = time.monotonic() + timeout
        while self._thread.is_alive():
            try:
                self._queue.put(self._sentinel, timeout=.1)
                break
            except queue.Full:
                if time.monotonic() >= deadline:
                    self._fail("Writer close queue timeout")
                    return False
        self._thread.join(max(0.0, deadline - time.monotonic()))
        if self._thread.is_alive():
            self._fail("Writer close timeout")
            return False
        status = {"complete": not self.failed, "failed": self.failed, "error": self.error,
                  "dropped": self.dropped, "telemetry_records": self.written,
                  "events": self.events_written}
        try:
            (self.directory / "recording_status.json").write_text(json_text(status) + "\n", encoding="utf-8")
        except Exception as exc:
            self._fail("Status write failed: {}".format(exc))
        return not self.failed


def decode_packet(text):
    """Strict root/type/version checks; bad external packets never execute code."""
    try:
        packet = json.loads(text, parse_constant=lambda value: (_ for _ in ()).throw(ValueError("Nonfinite JSON")))
    except (TypeError, ValueError):
        return None
    if not isinstance(packet, dict) or packet.get("version") != 1:
        return None
    return packet
