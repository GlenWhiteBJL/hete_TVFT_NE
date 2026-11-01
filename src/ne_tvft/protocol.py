"""Deterministic experiment supervisor. No ROS imports and no hardware actions."""
import json
import math
import re
from datetime import datetime, timezone, timedelta

VERSION = 1
PHASES = {"IDLE", "PREPARING", "READY", "ARMING", "ARMED", "TAKING_OFF", "AIRBORNE",
          "POSITIONING", "POSITIONED", "RECORDING", "SCHEDULED", "RUNNING", "STOPPING",
          "STOPPED", "LANDING", "LANDED", "FAULT"}


def jsonable(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, dict):
        return {k: jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    return value


def encode(value):
    return json.dumps(jsonable(value), allow_nan=False, separators=(",", ":"))


def decode(payload):
    value = json.loads(payload, parse_constant=lambda x: (_ for _ in ()).throw(ValueError("Nonfinite JSON")))
    if not isinstance(value, dict) or value.get("version") != VERSION:
        raise ValueError("Expected protocol version 1 object")
    return value


def agent_ids(config):
    return [robot for group in config["groups"] for robot in [group["leader"]] + group["followers"]]


class Supervisor:
    """Prepare, feedback transitions, recorder barrier, then future-time commit.

    All timestamps are from the same ROS clock. Receipt/connection watchdogs
    also exist in the ROS executor, using a monotonic wall clock.
    """
    def __init__(self, config):
        self.config = config
        self.ids = agent_ids(config)
        if len(self.ids) != len(set(self.ids)):
            raise ValueError("Robot ids must be unique")
        self.phase, self.session_id, self.t0 = "IDLE", "", 0.0
        self.reason, self.entered, self.last_time = "", 0.0, None
        self.reports, self.recorder = {}, None
        self.serial = 0
        self.commit_acked = False
        self.events = []

    def transition(self, phase, now, reason=""):
        if phase not in PHASES:
            raise ValueError("Unknown phase")
        old = self.phase
        self.phase, self.entered, self.reason = phase, now, reason
        self.events.append({"stamp": now, "from": old, "to": phase, "reason": reason, "session_id": self.session_id})

    def status(self, now):
        return {"version": VERSION, "phase": self.phase, "session_id": self.session_id,
                "t0": self.t0, "stamp": now, "reason": self.reason}

    def report(self, packet, now):
        if packet.get("id") not in self.ids or packet.get("version") != VERSION:
            raise ValueError("Unexpected telemetry producer")
        stamp=float(packet.get("stamp",-math.inf))
        if not math.isfinite(stamp) or not -.1 <= now-stamp <= self.config["runtime"]["state_timeout"]:
            raise ValueError("Stale or invalid telemetry source stamp")
        self.reports[packet["id"]] = (now, packet)

    def recorder_report(self, packet, now):
        if packet.get("version") != VERSION:
            raise ValueError("Invalid recorder status")
        stamp=float(packet.get("stamp",-math.inf))
        if not math.isfinite(stamp) or not -.1 <= now-stamp <= self.config["runtime"]["session_timeout"]:
            raise ValueError("Stale or invalid recorder source stamp")
        self.recorder = (now, packet)

    def _all_ready(self, now, phase=None):
        timeout = self.config["runtime"]["state_timeout"]
        for rid in self.ids:
            received, p = self.reports.get(rid, (-math.inf, {}))
            if (now-received > timeout or not p.get("healthy") or not p.get("ready")
                    or p.get("phase") != (phase or self.phase)):
                return False
            if self.session_id and p.get("session_id") != self.session_id:
                return False
            if (phase or self.phase) in {"SCHEDULED","RUNNING"}:
                try:
                    if not math.isfinite(float(p.get("t0",-1))) or abs(float(p.get("t0",-1))-self.t0)>1e-9:
                        return False
                except (ValueError,TypeError):
                    return False
        return True

    def command(self, number, now):
        if isinstance(number, bool) or number not in range(1,8):
            return False, "Unknown command"
        # Idempotent repeated requests never reset a controller or create logs.
        same = {1: {"PREPARING","READY"},2:{"ARMING","ARMED"},3:{"TAKING_OFF","AIRBORNE"},
                4:{"POSITIONING","POSITIONED"},5:{"RECORDING","SCHEDULED","RUNNING"},
                6:{"STOPPING","STOPPED"},7:{"LANDING","LANDED"}}
        if self.phase in same[number]:
            return True, "Already requested"
        if number == 6:
            self.transition("STOPPING",now,"operator stop")
            return True,"Stopping"
        allowed = {1:{"IDLE","STOPPED","LANDED"},2:{"READY"},3:{"ARMED"},
                   4:{"AIRBORNE","STOPPED"},5:{"POSITIONED"},7:{"AIRBORNE","POSITIONED","STOPPED","FAULT"}}
        if self.phase not in allowed[number]:
            return False,"Illegal transition from " + self.phase
        if number == 1:
            self.session_id,self.t0="",0.0
        if number == 5:
            self.serial+=1
            stamp=datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")
            self.session_id=stamp+"_"+str(self.serial)
            self.t0=0.0
            self.commit_acked=False
            self.recorder=None
        target={1:"PREPARING",2:"ARMING",3:"TAKING_OFF",4:"POSITIONING",5:"RECORDING",7:"LANDING"}[number]
        self.transition(target,now,"command " + str(number))
        return True,"Accepted"

    def tick(self, now):
        rt=self.config["runtime"]
        if not math.isfinite(now):
            raise ValueError("Nonfinite time")
        if self.last_time is not None and now < self.last_time:
            self.transition("FAULT",now,"ROS clock moved backward")
        self.last_time=now
        if self.phase in {"IDLE","STOPPED","LANDED","FAULT"}:
            return self.status(now)
        complete={"PREPARING":"READY","ARMING":"ARMED","TAKING_OFF":"AIRBORNE",
                  "POSITIONING":"POSITIONED","STOPPING":"STOPPED","LANDING":"LANDED"}
        if self.phase in complete and self._all_ready(now):
            self.transition(complete[self.phase],now,"all actual feedback ready")
        elif self.phase=="RECORDING":
            received,p=self.recorder or (-math.inf,{})
            if (now-received < rt["session_timeout"] and p.get("session_id")==self.session_id
                    and p.get("healthy") and p.get("ready") and self._all_ready(now)):
                self.t0=now+rt["start_lead_seconds"]
                self.transition("SCHEDULED",now,"recorder open; prepare all controllers")
        elif self.phase=="SCHEDULED":
            if now >= self.t0-rt["commit_margin"]:
                self.transition("FAULT",now,"startup acknowledgement deadline missed")
            elif self._all_ready(now):
                self.transition("RUNNING",now,"all controllers acknowledged; committed future start")
        if self.phase in {"RECORDING","SCHEDULED","RUNNING"}:
            received,p=self.recorder or (-math.inf,{})
            if (self.phase!="RECORDING" and (now-received > rt["session_timeout"] or not p.get("healthy")
                    or p.get("session_id")!=self.session_id)):
                self.transition("FAULT",now,"recorder heartbeat/write failure")
        if self.phase in {"TAKING_OFF","AIRBORNE","POSITIONING","POSITIONED","RECORDING","SCHEDULED"}:
            for rid in self.ids:
                received,p=self.reports.get(rid,(-math.inf,{}))
                if now-received>rt["state_timeout"] or not p.get("healthy"):
                    self.transition("FAULT",now,"agent unhealthy during flight preparation: "+rid)
                    break
        if self.phase=="RUNNING":
            if now >= self.t0-rt["commit_margin"] and not self.commit_acked:
                if self._all_ready(now,"RUNNING"):
                    self.commit_acked=True
                else:
                    self.transition("FAULT",now,"not all agents acknowledged start commit")
            for rid in self.ids:
                received,p=self.reports.get(rid,(-math.inf,{}))
                if now-received > rt["state_timeout"] or not p.get("healthy"):
                    self.transition("FAULT",now,"agent unhealthy or missing: " + rid)
                    break
                if now>=self.t0 and (p.get("phase")!="RUNNING" or p.get("session_id")!=self.session_id):
                    self.transition("FAULT",now,"agent phase/session mismatch: " + rid)
                    break
            if self.phase=="RUNNING" and now-self.t0>=rt.get("max_run_seconds",120):
                self.transition("STOPPING",now,"configured experiment duration reached")
        if self.phase not in {"IDLE","READY","ARMED","AIRBORNE","POSITIONED","RUNNING","STOPPED","LANDED","FAULT"}:
            if now-self.entered > rt["transition_timeout"]:
                self.transition("FAULT",now,"phase feedback timeout")
        return self.status(now)
