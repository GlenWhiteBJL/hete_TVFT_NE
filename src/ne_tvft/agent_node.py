"""One ROS node per physical agent. Sole owner of its actuator publisher.

ROS imports are confined to this module. Core mathematics and frame adapters can
be tested without ROS. The node never subscribes to non-neighbor control signals.
"""
import copy
import json
import math
import queue
import threading
import time
from pathlib import Path
import numpy as np
import yaml
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from .adapters import (PlanarTransform, finite_vector, quaternion_yaw, rotation,
    ugv_command, position_target, velocity_target, acceleration_target,
    height_acceleration, limit_norm, sample_is_fresh, DerivativeEstimate, StartGate)
from .core import LeaderController, FollowerController, circle

PHASES = {"IDLE", "PREPARING", "READY", "ARMING", "ARMED", "TAKING_OFF", "AIRBORNE",
          "POSITIONING", "POSITIONED", "RECORDING", "SCHEDULED", "RUNNING", "STOPPING",
          "STOPPED", "LANDING", "LANDED", "FAULT"}


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def load_config():
    if rospy.has_param("/ne_tvft/config"):
        config = copy.deepcopy(rospy.get_param("/ne_tvft/config"))
    else:
        with open(rospy.get_param("~config_file"), encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
    topics = rospy.get_param("~topics_file", "")
    if topics:
        with open(topics, encoding="utf-8") as stream:
            extra = yaml.safe_load(stream) or {}
        config.setdefault("robots", {}).update(extra.get("robots", {}))
    return config


class AgentNode:
    def __init__(self):
        self.config = load_config()
        self.agent_id = str(rospy.get_param("~agent_id"))
        self.robot = self.config["robots"][self.agent_id]
        self.is_uav = self.robot["kind"] == "uav"
        self.group, self.follower = self._membership()
        self.runtime = self.config["runtime"]
        self.limits = self.config["limits"]
        self.mock = bool(rospy.get_param("~mock", False))
        hardware = self.config.get("hardware", {})
        self.hardware_allowed = (self.mock or (bool(rospy.get_param("~allow_hardware", False))
            and hardware.get("enabled", False) and hardware.get("frames_verified", False)
            and hardware.get("acceleration_interface_verified", False)))
        self.sensor_transform = PlanarTransform(self.robot.get("transform"))
        self.velocity_transform = PlanarTransform(self.robot.get("velocity_transform", self.robot.get("transform")))
        self.setpoint_transform = PlanarTransform(self.robot.get("setpoint_transform"))
        if self.is_uav and self.robot.get("velocity_frame","world") == "body":
            raise ValueError("UAV velocity must be local/world ENU; body velocity needs full attitude rotation and is not supported")
        self.lock = threading.RLock()
        self.position = self.velocity = self.acceleration = None
        self.yaw = 0.0
        self.pose_stamp = self.velocity_stamp = 0.0
        self.pose_receipt = self.velocity_receipt = -math.inf
        self.pose_error = self.velocity_error = "waiting for sensors"
        self.velocity_timestamp_origin = "source"
        self.velocity_header_frame = ""
        self.derivative = DerivativeEstimate(self.runtime["state_timeout"])
        self.imu_force = None
        self.imu_stamp = 0.0
        self.imu_receipt = -math.inf
        self.fcu = {"connected": False, "armed": False, "mode": ""}
        self.fcu_receipt = -math.inf
        self.landed_state = 0
        self.landed_receipt = -math.inf
        self.session = {"phase": "IDLE", "session_id": "", "t0": 0.0}
        self.session_receipt = time.monotonic()
        self.gate = StartGate()
        self.controller = (FollowerController(self.config, self.group, self.follower)
                           if self.is_uav else LeaderController(self.config, self.group))
        self.controller_initialized = False
        self.neighbors = {}
        self.fault = ""
        self.last_tick = None
        self.start_offset = None
        self.last_control_time = None
        self.reference_tau = 0.0
        self.at_target_since = None
        self.stop_hold = None
        self.hold_position = None
        self.takeoff_xy = None
        self.takeoff_z = None
        self.service_queue = queue.Queue(maxsize=1)
        self.service_busy = False
        self.last_service = -math.inf
        self.service_error = ""
        self.last_service_result = None
        self.first_setpoint = None
        self.expected_ids = self._neighbor_ids()
        self.telemetry_pub = rospy.Publisher("/ne_tvft/telemetry", String, queue_size=100)
        self.signal_pub = rospy.Publisher("/ne_tvft/signal/"+self.agent_id, String, queue_size=5)
        self.command_pub = rospy.Publisher(self.external(self.robot["command_topic"]),
                                          PositionTarget if self.is_uav else Twist, queue_size=1)
        self.subscribers = [rospy.Subscriber("/ne_tvft/session", String, self.session_callback, queue_size=10)]
        self._sensor_subscriptions()
        for agent_id in self.expected_ids:
            self.subscribers.append(rospy.Subscriber("/ne_tvft/signal/"+agent_id, String,
                lambda msg, aid=agent_id: self.signal_callback(aid, msg), queue_size=5))
        threading.Thread(target=self.service_worker, daemon=True).start()
        self.timer = rospy.Timer(rospy.Duration(1.0/self.runtime["rate_hz"]), self.tick, reset=True)
        rospy.on_shutdown(self.shutdown)

    def external(self, topic):
        return "/mock"+topic if self.mock and not topic.startswith("/mock/") else topic

    def _membership(self):
        for i, group in enumerate(self.config["groups"]):
            if self.agent_id == group["leader"]:
                return i, None
            if self.agent_id in group["followers"]:
                return i, group["followers"].index(self.agent_id)
        raise ValueError("Agent not assigned to any group: "+self.agent_id)

    def _neighbor_ids(self):
        groups, topology = self.config["groups"], self.config["topology"]
        if not self.is_uav:
            return {groups[k]["leader"]: k for k, weight in enumerate(topology["leaders"][self.group]) if weight > 0}
        result = {}
        if topology["pinning"][self.group][self.follower] > 0:
            result[groups[self.group]["leader"]] = 0
        for k, weight in enumerate(topology["followers"][self.follower]):
            if weight > 0:
                result[groups[self.group]["followers"][k]] = k+1
        return result

    def _sensor_subscriptions(self):
        if self.robot.get("pose_type", "PoseStamped") != "PoseStamped":
            raise ValueError("Pose source must publish geometry_msgs/PoseStamped")
        self.subscribers.append(rospy.Subscriber(self.external(self.robot["pose_topic"]), PoseStamped, self.pose_callback, queue_size=5))
        vtype = self.robot.get("velocity_type", "TwistStamped")
        if vtype not in ("TwistStamped", "Twist"):
            raise ValueError("velocity_type must be Twist or TwistStamped")
        self.subscribers.append(rospy.Subscriber(self.external(self.robot["velocity_topic"]),
            Twist if vtype == "Twist" else TwistStamped, self.velocity_callback, queue_size=5))
        if self.robot.get("imu_topic"):
            self.subscribers.append(rospy.Subscriber(self.external(self.robot["imu_topic"]), Imu, self.imu_callback, queue_size=5))
        if self.is_uav:
            self.subscribers.append(rospy.Subscriber(self.external(self.robot["state_topic"]), State, self.state_callback, queue_size=5))
            self.subscribers.append(rospy.Subscriber(self.external(self.robot["extended_state_topic"]), ExtendedState, self.landed_callback, queue_size=5))

    def pose_callback(self, msg):
        with self.lock:
            try:
                expected = self.robot.get("pose_frame", "")
                if expected and msg.header.frame_id != expected:
                    raise ValueError("Pose frame %r, expected %r" % (msg.header.frame_id, expected))
                stamp = msg.header.stamp.to_sec()
                if stamp <= 0 or stamp <= self.pose_stamp:
                    raise ValueError("Zero or non-increasing pose timestamp")
                p, q = msg.pose.position, msg.pose.orientation
                self.position = self.sensor_transform.point_to_world([p.x, p.y, p.z])
                self.yaw = quaternion_yaw([q.x, q.y, q.z, q.w])+self.sensor_transform.yaw
                self.pose_stamp, self.pose_receipt = stamp, time.monotonic()
                self.pose_error = ""
            except (ValueError, TypeError) as exc:
                self.pose_error = str(exc)

    def velocity_callback(self, msg):
        with self.lock:
            try:
                stamped = hasattr(msg, "header")
                header_frame = str(msg.header.frame_id) if stamped else ""
                expected_frame = self.robot.get("velocity_header_frame", "")
                if expected_frame and header_frame != expected_frame:
                    raise ValueError("Velocity frame %r, expected %r" % (header_frame,expected_frame))
                stamp = msg.header.stamp.to_sec() if stamped else rospy.Time.now().to_sec()
                if stamp <= 0 or stamp <= self.velocity_stamp:
                    raise ValueError("Zero or non-increasing velocity timestamp")
                v = (msg.twist if stamped else msg).linear
                value = finite_vector([v.x, v.y, v.z])
                frame = self.robot.get("velocity_frame", "world")
                if frame == "body":
                    if self.position is None:
                        raise ValueError("Body-frame velocity requires a valid pose")
                    value = rotation(self.yaw) @ value
                elif frame == "world":
                    value = self.velocity_transform.vector_to_world(value)
                else:
                    raise ValueError("Unknown velocity_frame")
                self.velocity = value
                self.acceleration = self.derivative.update(value, stamp)
                self.velocity_stamp, self.velocity_receipt = stamp, time.monotonic()
                self.velocity_timestamp_origin = "source" if stamped else "receipt_ros_time"
                self.velocity_header_frame = header_frame
                self.velocity_error = ""
            except (ValueError, TypeError) as exc:
                self.velocity_error = str(exc)

    def imu_callback(self, msg):
        with self.lock:
            try:
                a = msg.linear_acceleration
                self.imu_force = finite_vector([a.x, a.y, a.z])
                self.imu_stamp, self.imu_receipt = msg.header.stamp.to_sec(), time.monotonic()
            except ValueError:
                self.imu_force = None

    def state_callback(self, msg):
        with self.lock:
            self.fcu = {"connected": bool(msg.connected), "armed": bool(msg.armed), "mode": str(msg.mode)}
            self.fcu_receipt = time.monotonic()

    def landed_callback(self, msg):
        with self.lock:
            self.landed_state = int(msg.landed_state)
            self.landed_receipt = time.monotonic()

    def session_callback(self, msg):
        with self.lock:
            try:
                payload = json.loads(msg.data)
                phase = payload["phase"]
                sid, t0 = str(payload.get("session_id", "")), float(payload.get("t0", 0.0))
                if payload.get("version") != 1 or phase not in PHASES or not math.isfinite(t0):
                    raise ValueError("Invalid session protocol")
                now = rospy.Time.now().to_sec()
                previous = self.session["phase"]
                if phase == "SCHEDULED":
                    if self.gate.prepare(sid, t0, now):
                        if self.position is None:
                            raise ValueError("Scheduled without own pose")
                        self._reset_controller()
                        self.neighbors.clear()
                        self.start_offset = self.last_control_time = None
                        self.reference_tau = 0.0
                elif phase == "RUNNING":
                    self.gate.commit(sid, t0, now)
                if phase != previous:
                    self.at_target_since = None
                    if self.position is not None:
                        self.hold_position = self.position.copy()
                        if self.is_uav and phase in ("RECORDING","SCHEDULED","RUNNING") and previous in ("POSITIONED","RECORDING","SCHEDULED"):
                            self.hold_position = self.initial_uav_point()
                    if phase in ("STOPPING", "FAULT"):
                        self.stop_hold = None
                    if phase == "TAKING_OFF" and self.position is not None:
                        self.takeoff_xy = self.position[:2].copy()
                        self.takeoff_z = float(self.position[2])
                    if phase == "PREPARING" and (not self.is_uav or (self.landed_state == 1 and not self.fcu["armed"])):
                        self.fault = ""
                        self.gate = StartGate()
                        self.first_setpoint = None
                self.session = {"phase": phase, "session_id": sid, "t0": t0}
                self.session_receipt = time.monotonic()
            except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
                self.fault = "session: "+str(exc)

    def _reset_controller(self):
        if self.is_uav:
            self.controller.reset()
        else:
            self.controller.reset(self.position[:2])
        self.controller_initialized = True

    def initial_uav_point(self):
        h,_,_ = circle(self.config,self.group,self.follower,0.0)
        group = self.config["groups"][self.group]
        return np.r_[np.asarray(group["initial_xy"])+h,group["altitude"]]

    def signal_callback(self, agent_id, msg):
        with self.lock:
            try:
                p = json.loads(msg.data)
                if p.get("version") != 1 or p.get("id") != agent_id:
                    raise ValueError("Signal identity mismatch")
                p["stamp"], p["t0"] = float(p["stamp"]), float(p["t0"])
                if not math.isfinite(p["stamp"]) or not math.isfinite(p["t0"]):
                    raise ValueError("Nonfinite signal timestamp")
                p["w"] = finite_vector(p["w"], 2).tolist()
                if not self.is_uav:
                    p["y"] = finite_vector(p["y"], 2).tolist()
                    z = np.asarray(p["z"], dtype=float)
                    if z.shape != (len(self.config["groups"]), 2) or not np.isfinite(z).all():
                        raise ValueError("Invalid neighbor estimate array")
                self.neighbors[agent_id] = (p, time.monotonic())
            except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                self.neighbors.pop(agent_id, None)

    def _sensors_healthy(self, now, wall):
        timeout = self.runtime["state_timeout"]
        if self.pose_error or self.velocity_error:
            return False, self.pose_error or self.velocity_error
        if not sample_is_fresh(now, wall-self.pose_receipt, self.pose_stamp, timeout):
            return False, "stale pose"
        if not sample_is_fresh(now, wall-self.velocity_receipt, self.velocity_stamp, timeout):
            return False, "stale velocity"
        if self.is_uav and (not self.fcu["connected"] or wall-self.fcu_receipt > self.runtime.get("fcu_state_timeout",3.0)):
            return False, "FCU disconnected or stale state"
        return True, ""

    def _signals(self, now, wall):
        result, complete = {}, True
        m = len(self.config["groups"])
        for aid, index in self.expected_ids.items():
            pair = self.neighbors.get(aid)
            valid = False
            if pair is not None:
                p, receipt = pair
                valid = (sample_is_fresh(now, wall-receipt, p["stamp"], self.runtime["signal_timeout"])
                         and p.get("session_id") == self.session["session_id"]
                         and abs(p["t0"]-self.session["t0"]) < 1e-9)
            if valid:
                result[index] = np.asarray(p["w"]) if self.is_uav else {"y": np.asarray(p["y"]), "z": np.asarray(p["z"])}
            else:
                complete = False
                # Output evaluations only: placeholders permit own immutable state
                # broadcasting while the start barrier is filling neighbor caches.
                result[index] = np.zeros(2) if self.is_uav else {"y": np.zeros(2), "z": np.zeros((m,2))}
        return result, complete

    def _ready_with_dwell(self, predicate, now):
        if not predicate:
            self.at_target_since = None
            return False
        if self.at_target_since is None:
            self.at_target_since = now
        return now-self.at_target_since >= self.runtime.get("ready_dwell", .5)

    def request_service(self, mode=None, armed=None):
        wall = time.monotonic()
        if not self.hardware_allowed or self.service_busy or wall-self.last_service < self.runtime.get("service_interval",1.0):
            return
        try:
            self.service_busy, self.last_service = True, wall
            self.service_queue.put_nowait((mode, armed, self.session["phase"], self.session["session_id"]))
        except queue.Full:
            self.service_busy = False

    def service_worker(self):
        while not rospy.is_shutdown():
            try:
                job = self.service_queue.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                self._process_service_job(job)
            except Exception as exc:
                self.service_error = str(exc)
                rospy.logwarn_throttle(2.0, "%s service: %s", self.agent_id, exc)
            finally:
                self.service_busy = False
                self.service_queue.task_done()

    def _service_is_current(self, job):
        mode, armed, phase, sid = job
        with self.lock:
            if not self.hardware_allowed or self.session["phase"] != phase or self.session["session_id"] != sid:
                return False
            if mode == "AUTO.LAND" and armed is None:
                return phase == "LANDING" and self.fcu["connected"]
            if ((mode == "OFFBOARD" and armed is None) or (mode is None and armed is True)):
                return phase == "ARMING" and not self.fault and self._sensors_healthy(rospy.Time.now().to_sec(),time.monotonic())[0]
            return False

    def _process_service_job(self, job):
        mode, armed, phase, sid = job
        record = {"mode":mode,"armed":armed,"requested_phase":phase,"session_id":sid,
                  "request_stamp":rospy.Time.now().to_sec(),"outcome":"cancelled","sent":False}
        self.last_service_result = record
        if not self._service_is_current(job):
            return
        name = self.external(self.robot["mode_service" if mode is not None else "arming_service"])
        rospy.wait_for_service(name,timeout=2.0)
        # Recheck after a potentially blocking wait. A previous ARMING request
        # must not arm a vehicle after the user has issued STOP or LAND.
        if not self._service_is_current(job):
            return
        record["sent"], record["outcome"] = True,"in_flight"
        if mode is not None:
            response = rospy.ServiceProxy(name,SetMode)(base_mode=0,custom_mode=mode)
            accepted = bool(response.mode_sent)
        else:
            response = rospy.ServiceProxy(name,CommandBool)(value=armed)
            accepted = bool(response.success)
        record["outcome"] = "accepted" if accepted else "rejected"
        record["phase_changed_during_call"] = self.session["phase"] != phase or self.session["session_id"] != sid
        self.last_service_result = record
        if not accepted:
            raise RuntimeError("Vehicle service request rejected")
        # Sent requests cannot be recalled: acknowledge their actual outcome,
        # while the main publisher and supervisor enforce the latest phase.
        self.service_error = ""

    def _publish_uav(self, target):
        if not self.hardware_allowed:
            return
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.robot.get("setpoint_frame", "map")
        msg.coordinate_frame, msg.type_mask = target["coordinate_frame"], target["type_mask"]
        for field in ("position", "velocity", "acceleration_or_force"):
            vector = getattr(msg, field)
            vector.x, vector.y, vector.z = target[field]
        msg.yaw, msg.yaw_rate = target["yaw"], target["yaw_rate"]
        self.command_pub.publish(msg)
        if self.first_setpoint is None:
            self.first_setpoint = time.monotonic()

    def _publish_ugv(self, world_velocity):
        command, applied, clipped = ugv_command(world_velocity, self.yaw,
                                                self.robot.get("cmd_frame", "body"), self.limits["ugv_speed"])
        if self.hardware_allowed:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.linear.z = command
            self.command_pub.publish(msg)
        return applied, clipped, command

    def tick(self, event):
        with self.lock:
            try:
                self._tick()
            except Exception as exc:
                self.fault = "control exception: "+str(exc)
                rospy.logerr_throttle(1.0, "%s: %s", self.agent_id, self.fault)
                if self.is_uav and self.position is not None:
                    self._publish_uav(velocity_target([0,0,0],self.robot.get("yaw",0.0),self.setpoint_transform))
                elif not self.is_uav:
                    self._publish_ugv([0,0,0])
                diagnostic = {"version":1,"id":self.agent_id,"group":self.group,
                    "role":"follower" if self.is_uav else "leader","phase":"FAULT",
                    "session_id":self.session["session_id"],"t0":self.session["t0"],"stamp":rospy.Time.now().to_sec(),
                    "healthy":False,"ready":False,"reason":self.fault,"fault_reason":self.fault,
                    "position":None if self.position is None else self.position.tolist(),
                    "velocity":None if self.velocity is None else self.velocity.tolist(),
                    "input_raw":[0.,0.,0.],"input_applied":[0.,0.,0.],"limited":False,
                    "controller":{},"start_offset_s":self.start_offset,"reference_tau":self.reference_tau}
                self.telemetry_pub.publish(String(data=json.dumps(diagnostic,allow_nan=False)))

    def _tick(self):
        now, wall = rospy.Time.now().to_sec(), time.monotonic()
        dt = 1.0/self.runtime["rate_hz"] if self.last_tick is None else now-self.last_tick
        phase = self.session["phase"]
        if self.last_tick is not None and (dt < 0 or (phase == "RUNNING" and dt > self.runtime["max_dt"])):
            self.fault = "ROS clock moved backwards or control deadline missed"
        self.last_tick = now
        healthy, reason = self._sensors_healthy(now, wall)
        if phase not in ("IDLE", "LANDED") and wall-self.session_receipt > self.runtime["session_timeout"]:
            self.fault = "supervisor heartbeat timeout"
        active = phase == "RUNNING" and now >= self.session["t0"]
        if active and not healthy:
            self.fault = reason
        if active and healthy and self.is_uav and np.linalg.norm(self.velocity[:2]) > self.limits["uav_speed"]:
            self.fault = "UAV horizontal speed limit exceeded"
        if self.is_uav and phase in ("ARMED", "TAKING_OFF", "AIRBORNE", "POSITIONING", "POSITIONED", "RECORDING", "SCHEDULED", "RUNNING"):
            if not self.fcu["armed"] or self.fcu["mode"] != "OFFBOARD":
                self.fault = "FCU unexpectedly disarmed or left OFFBOARD"
        if not self.controller_initialized and self.position is not None:
            self._reset_controller()
        neighbor_values, neighbor_ready = self._signals(now,wall)
        if active and not neighbor_ready:
            self.fault = "neighbor signal missing, stale, or from another session"
        if active and self.start_offset is None and now-self.session["t0"] > self.runtime.get("start_tolerance",self.runtime["max_dt"]):
            self.fault = "late physical control start"
        execute = healthy and not self.fault and self.gate.may_run(phase,now) and neighbor_ready
        controller, ready, target = {}, False, None
        raw = np.zeros(3)
        applied = np.zeros(3)
        limited = False
        mode = "inhibited"
        yaw = float(self.robot.get("yaw",0.0))
        if self.position is not None and self.controller_initialized:
            if execute:
                self.reference_tau = max(0.0,now-self.session["t0"])
            tau = self.reference_tau
            if execute:
                if self.start_offset is None:
                    self.start_offset = now-self.session["t0"]
                    self.last_control_time = now
                else:
                    elapsed = now-self.last_control_time
                    if elapsed <= 0 or elapsed > self.runtime["max_dt"]:
                        self.fault = "invalid controller integration interval"
                        execute = False
                    else:
                        self.last_control_time = now
                        if self.is_uav:
                            controller = self.controller.step(elapsed,tau,self.position[:2],self.velocity[:2],neighbor_values)
                        else:
                            controller = self.controller.step(elapsed,self.position[:2],neighbor_values)
            if not controller:
                if self.is_uav:
                    controller = self.controller.output(tau,self.position[:2],
                        self.velocity[:2] if self.velocity is not None else np.zeros(2),neighbor_values)
                else:
                    controller = self.controller.output(self.position[:2],neighbor_values)
            signal = {"version":1,"id":self.agent_id,"group":self.group,"session_id":self.session["session_id"],
                      "t0":self.session["t0"],"stamp":now,"w":controller["w"]}
            if not self.is_uav:
                signal.update(y=controller["y"],z=controller["z"])
            self.signal_pub.publish(String(data=json.dumps(signal,default=jsonable,allow_nan=False)))
        recovery = phase in ("STOPPING","STOPPED","LANDING","LANDED")
        effective = "FAULT" if self.fault and not recovery else phase
        stopped = self.velocity is not None and np.linalg.norm(self.velocity) < self.runtime["velocity_tolerance"]
        group = self.config["groups"][self.group]
        if self.is_uav and self.position is not None:
            if execute and not self.fault:
                raw[:2] = controller["u"]
                xy, cxy = limit_norm(raw[:2],self.limits["uav_acceleration"])
                az, cz = height_acceleration(self.position[2],self.velocity[2],group["altitude"],
                    self.limits["altitude_kp"],self.limits["altitude_kd"],self.limits["z_acceleration"])
                raw[2] = self.limits["altitude_kp"]*(group["altitude"]-self.position[2])-self.limits["altitude_kd"]*self.velocity[2]
                applied = np.r_[xy,az]
                limited = cxy or cz
                target = acceleration_target(applied,yaw,self.setpoint_transform)
                mode = "acceleration"
                ready = True
            if target is None:
                target, ready, mode = self._uav_nonrunning(effective,healthy,stopped,dt,now,wall,yaw)
            if self.fault and not recovery:
                target, ready, mode = self._uav_nonrunning("FAULT",healthy,stopped,dt,now,wall,yaw)
            if mode != "acceleration":
                applied = np.zeros(3)
            self._publish_uav(target)
        elif not self.is_uav:
            if execute and not self.fault:
                raw[:2] = controller["u"]
                ready = True
            elif effective == "POSITIONING" and healthy:
                raw[:2] = self.runtime["goto_kp"]*(np.asarray(group["initial_xy"])-self.position[:2])
                ready = self._ready_with_dwell(np.linalg.norm(np.asarray(group["initial_xy"])-self.position[:2]) < self.runtime["position_tolerance"] and stopped,now)
            elif effective in ("PREPARING","READY","ARMING","ARMED","TAKING_OFF","AIRBORNE","RECORDING","POSITIONED"):
                ready = healthy
            elif effective == "SCHEDULED":
                ready = healthy and neighbor_ready and self.controller_initialized and self.gate.prepared
            elif effective == "RUNNING" and not active:
                ready = healthy and neighbor_ready and self.gate.committed
            elif effective in ("STOPPING","STOPPED","LANDING","LANDED"):
                ready = healthy and self._ready_with_dwell(stopped,now)
            applied, limited, body = self._publish_ugv(raw)
            mode = "velocity"
            target = {"kind":"velocity","frame":self.robot.get("cmd_frame","body"),"velocity":body.tolist()}
        if not self.hardware_allowed:
            ready = False
            reason = "hardware outputs inhibited; use verified hardware settings or mock launch"
        if not healthy:
            ready = False
        if self.fault and not recovery:
            ready, healthy, reason = False, False, self.fault
        elif self.fault:
            reason = self.fault
        telemetry = {"version":1,"id":self.agent_id,"group":self.group,"role":"follower" if self.is_uav else "leader",
            "phase":"FAULT" if self.fault and not recovery else phase,"fault_reason":self.fault,"session_id":self.session["session_id"],"t0":self.session["t0"],"stamp":now,"tau":max(0.0,now-self.session["t0"]),"reference_tau":self.reference_tau,
            "healthy":healthy,"ready":bool(ready),"reason":reason,"position":None if self.position is None else self.position.tolist(),
            "velocity":None if self.velocity is None else self.velocity.tolist(),
            "acceleration_estimate":self.acceleration.tolist() if healthy and self.acceleration is not None else None,
            "acceleration_valid":bool(healthy and self.acceleration is not None),
            "imu_specific_force_body":self.imu_force.tolist() if self.imu_force is not None and wall-self.imu_receipt <= self.runtime["state_timeout"] else None,
            "imu_stamp":self.imu_stamp,"pose_stamp":self.pose_stamp,"velocity_stamp":self.velocity_stamp,
            "velocity_timestamp_origin":self.velocity_timestamp_origin,"pose_age":now-self.pose_stamp,"velocity_age":now-self.velocity_stamp,
            "velocity_header_frame":self.velocity_header_frame,
            "input_raw":raw.tolist(),"input_applied":applied.tolist(),"limited":bool(limited),"controller":controller,
            "start_offset_s":self.start_offset,"command_mode":mode,"setpoint":target,"fcu":self.fcu.copy(),
            "landed_state":self.landed_state,"dt":dt,"service_error":self.service_error,
            "last_service_result":self.last_service_result,"outputs_enabled":self.hardware_allowed}
        self.telemetry_pub.publish(String(data=json.dumps(telemetry,default=jsonable,allow_nan=False)))

    def _uav_nonrunning(self, phase, healthy, stopped, dt, now, wall, yaw):
        hold = position_target(self.position if self.hold_position is None else self.hold_position,
                               yaw,self.setpoint_transform)
        if phase == "LANDING":
            landed = self.landed_state == 1 and wall-self.landed_receipt <= self.runtime.get("fcu_state_timeout",3.0)
            if not landed and self.fcu["connected"] and self.fcu["mode"] != "AUTO.LAND":
                self.request_service(mode="AUTO.LAND")
            return velocity_target([0,0,0],yaw,self.setpoint_transform),landed and not self.fcu["armed"],"autoland"
        if phase in ("FAULT","STOPPING","STOPPED") or not healthy:
            stable = healthy and self._ready_with_dwell(stopped,now)
            if stable and self.stop_hold is None:
                self.stop_hold = self.position.copy()
            if self.stop_hold is not None:
                return position_target(self.stop_hold,yaw,self.setpoint_transform), stable and phase != "FAULT", "hold"
            return velocity_target([0,0,0],yaw,self.setpoint_transform), False, "brake_velocity"
        if phase in ("ARMING","ARMED"):
            warm = self.first_setpoint is not None and wall-self.first_setpoint >= self.runtime["offboard_warmup"]
            if phase == "ARMING" and warm:
                if self.fcu["mode"] != "OFFBOARD":
                    self.request_service(mode="OFFBOARD")
                elif not self.fcu["armed"]:
                    self.request_service(armed=True)
            return hold, self.fcu["armed"] and self.fcu["mode"] == "OFFBOARD", "ground_hold"
        if phase in ("TAKING_OFF","AIRBORNE"):
            if self.takeoff_xy is None:
                self.takeoff_xy, self.takeoff_z = self.position[:2].copy(),float(self.position[2])
            target_z = float(self.config["groups"][self.group]["altitude"])
            self.takeoff_z = min(target_z,self.takeoff_z+self.runtime["takeoff_speed"]*max(0.0,dt))
            point = np.r_[self.takeoff_xy,self.takeoff_z]
            ready = self._ready_with_dwell(abs(self.position[2]-target_z) < self.runtime["position_tolerance"] and stopped,now)
            return position_target(point,yaw,self.setpoint_transform),ready,"takeoff_position"
        if phase in ("POSITIONING","POSITIONED"):
            point = self.initial_uav_point()
            ready = self._ready_with_dwell(np.linalg.norm(point-self.position) < self.runtime["position_tolerance"] and stopped,now)
            return position_target(point,yaw,self.setpoint_transform),ready,"initial_position"
        if phase == "LANDED":
            return hold,self.landed_state == 1 and not self.fcu["armed"],"landed"
        if phase == "SCHEDULED":
            _,neighbor_ready = self._signals(now,wall)
            return hold,neighbor_ready and self.gate.prepared,"scheduled_hold"
        if phase == "RUNNING":
            _,neighbor_ready = self._signals(now,wall)
            return hold,neighbor_ready and self.gate.committed,"prestart_hold"
        return hold,healthy,"hold"

    def shutdown(self):
        try:
            if self.hardware_allowed:
                if self.is_uav and self.position is not None:
                    self._publish_uav(velocity_target([0,0,0],self.robot.get("yaw",0.0),self.setpoint_transform))
                elif not self.is_uav:
                    self._publish_ugv([0,0,0])
        except Exception:
            pass


def main():
    rospy.init_node("ne_tvft_agent")
    AgentNode()
    rospy.spin()
