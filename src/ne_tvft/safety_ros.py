"""Fresh direct group-state subscriptions and the agent's sole output filter."""
import math
import time
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from .adapters import (PlanarTransform, quaternion_yaw, rotation, sample_is_fresh,
                       acceleration_target, velocity_target)
from .safety import GroupSafety, SafetyError, challenge_acceleration
from .core import circle


class AgentSafety:
    def __init__(self, agent):
        self.a = agent
        self.model = GroupSafety(agent.config)
        self.enabled = self.model.enabled and agent.is_uav
        self.states, self.snapshot = {}, None
        self.diagnostic = {"enabled": self.enabled, "healthy": not self.enabled, "active": False}
        self.keep_acceleration_until = -math.inf
        self.challenge = agent.config.get("simulation", {}).get("safety_challenge", {})
        if self.challenge.get("enabled", False) and agent.backend != "sss":
            raise SafetyError("The artificial safety challenge is allowed ONLY on the SSS backend")
        if not self.enabled:
            return
        group = agent.config["groups"][agent.group]
        self.peer = group["followers"][1-agent.follower]
        self.leader = group["leader"]
        for rid in [self.peer, self.leader]:
            robot = agent.config["robots"][rid]
            if robot.get("pose_type", "PoseStamped") != "PoseStamped":
                raise SafetyError("Group safety requires stamped poses")
            self.states[rid] = {"error": "waiting for group safety state"}
            agent.subscribers.append(rospy.Subscriber(agent.external(robot["pose_topic"]), PoseStamped,
                lambda msg, r=rid: self.receive(r, msg, True), queue_size=1))
            vtype = robot.get("velocity_type", "TwistStamped")
            if vtype != "TwistStamped":
                raise SafetyError("Group safety requires source-stamped velocities")
            agent.subscribers.append(rospy.Subscriber(agent.external(robot["velocity_topic"]), TwistStamped,
                lambda msg, r=rid: self.receive(r, msg, False), queue_size=1))

    def receive(self, rid, msg, pose):
        with self.a.lock:
            state, robot = self.states[rid], self.a.config["robots"][rid]
            key = "pose" if pose else "velocity"
            try:
                stamp = msg.header.stamp.to_sec()
                if stamp <= 0 or stamp <= state.get(key+"_stamp", 0.):
                    raise SafetyError("non-increasing group "+key+" stamp")
                if pose:
                    if robot.get("pose_frame", "") and msg.header.frame_id != robot["pose_frame"]:
                        raise SafetyError("group pose frame mismatch")
                    tf = PlanarTransform(robot.get("transform"))
                    v = msg.pose.position
                    state["position"] = tf.point_to_world([v.x, v.y, v.z])
                    state["yaw"] = quaternion_yaw([msg.pose.orientation.x, msg.pose.orientation.y,
                                                    msg.pose.orientation.z, msg.pose.orientation.w])+tf.yaw
                else:
                    tf = PlanarTransform(robot.get("velocity_transform", robot.get("transform")))
                    v = msg.twist.linear
                    value = np.array([v.x, v.y, v.z])
                    frame = robot.get("velocity_frame", "world")
                    if frame == "body":
                        if "yaw" not in state:
                            raise SafetyError("group body velocity without pose")
                        value = rotation(state["yaw"])@value
                    elif frame == "world":
                        value = tf.vector_to_world(value)
                    else:
                        raise SafetyError("unsupported group velocity frame")
                    state["velocity"] = value
                state[key+"_stamp"], state[key+"_receipt"] = stamp, time.monotonic()
                state[key+"_error"] = ""
            except (ValueError, TypeError) as exc:
                state[key+"_error"] = str(exc)

    def check(self, now, wall):
        if not self.enabled:
            return True, ""
        a, states = self.a, {}
        for rid, source in [(a.agent_id, {"position": a.position, "velocity": a.velocity,
                "pose_stamp": a.pose_stamp, "velocity_stamp": a.velocity_stamp,
                "pose_receipt": a.pose_receipt, "velocity_receipt": a.velocity_receipt})]+list(self.states.items()):
            for key in ["pose", "velocity"]:
                if source.get(key+"_error") or not sample_is_fresh(now,
                        wall-source.get(key+"_receipt", -math.inf), source.get(key+"_stamp", 0.), self.model.max_age):
                    self.snapshot = None
                    self.diagnostic = {"enabled": True, "healthy": False, "active": False,
                                       "reason": "stale/invalid safety "+key+": "+rid}
                    return False, self.diagnostic["reason"]
            p, v = source.get("position"), source.get("velocity")
            if p is None or v is None or not np.isfinite(np.r_[p, v]).all():
                return False, "invalid safety vector: "+rid
            if abs(source["pose_stamp"]-source["velocity_stamp"]) > self.model.max_age:
                return False, "unaligned safety position/velocity: "+rid
            # Short bounded source-age prediction; never interpolate delayed
            # algorithm signals or advance an unboundedly old observation.
            states[rid] = (p+v*max(0., now-source["pose_stamp"]), v.copy())
        self.snapshot = states
        geo = self.model.geometry(states[a.agent_id][0], states[self.peer][0], states[self.leader][0])
        self.diagnostic = dict(geo, enabled=True, healthy=True, active=False)
        if a.session["phase"] in ("IDLE", "PREPARING", "READY", "ARMING", "ARMED"):
            if geo["uav_barrier_h"] < 0 or geo["ugv_barrier_h"] < 0:
                return False, "initial group geometry violates safety envelope"
        return True, ""

    def apply(self, target, now, healthy):
        a, phase = self.a, self.a.session["phase"]
        if not a.is_uav or (not self.enabled and not self.challenge.get("enabled", False)):
            return target, False, None
        if phase in ("IDLE", "PREPARING", "READY", "ARMING", "ARMED", "LANDED"):
            return target, False, None
        if self.enabled and (not healthy or self.snapshot is None):
            a.fault = a.fault or self.diagnostic.get("reason", "safety feedback unavailable")
            self.diagnostic.update(healthy=False, reason=a.fault)
            return velocity_target([0, 0, 0], a.robot.get("yaw", 0.), a.setpoint_transform), True, None
        # Keep the proven PX4 position-mode takeoff/landing path. Switching to
        # acceleration while still on the ground changes PX4's takeoff logic.
        # Non-running position references are guarded before forwarding, rather
        # than silently replacing their flight-control interface.
        if self.enabled and target["kind"] == "position" and phase != "RUNNING":
            try:
                self.guard_reference(target)
                return target, False, None
            except SafetyError as exc:
                a.fault = "safety reference: "+str(exc)
                self.diagnostic.update(healthy=False, reason=a.fault)
                return velocity_target([0, 0, 0], a.robot.get("yaw", 0.), a.setpoint_transform), True, None
        if self.enabled and phase != "RUNNING" and a.position[2] < .6:
            return target, False, None
        tf, p, v = a.setpoint_transform, a.position, a.velocity
        kind = target["kind"]
        kp, kd = float(self.model.s.get("position_kp", 1.0)), float(self.model.s.get("velocity_kp", 2.0))
        if kind == "acceleration":
            nominal = tf.vector_to_world(target["acceleration_or_force"])
        elif kind == "velocity":
            nominal = kd*(tf.vector_to_world(target["velocity"])-v)
        else:
            nominal = kp*(tf.point_to_world(target["position"])-p)-kd*v
        zcap = a.limits["z_acceleration"]
        nominal[2] = np.clip(nominal[2], -zcap, zcap)
        challenge_active = False
        tau = now-a.session["t0"]
        if phase == "RUNNING":
            challenge_input = challenge_acceleration(a.config, a.group, a.follower, tau, v)
            challenge_active = challenge_input is not None
            if challenge_active:
                nominal[:2] = challenge_input
        if not self.enabled:
            self.diagnostic = {"enabled": False, "healthy": True, "active": False,
                               "challenge_active": challenge_active}
            if challenge_active:
                return acceleration_target(nominal, a.robot.get("yaw", 0.), tf), True, nominal
            return target, False, None
        try:
            own, peer, leader = [self.snapshot[rid] for rid in [a.agent_id, self.peer, self.leader]]
            stationary = phase != "RUNNING" and np.linalg.norm(leader[1]) < .03
            output, self.diagnostic = self.model.filter(nominal[:2], own[0], own[1], peer[0], peer[1],
                leader[0], leader[1], nominal[2], stationary)
            self.diagnostic["challenge_active"] = challenge_active
            if self.diagnostic["active"]:
                self.keep_acceleration_until = now+.3
            use_acceleration = kind == "acceleration" or challenge_active or now < self.keep_acceleration_until
            if use_acceleration:
                applied = np.r_[output, nominal[2]]
                return acceleration_target(applied, a.robot.get("yaw", 0.), tf), True, applied
            return target, False, None
        except SafetyError as exc:
            a.fault = "safety filter: "+str(exc)
            self.diagnostic.update(healthy=False, active=False, reason=a.fault)
            return velocity_target([0, 0, 0], a.robot.get("yaw", 0.), tf), True, None

    def guard_reference(self, target):
        """Reject intersecting position transfers; no claim of a full planner."""
        a = self.a
        own, peer, leader = [self.snapshot[rid] for rid in [a.agent_id, self.peer, self.leader]]
        goal = a.setpoint_transform.point_to_world(target["position"])
        peer_goal = peer[0].copy()
        if a.session["phase"] in ("POSITIONING", "POSITIONED"):
            group = a.config["groups"][a.group]
            peer_goal[:2] = np.asarray(group["initial_xy"])+circle(a.config, a.group, 1-a.follower, 0.)[0]
        self.model.guard_transfer(own[0], peer[0], leader[0], goal, peer_goal)
        if a.session["phase"] == "LANDING" and np.linalg.norm(leader[1]) > .03:
            raise SafetyError("vehicle must stop before the UAV descends")

    def landing_corridor(self, reference_xy):
        """Move outward at altitude before descending beside the vehicle."""
        leader = self.snapshot[self.leader][0][:2]
        delta = np.asarray(reference_xy)-leader
        radius = np.linalg.norm(delta)
        if radius < 1e-6:
            delta = circle(self.a.config, self.a.group, self.a.follower, self.a.reference_tau)[0]
            radius = np.linalg.norm(delta)
        minimum = self.model.rx+float(self.model.s.get("landing_clearance", .05))
        target = leader+delta*max(1., minimum/radius)
        ready = (np.linalg.norm(self.a.position[:2]-target) < .02 and
                 np.linalg.norm(self.a.position[:2]-leader) >= self.model.rx+.025)
        return target, ready
