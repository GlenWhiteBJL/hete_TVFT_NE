import math
from pathlib import Path
import sys
import unittest
import importlib
import json
import types
from unittest.mock import patch
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from ne_tvft.adapters import (PlanarTransform, ugv_command, position_target,
    velocity_target, acceleration_target, height_acceleration, sample_is_fresh,
    DerivativeEstimate, StartGate, quaternion_yaw, finite_vector,
    POSITION_MASK, VELOCITY_MASK, ACCELERATION_MASK)


class AdapterTests(unittest.TestCase):
    def test_frame_roundtrip_and_translation_not_applied_to_vectors(self):
        tf = PlanarTransform({"yaw":math.pi/2,"translation":[2,3,1]})
        np.testing.assert_allclose(tf.point_to_world([1,0,0]),[2,4,1],atol=1e-14)
        np.testing.assert_allclose(tf.point_from_world([2,4,1]),[1,0,0],atol=1e-14)
        np.testing.assert_allclose(tf.vector_from_world([0,1,0]),[1,0,0],atol=1e-14)

    def test_ugv_world_velocity_rotated_to_body(self):
        body,applied,limited = ugv_command([0,.2,0],math.pi/2,"body",.3)
        np.testing.assert_allclose(body,[.2,0,0],atol=1e-14)
        np.testing.assert_allclose(applied,[0,.2,0])
        self.assertFalse(limited)
        _,applied,limited = ugv_command([3,4,0],0,"world",.5)
        np.testing.assert_allclose(applied,[.3,.4,0])
        self.assertTrue(limited)

    def test_ros_enu_payload_no_extra_ned_swap(self):
        tf = PlanarTransform()
        target = acceleration_target([.1,.2,.3],.5,tf)
        np.testing.assert_allclose(target["acceleration_or_force"],[.1,.2,.3])
        self.assertEqual(target["coordinate_frame"],1)
        self.assertEqual(target["type_mask"],2111)
        self.assertFalse(target["type_mask"] & 512)  # acceleration, not FORCE
        self.assertFalse(target["type_mask"] & (64|128|256))
        self.assertTrue(target["type_mask"] & 2048)  # yaw-rate ignored
        self.assertFalse(target["type_mask"] & 1024)  # yaw retained

    def test_position_velocity_and_acceleration_masks_are_disjoint(self):
        self.assertEqual(POSITION_MASK,2552)
        self.assertEqual(VELOCITY_MASK,2503)
        self.assertEqual(ACCELERATION_MASK,2111)
        tf=PlanarTransform({"yaw":math.pi/2,"translation":[5,2,0]})
        p=position_target([5,3,1],math.pi/2,tf)
        v=velocity_target([0,.3,0],math.pi/2,tf)
        np.testing.assert_allclose(p["position"],[1,0,1],atol=1e-14)
        np.testing.assert_allclose(v["velocity"],[.3,0,0],atol=1e-14)
        self.assertAlmostEqual(p["yaw"],0)

    def test_brake_is_a_velocity_target(self):
        stop=velocity_target([0,0,0],0,PlanarTransform())
        self.assertEqual(stop["kind"],"velocity")
        self.assertTrue(stop["type_mask"] & (64|128|256))
        self.assertFalse(stop["type_mask"] & (8|16|32))

    def test_height_pd_clips_and_damps_existing_velocity(self):
        self.assertEqual(height_acceleration(1,0,2,1,1.5,.6),(.6,True))
        value,clipped=height_acceleration(1,.2,1,1,1.5,.6)
        self.assertAlmostEqual(value,-.3)
        self.assertFalse(clipped)

    def test_source_and_receipt_freshness_both_required(self):
        self.assertTrue(sample_is_fresh(10,.02,9.95,.3))
        self.assertFalse(sample_is_fresh(10,.02,9.0,.3))
        self.assertFalse(sample_is_fresh(10,1.0,9.95,.3))
        self.assertFalse(sample_is_fresh(10,.02,11,.3))
        self.assertFalse(sample_is_fresh(10,.02,0,.3))

    def test_derivative_does_not_invent_acceleration(self):
        d=DerivativeEstimate(.3)
        self.assertIsNone(d.update([0,0,0],1.0))
        np.testing.assert_allclose(d.update([.02,.04,0],1.02),[1,2,0],atol=1e-12)
        self.assertIsNone(d.update([.03,.04,0],1.01))
        self.assertIsNone(d.update([.04,.05,0],2.0))

    def test_schedule_alone_never_runs_even_after_t0(self):
        g=StartGate()
        self.assertTrue(g.prepare("abc",12,10))
        self.assertFalse(g.may_run("SCHEDULED",13))
        self.assertFalse(g.may_run("RUNNING",13))

    def test_two_phase_commit_runs_only_at_t0(self):
        g=StartGate()
        g.prepare("abc",12,10)
        g.commit("abc",12,11)
        self.assertFalse(g.may_run("RUNNING",11.99))
        self.assertTrue(g.may_run("RUNNING",12))
        self.assertFalse(g.may_run("STOPPING",13))
        # Heartbeat after start is idempotent.
        g.commit("abc",12,13)
        self.assertTrue(g.may_run("RUNNING",13))

    def test_late_or_mismatched_commit_is_rejected(self):
        g=StartGate()
        with self.assertRaises(ValueError):
            g.commit("abc",12,10)
        g.prepare("abc",12,10)
        with self.assertRaises(ValueError):
            g.commit("other",12,11)
        with self.assertRaises(ValueError):
            g.commit("abc",12.1,11)
        with self.assertRaises(ValueError):
            g.commit("abc",12,12.01)

    def test_duplicate_schedule_does_not_reset_and_cannot_change_t0(self):
        g=StartGate()
        g.prepare("abc",12,10)
        self.assertFalse(g.prepare("abc",12,10.1))
        with self.assertRaises(ValueError):
            g.prepare("abc",13,10.2)

    def test_invalid_numeric_inputs_rejected(self):
        with self.assertRaises(ValueError):
            finite_vector([math.nan,0,0])
        with self.assertRaises(ValueError):
            quaternion_yaw([0,0,0,0])
        with self.assertRaises(ValueError):
            ugv_command([1,0,0],0,"unspecified",.2)


class FakeStamp:
    def __init__(self,value=0.0):
        self.value=float(value)
    def to_sec(self):
        return self.value


class FakeString:
    def __init__(self,data=""):
        self.data=data


def vector():
    return types.SimpleNamespace(x=0.0,y=0.0,z=0.0)


class FakeTarget:
    def __init__(self):
        self.header=types.SimpleNamespace(stamp=FakeStamp(),frame_id="")
        self.position,self.velocity,self.acceleration_or_force=vector(),vector(),vector()


class FakeTwist:
    def __init__(self):
        self.linear,self.angular=vector(),vector()


class FakePublisher:
    def __init__(self,name,*args,**kwargs):
        self.name,self.messages=name,[]
    def publish(self,message):
        self.messages.append(message)


def fake_ros(config,agent_id="uav1"):
    ros=types.ModuleType("rospy")
    ros.clock=100.0
    params={"/ne_tvft/config":config,"~agent_id":agent_id,"~mock":True,"~topics_file":""}
    ros.has_param=lambda key:key in params
    ros.get_param=lambda key,default=None:params.get(key,default)
    ros.Time=types.SimpleNamespace(now=lambda:FakeStamp(ros.clock))
    ros.Duration=lambda seconds:seconds
    ros.Publisher=FakePublisher
    ros.Subscriber=lambda *args,**kwargs:object()
    ros.Timer=lambda *args,**kwargs:object()
    ros.on_shutdown=lambda callback:None
    ros.is_shutdown=lambda:True  # worker exits immediately; service behavior tested independently
    ros.logerr_throttle=ros.logwarn_throttle=lambda *args,**kwargs:None
    return ros


def ros_stubs(ros):
    modules={"rospy":ros}
    messages={"std_msgs.msg":{"String":FakeString},
              "geometry_msgs.msg":{"PoseStamped":type("PoseStamped",(),{}),"TwistStamped":type("TwistStamped",(),{}),"Twist":FakeTwist},
              "sensor_msgs.msg":{"Imu":type("Imu",(),{})},
              "mavros_msgs.msg":{"State":type("State",(),{}),"ExtendedState":type("ExtendedState",(),{}),"PositionTarget":FakeTarget},
              "mavros_msgs.srv":{"CommandBool":type("CommandBool",(),{}),"SetMode":type("SetMode",(),{})}}
    for name,values in messages.items():
        modules.setdefault(name.split('.')[0],types.ModuleType(name.split('.')[0]))
        module=types.ModuleType(name)
        for key,value in values.items():
            setattr(module,key,value)
        modules[name]=module
    return modules


class AgentNodeTests(unittest.TestCase):
    def setUp(self):
        root=Path(__file__).resolve().parents[1]
        self.config=yaml.safe_load((root/"config/experiment.yaml").read_text(encoding="utf-8"))
        self.config.update(yaml.safe_load((root/"config/topics.yaml").read_text(encoding="utf-8")))
        self.ros=fake_ros(self.config)
        self.module_patch=patch.dict(sys.modules,ros_stubs(self.ros))
        self.module_patch.start()
        self.module=importlib.import_module("ne_tvft.agent_node")
        self.module.rospy=self.ros
        self.node=self.module.AgentNode()

    def tearDown(self):
        self.module_patch.stop()

    def sensors(self,position=None,velocity=None):
        n=self.node
        if position is None:
            position=n.initial_uav_point()
        if velocity is None:
            velocity=[0,0,0]
        p,q=vector(),types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.)
        p.x,p.y,p.z=position
        n.pose_callback(types.SimpleNamespace(header=types.SimpleNamespace(stamp=FakeStamp(self.ros.clock),frame_id=n.robot["pose_frame"]),
                         pose=types.SimpleNamespace(position=p,orientation=q)))
        v=FakeTwist()
        v.linear.x,v.linear.y,v.linear.z=velocity
        n.velocity_callback(types.SimpleNamespace(header=types.SimpleNamespace(stamp=FakeStamp(self.ros.clock),frame_id="map"),twist=v))
        n.state_callback(types.SimpleNamespace(connected=True,armed=True,mode="OFFBOARD"))
        n.landed_callback(types.SimpleNamespace(landed_state=2))

    def session(self,phase,t0=102,session_id="test"):
        self.node.session_callback(FakeString(json.dumps({"version":1,"phase":phase,"t0":t0,"session_id":session_id})))

    def neighbors(self):
        n=self.node
        for aid in n.expected_ids:
            n.signal_callback(aid,FakeString(json.dumps({"version":1,"id":aid,"session_id":n.session["session_id"],
                "stamp":self.ros.clock,"t0":n.session["t0"],"w":[0.,0.]})))

    def telemetry(self):
        return json.loads(self.node.telemetry_pub.messages[-1].data)

    def test_node_initializes_and_never_emits_uav_target_without_pose(self):
        self.node._tick()
        self.assertEqual(len(self.node.command_pub.messages),0)
        self.assertFalse(self.telemetry()["healthy"])
        self.node.shutdown()
        self.assertEqual(len(self.node.command_pub.messages),0)

    def test_schedule_ack_requires_own_state_and_all_local_neighbors(self):
        self.sensors()
        self.session("SCHEDULED")
        self.node._tick()
        self.assertFalse(self.telemetry()["ready"])
        self.neighbors()
        self.node._tick()
        self.assertTrue(self.telemetry()["ready"])
        self.assertEqual(self.telemetry()["command_mode"],"scheduled_hold")
        self.assertEqual(self.telemetry()["t0"],102)
        self.assertEqual(self.node.controller.beta,self.config["gains"]["beta_initial"])

    def test_future_commit_holds_initial_point_then_starts_at_t0(self):
        self.sensors()
        self.session("POSITIONED")
        self.session("SCHEDULED",t0=100.1)
        self.neighbors()
        self.session("RUNNING",t0=100.1)
        expected=self.node.initial_uav_point()
        self.ros.clock=100.02
        self.sensors(expected+np.array([.02,0,0]))
        self.node._tick()
        np.testing.assert_allclose(self.telemetry()["setpoint"]["position"],expected)
        self.assertIsNone(self.telemetry()["start_offset_s"])
        self.assertEqual(self.telemetry()["t0"],100.1)
        self.ros.clock=100.06
        self.sensors(expected)
        self.neighbors()
        self.node._tick()
        self.ros.clock=100.1
        self.sensors(expected)
        self.neighbors()
        self.node._tick()
        self.assertEqual(self.telemetry()["command_mode"],"acceleration")
        self.assertAlmostEqual(self.telemetry()["start_offset_s"],0)

    def test_late_running_commit_latches_fault_and_freezes_adaptation(self):
        self.sensors()
        self.session("SCHEDULED",t0=100.1)
        self.ros.clock=100.11
        self.session("RUNNING",t0=100.1)
        self.sensors()
        self.node._tick()
        self.assertEqual(self.telemetry()["phase"],"FAULT")
        self.assertEqual(self.node.controller.beta,self.config["gains"]["beta_initial"])
        self.assertEqual(self.telemetry()["command_mode"],"brake_velocity")

    def test_stale_neighbor_brakes_and_preserves_gain(self):
        self.sensors()
        self.session("SCHEDULED",t0=100.1)
        self.neighbors()
        self.session("RUNNING",t0=100.1)
        self.ros.clock=100.1
        self.sensors()
        self.neighbors()
        self.node._tick()
        initial=self.node.controller.beta
        # Remove one required input, without causing a sensor/control-time timeout.
        self.node.neighbors.pop(next(iter(self.node.expected_ids)))
        self.ros.clock=100.12
        self.sensors()
        self.node._tick()
        self.assertEqual(self.telemetry()["phase"],"FAULT")
        self.assertEqual(self.node.controller.beta,initial)
        self.assertEqual(self.telemetry()["command_mode"],"brake_velocity")

    def test_clock_rollback_faults(self):
        self.sensors()
        self.node._tick()
        self.ros.clock=99.9
        self.node._tick()
        self.assertIn("clock",self.telemetry()["fault_reason"])

    def test_velocity_transform_is_independent_of_pose_transform(self):
        self.node.velocity_transform=PlanarTransform({"yaw":math.pi/2})
        self.sensors(velocity=[.1,0,0])
        np.testing.assert_allclose(self.node.velocity,[0,.1,0],atol=1e-14)
        self.assertEqual(self.telemetry()["velocity_header_frame"] if self.node.telemetry_pub.messages else self.node.velocity_header_frame,"map")

    def test_fault_allows_explicit_landing_request_but_cannot_restart_control(self):
        self.sensors()
        self.node.fault="test fault"
        requested=[]
        self.node.request_service=lambda **kwargs:requested.append(kwargs)
        self.session("LANDING")
        self.node._tick()
        self.assertIn({"mode":"AUTO.LAND"},requested)
        self.assertEqual(self.telemetry()["phase"],"LANDING")
        self.assertEqual(self.telemetry()["fault_reason"],"test fault")
        self.assertEqual(self.telemetry()["command_mode"],"autoland")

    def test_stop_freezes_reference_and_brakes_before_position_hold(self):
        self.sensors()
        self.session("SCHEDULED",t0=100.1)
        self.neighbors()
        self.session("RUNNING",t0=100.1)
        self.ros.clock=100.1
        self.sensors()
        self.neighbors()
        self.node._tick()
        self.ros.clock=100.12
        self.sensors()
        self.neighbors()
        self.node._tick()
        before=self.telemetry()
        self.session("STOPPING",t0=100.1)
        self.ros.clock=100.14
        self.sensors(velocity=[.2,0,0])
        self.node._tick()
        current=self.telemetry()
        self.assertEqual(current["command_mode"],"brake_velocity")
        self.assertEqual(current["reference_tau"],before["reference_tau"])
        np.testing.assert_allclose(current["controller"]["h"],before["controller"]["h"])
        self.assertEqual(current["controller"]["beta"],before["controller"]["beta"])
        # Stable-velocity dwell completes before capturing the hold point.
        self.ros.clock=100.16
        self.sensors()
        self.node._tick()
        self.ros.clock=100.70
        self.sensors()
        self.node._tick()
        self.assertEqual(self.telemetry()["command_mode"],"hold")
        self.assertTrue(self.telemetry()["ready"])

    def test_exception_without_pose_emits_fault_but_no_uav_target(self):
        def fail():
            raise ValueError("injected failure")
        self.node._tick=fail
        self.node.tick(None)
        self.assertEqual(len(self.node.command_pub.messages),0)
        self.assertEqual(self.telemetry()["phase"],"FAULT")

    def test_velocity_header_frame_mismatch_rejected(self):
        self.node.robot["velocity_header_frame"]="odom"
        self.sensors()
        self.assertIn("Velocity frame",self.node.velocity_error)

    def test_queued_arming_cancelled_when_phase_changed_before_worker(self):
        self.sensors()
        self.session("ARMING")
        self.session("STOPPING")
        self.node._process_service_job((None,True,"ARMING","test"))
        self.assertFalse(self.node.last_service_result["sent"])
        self.assertEqual(self.node.last_service_result["outcome"],"cancelled")

    def test_queued_arming_cancelled_after_wait_for_service(self):
        self.sensors()
        self.session("ARMING")
        sent=[]
        self.ros.wait_for_service=lambda *args,**kwargs:self.session("STOPPING")
        self.ros.ServiceProxy=lambda *args,**kwargs:lambda **request:sent.append(request)
        self.node._process_service_job((None,True,"ARMING","test"))
        self.assertEqual(sent,[])
        self.assertFalse(self.node.last_service_result["sent"])

    def test_already_sent_request_records_outcome_after_phase_changes(self):
        self.sensors()
        self.session("ARMING")
        self.ros.wait_for_service=lambda *args,**kwargs:None
        def send(**request):
            self.session("STOPPING")
            return types.SimpleNamespace(success=True)
        self.ros.ServiceProxy=lambda *args,**kwargs:send
        self.node._process_service_job((None,True,"ARMING","test"))
        self.assertTrue(self.node.last_service_result["sent"])
        self.assertEqual(self.node.last_service_result["outcome"],"accepted")
        self.assertTrue(self.node.last_service_result["phase_changed_during_call"])

    def test_body_velocity_uav_config_rejected(self):
        self.config["robots"]["uav1"]["velocity_frame"]="body"
        with self.assertRaisesRegex(ValueError,"UAV velocity"):
            self.module.AgentNode()


if __name__=="__main__":
    unittest.main()
