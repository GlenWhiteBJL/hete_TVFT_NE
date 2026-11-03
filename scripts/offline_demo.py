#!/usr/bin/env python3
"""Run the same local controllers against ideal sampled integrator plants.

This is an offline algorithm/integration check, not a ROS/PX4 or hardware test.
No simulator reads the true NE to compute a command. A packet snapshot is taken
once per integration step, then all nine controllers advance from that snapshot.
"""
import argparse
import copy
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import sys
import time
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT / "src"))
from ne_tvft.core import LeaderController, FollowerController, MonitoringGame, circle
from ne_tvft.communication import SampledNetwork
from ne_tvft.safety import GroupSafety, challenge_acceleration


def jsonable(value):
    if isinstance(value,np.ndarray):
        return value.tolist()
    if isinstance(value,np.generic):
        return value.item()
    if isinstance(value,dict):
        return {key:jsonable(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)):
        return [jsonable(item) for item in value]
    return value


def initialize(config):
    groups = config["groups"]
    m = len(groups)
    leaders = [LeaderController(config,i) for i in range(m)]
    followers = [[FollowerController(config,i,j) for j in range(2)] for i in range(m)]
    xl = np.array([group["initial_xy"] for group in groups],dtype=float)
    xf = np.array([[xl[i]+circle(config,i,j,0.0)[0] for j in range(2)] for i in range(m)])
    # Startup begins from hover, so the initial tangential velocity error is
    # visible instead of initializing the aircraft on a perfect circular orbit.
    vf = np.zeros_like(xf)
    for i,controller in enumerate(leaders):
        controller.reset(xl[i])
    return leaders,followers,xl,xf,vf


def snapshot(config,leaders,followers,xl,xf,vf,tau,transport=None):
    m = len(leaders)
    leader_packets = {i:{"y":controller.y.copy(),"z":controller.z.copy()} for i,controller in enumerate(leaders)}
    # Build w from only each agent's own physical state and local reference.
    group_w = []
    for i in range(m):
        values = {0:xl[i].copy()}
        for j in range(2):
            h,dh,_ = circle(config,i,j,tau)
            values[j+1] = xf[i,j]-h+(vf[i,j]-dh)/config["gains"]["lambda_follower"]
        group_w.append(values)
    outputs_l,outputs_f,neighbor_l,neighbor_f = [],[],[],[]
    A = np.asarray(config["topology"]["leaders"])
    AF = np.asarray(config["topology"]["followers"])
    pin = np.asarray(config["topology"]["pinning"])
    for i in range(m):
        received = {k:leader_packets[k] for k in range(m) if A[i,k] > 0}
        if transport is not None:
            received = {k:transport.value(config["groups"][k]["leader"],config["groups"][i]["leader"],v,tau)
                        for k,v in received.items()}
        neighbor_l.append(received)
        outputs_l.append(leaders[i].output(xl[i],received))
        group_outputs,group_neighbors = [],[]
        for j in range(2):
            received_w = {k+1:group_w[i][k+1] for k in range(2) if AF[j,k] > 0}
            if pin[i,j] > 0:
                received_w[0] = group_w[i][0]
            if transport is not None:
                group = config["groups"][i]
                received_w = {k:transport.value(group["leader"] if k == 0 else group["followers"][k-1],
                              group["followers"][j],v,tau) for k,v in received_w.items()}
            group_neighbors.append(received_w)
            group_outputs.append(followers[i][j].output(tau,xf[i,j],vf[i,j],received_w))
        outputs_f.append(group_outputs)
        neighbor_f.append(group_neighbors)
    return outputs_l,outputs_f,neighbor_l,neighbor_f


def limited_inputs(config, leader_outputs, follower_outputs, apply_limits=False):
    """Return applied commands and per-agent flags, without clipping velocity states."""
    ul = np.array([value["u"] for value in leader_outputs],dtype=float)
    uf = np.array([[value["u"] for value in row] for row in follower_outputs],dtype=float)
    limited_l = np.zeros(ul.shape[:-1],dtype=bool)
    limited_f = np.zeros(uf.shape[:-1],dtype=bool)
    if apply_limits:
        for values,flags,key in ((ul,limited_l,"ugv_speed"),(uf,limited_f,"uav_acceleration")):
            cap = float(config["limits"][key])
            if not np.isfinite(cap) or cap <= 0:
                raise ValueError("Command limit {} must be positive and finite".format(key))
            norms = np.linalg.norm(values,axis=-1)
            flags[:] = norms > cap
            values *= np.minimum(1.0,cap/np.maximum(norms,1e-300))[...,None]
    return ul,uf,limited_l,limited_f


def simulate(config,duration,dt,out=None,session_id="offline",sample_dt=.02,apply_limits=False):
    if duration <= 0 or dt <= 0 or dt > .1 or not np.isfinite([duration,dt]).all():
        raise ValueError("Require positive finite duration and 0 < dt <= 0.1 seconds")
    leaders,followers,xl,xf,vf = initialize(config)
    transport = SampledNetwork(config.get("communication", {})) if config.get("communication", {}).get("enabled",False) else None
    m = len(leaders)
    game = MonitoringGame(config)
    safety = GroupSafety(config)
    equilibrium = game.equilibrium()  # evaluation only; controllers do not see it.
    alpha_max,beta_max = 0.,0.
    limited_samples = {group["leader"]:0 for group in config["groups"]}
    limited_samples.update({name:0 for group in config["groups"] for name in group["followers"]})
    speed_violation_samples = 0
    max_uav_speed = 0.0
    start = time.perf_counter()
    logfile = (out/"telemetry.jsonl").open("w",encoding="utf-8") if out else None
    try:
        t = 0.0
        sample_at = 0.0
        final_outputs = None
        # Advance with deterministic controller/communication/plant sampling.
        while True:
            ol,of,nl,nf = snapshot(config,leaders,followers,xl,xf,vf,t,transport)
            ul,uf,limited_l,limited_f = limited_inputs(config,ol,of,apply_limits)
            safety_diagnostics = {}
            for i, group in enumerate(config["groups"]):
                for j, name in enumerate(group["followers"]):
                    challenge = challenge_acceleration(config, i, j, t, vf[i,j])
                    if challenge is not None:
                        uf[i,j] = challenge
                    if safety.enabled:
                        altitude = group["altitude"]
                        uf[i,j], diagnostic = safety.filter(uf[i,j], np.r_[xf[i,j],altitude], np.r_[vf[i,j],0.],
                            np.r_[xf[i,1-j],altitude], np.r_[vf[i,1-j],0.], np.r_[xl[i],0.], np.r_[ul[i],0.])
                        limited_f[i,j] |= diagnostic["active"]
                        safety_diagnostics[name] = dict(diagnostic, challenge_active=challenge is not None)
                    else:
                        safety_diagnostics[name] = {"enabled":False,"active":False,"challenge_active":challenge is not None}
            for i,group in enumerate(config["groups"]):
                limited_samples[group["leader"]] += int(limited_l[i])
                for j,name in enumerate(group["followers"]):
                    limited_samples[name] += int(limited_f[i,j])
            speeds = np.linalg.norm(vf,axis=-1)
            max_uav_speed = max(max_uav_speed,float(speeds.max()))
            if "uav_speed" in config.get("limits",{}):
                speed_violation_samples += int(np.count_nonzero(speeds > config["limits"]["uav_speed"]))
            final_outputs = (ol,of)
            alpha_max = max(alpha_max,max(float(v["alpha"].max()) for v in ol))
            beta_max = max(beta_max,max(v["beta"] for row in of for v in row))
            if logfile and (t >= sample_at-1e-9 or t >= duration-1e-9):
                sample_at += sample_dt
                for i,group in enumerate(config["groups"]):
                    records = [(group["leader"], [xl[i,0],xl[i,1],0.0],
                                [ul[i,0],ul[i,1],0.0], [0.,0.,0.],ol[i],"ugv",ul[i],bool(limited_l[i]))]
                    for j,name in enumerate(group["followers"]):
                        records.append((name,[xf[i,j,0],xf[i,j,1],group["altitude"]],
                                        [vf[i,j,0],vf[i,j,1],0.0],
                                        [uf[i,j,0],uf[i,j,1],0.0],of[i][j],"uav",uf[i,j],bool(limited_f[i,j])))
                    for name,position,velocity,acceleration,controller,kind,applied,limited in records:
                        command = list(controller["u"])+[0.0]
                        applied_command = list(applied)+[0.0]
                        row = {"schema_version":1,"id":name,"agent_id":name,"kind":kind,"group_index":i,
                            "group":i,"role":"leader" if kind=="ugv" else "follower",
                            "session_id":session_id,"phase":"RUNNING","sim_time":t,"time":t,"stamp":t,"tau":t,
                            "position":position,"velocity":velocity,"acceleration_estimate":acceleration,
                            "acceleration_valid":kind=="uav","input_raw":command,"input_applied":applied_command,
                            "controller":controller,"limited":limited,"start_offset_s":0.0,"dt":dt,
                            "healthy":True,"pose_stamp":t,"velocity_stamp":t,
                            "safety":safety_diagnostics.get(name,{"enabled":False,"active":False})}
                        logfile.write(json.dumps(jsonable(row),allow_nan=False)+"\n")
            if t >= duration-1e-10:
                break
            step = min(dt,duration-t)
            old_xl,old_xf,old_vf = xl.copy(),xf.copy(),vf.copy()
            # All controller updates consume immutable pre-step packets.
            for i in range(m):
                leaders[i].step(step,old_xl[i],nl[i])
                for j in range(2):
                    followers[i][j].step(step,t,old_xf[i,j],old_vf[i,j],nf[i][j])
            xl = old_xl+step*ul
            xf = old_xf+step*old_vf+0.5*step**2*uf
            vf = old_vf+step*uf
            if not all(np.isfinite(x).all() for x in (xl,xf,vf)):
                raise FloatingPointError("Non-finite offline plant state")
            t = min(duration,t+step)
        ol,of = final_outputs
        position_error = np.array([[xf[i,j]-circle(config,i,j,t)[0]-xl[i] for j in range(2)] for i in range(m)])
        final_ul,_,_,_ = limited_inputs(config,ol,of,apply_limits)
        velocity_error = np.array([[vf[i,j]-circle(config,i,j,t)[1]-final_ul[i] for j in range(2)] for i in range(m)])
        return {"duration_s":duration,"integration_dt_s":dt,"wall_seconds":time.perf_counter()-start,
                "equilibrium":equilibrium.tolist(),"game_certificate":game.certificate(),
                "leader_ne_error_max_m":float(np.linalg.norm(xl-equilibrium,axis=-1).max()),
                "formation_position_error_max_m":float(np.linalg.norm(position_error,axis=-1).max()),
                "formation_velocity_error_max_m_s":float(np.linalg.norm(velocity_error,axis=-1).max()),
                "peak_alpha":alpha_max,"peak_beta":beta_max,"final_xl":xl.tolist(),"final_xf":xf.tolist(),
                "apply_limits":bool(apply_limits),"safety_enabled":safety.enabled,
                "limited_output_samples_per_agent":limited_samples,
                "maximum_uav_speed_m_s":max_uav_speed,"uav_speed_limit_violation_samples":speed_violation_samples,
                "limit_semantics":"Nominal UGV speed/UAV acceleration commands are clipped when apply_limits=true; "
                    "an enabled safety filter additionally enforces its acceleration/predicted-speed limits regardless of that flag. "
                    "UAV speed is measured/reported, never changed by clipping the physical velocity state.",
                "interpretation":"Ideal sampled plant with same local controllers and optional safety filter; no ROS, firmware, localization noise or actuator dynamics. "
                    "Optional command saturation is explicit; sample/hold discretization can produce a finite circular tracking residual."}
    finally:
        if logfile:
            logfile.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,default=ROOT/"config"/"experiment.yaml")
    parser.add_argument("--duration",type=float,default=80.)
    parser.add_argument("--dt",type=float,default=.02)
    parser.add_argument("--output-root",type=Path,default=ROOT/"output")
    parser.add_argument("--refine",action="store_true",help="Repeat with half the sample/integration period and compare terminal states")
    parser.add_argument("--apply-limits",action="store_true",help="Apply configured UGV-speed/UAV-acceleration norm limits; report UAV-speed violations without altering velocity state")
    args = parser.parse_args()
    with args.config.open(encoding="utf-8") as source:
        config = yaml.safe_load(source)
    sid = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")+("_offline_bounded" if args.apply_limits else "_offline")
    out = args.output_root.resolve()/sid
    out.mkdir(parents=True,exist_ok=False)
    metadata = {"schema_version":1,"session_id":sid,"config":config,"source":"offline_demo",
                "t0":0.0,"duration_s":args.duration,"dt":args.dt,"apply_limits":bool(args.apply_limits),
                "mode":"sampled_integrator_with_command_limits" if args.apply_limits else "ideal_sampled_integrator",
                "agent_ids":[name for group in config["groups"] for name in [group["leader"]]+group["followers"]]}
    (out/"metadata.json").write_text(json.dumps(metadata,indent=2,ensure_ascii=False,allow_nan=False),encoding="utf-8")
    result = simulate(config,args.duration,args.dt,out,sid,apply_limits=args.apply_limits)
    if args.refine:
        refined = simulate(copy.deepcopy(config),args.duration,args.dt/2.,apply_limits=args.apply_limits)
        differences = {name:float(np.max(np.abs(np.asarray(result[name])-np.asarray(refined[name]))))
                       for name in ("final_xl","final_xf")}
        result["half_period_refinement"] = {"maximum_terminal_absolute_differences":differences,
            "refined_formation_position_error_max_m":refined["formation_position_error_max_m"],
            "refined_formation_velocity_error_max_m_s":refined["formation_velocity_error_max_m_s"],
            "note":"Compares two different sampled implementations; expected first-order sample/hold error, not the RK4 local truncation error alone."}
    (out/"offline_summary.json").write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False),encoding="utf-8")
    print(json.dumps({"output":str(out),**result},indent=2,ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
