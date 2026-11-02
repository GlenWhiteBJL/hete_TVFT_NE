#!/usr/bin/env python3
"""Compare completed ROS baseline and random-delay runs, including actual draws."""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def read_run(root):
    status=json.loads((root/"run-status.json").read_text())
    smoke=json.loads((root/"smoke-report.json").read_text())
    if not status.get("success") or not smoke.get("success"):
        raise ValueError("Refusing to label an incomplete ROS run successful")
    sid=smoke["session_id"]
    data=root/"output"/sid
    meta=json.loads((data/"metadata.json").read_text())
    metrics=json.loads((root/"picture"/sid/"metrics.json").read_text())
    for rid,count in smoke["recording_validation"]["formal_counts"].items():
        if metrics["agents"][rid]["formal_samples"] != count:
            raise ValueError("Formal count mismatch")
    latest={}
    with (data/"telemetry.jsonl").open() as f:
        for line in f:
            p=json.loads(line)
            if p.get("session_id")==sid: latest[p["id"]]=p
    links={}
    for p in latest.values():
        links.update(p.get("communication",{}).get("generated_per_link",{}))
    count=sum(x["count"] for x in links.values())
    mean=sum(x["sum_s"] for x in links.values())/count if count else 0.
    var=sum(x["sum_sq_s2"] for x in links.values())/count-mean**2 if count else 0.
    hist=np.sum([x["histogram"] for x in links.values()],axis=0).tolist() if links else [0]*20
    if sum(hist)!=count: raise ValueError("Incomplete delay histogram")
    leaders=[v["ne_error_m"]["tail_max_abs"] for v in metrics["agents"].values() if "ne_error_m" in v]
    followers=[v["formation_position_error_m"]["tail_max_abs"] for v in metrics["agents"].values() if "formation_position_error_m" in v]
    velocities=[v["formation_velocity_error_m_s"]["tail_max_abs"] for v in metrics["agents"].values() if "formation_velocity_error_m_s" in v]
    return {"session_id":sid,"profile":meta["config"].get("communication",{}),"metrics":metrics,
        "recording_validation":smoke["recording_validation"],"injected_draws":count,"mean_s":mean,
        "std_s":math.sqrt(max(0.,var)),"min_s":min(x["min_s"] for x in links.values()),
        "max_s":max(x["max_s"] for x in links.values()),"histogram":hist,"links":links,
        "out_of_order":sum(p.get("communication",{}).get("out_of_order",0) for p in latest.values()),
        "max_tail_leader_ne_error_m":max(leaders),"max_tail_formation_position_error_m":max(followers),
        "max_tail_formation_velocity_error_m_s":max(velocities)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline",type=Path)
    parser.add_argument("normal",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    baseline,normal=read_run(args.baseline),read_run(args.normal)
    profile=normal["profile"]
    if baseline["profile"].get("enabled",False) or profile.get("distribution")!="normal" or not profile.get("enabled"):
        raise ValueError("Expected an undelayed baseline and a normal-delay case")
    if not profile["min_delay_s"] <= normal["min_s"] <= normal["max_s"] <= profile["max_delay_s"]:
        raise ValueError("Observed injections outside configured truncation interval")
    args.output.mkdir(parents=True,exist_ok=True)
    report={"scope":"actual ROS Noetic nodes with ideal mock plants; not PX4 or hardware",
            "baseline":baseline,"normal":normal,
            "distribution_note":"Rejection-sampled truncated normal. Histograms count all generated injections since SCHEDULED, including packets later discarded as out-of-order; control data age additionally includes transport and sampling."}
    (args.output/"comparison.json").write_text(json.dumps(report,indent=2)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
    edges=np.linspace(0,profile["max_delay_s"],21)
    width=np.diff(edges)
    axes[0].bar((edges[:-1]+edges[1:])*500,np.asarray(normal["histogram"])/normal["injected_draws"]/width/1000,
                width=width*1000*.9,label="Actual generated delays")
    x=np.linspace(profile["min_delay_s"],profile["max_delay_s"],400)
    mu,sd=profile["delay_s"],profile["std_s"]
    if sd>0:
        mass=(math.erf((profile["max_delay_s"]-mu)/sd/2**.5)-math.erf((profile["min_delay_s"]-mu)/sd/2**.5))/2
        density=np.exp(-.5*((x-mu)/sd)**2)/(sd*(2*math.pi)**.5*mass)/1000
        axes[0].plot(x*1000,density,label="Configured truncated normal")
    axes[0].set(xlabel="Injected delay (ms)",ylabel="Density (1/ms)");axes[0].legend(fontsize=8)
    labels=["Leader NE (m)","Formation position (m)","Formation velocity (m/s)"]
    keys=["max_tail_leader_ne_error_m","max_tail_formation_position_error_m","max_tail_formation_velocity_error_m_s"]
    xx=np.arange(3)
    axes[1].bar(xx-.18,[max(baseline[k],1e-12) for k in keys],width=.35,label="No delay")
    axes[1].bar(xx+.18,[max(normal[k],1e-12) for k in keys],width=.35,label="Normal delay")
    axes[1].set_yscale("log");axes[1].set_xticks(xx);axes[1].set_xticklabels(labels,fontsize=8)
    axes[1].set_title("Maximum error over last 10 s");axes[1].legend()
    fig.savefig(args.output/"normal_delay_comparison.png",dpi=180);plt.close(fig)
    lines=["# ROS 正态随机通信时延验证","",report["scope"],"",
      "配置：均值 {:.1f} ms、标准差 {:.1f} ms，截断区间 [{:.1f}, {:.1f}] ms。".format(mu*1000,sd*1000,profile["min_delay_s"]*1000,profile["max_delay_s"]*1000),
      "实际生成 {} 次注入，均值 {:.3f} ms、标准差 {:.3f} ms，范围 [{:.3f}, {:.3f}] ms。".format(normal["injected_draws"],normal["mean_s"]*1000,normal["std_s"]*1000,normal["min_s"]*1000,normal["max_s"]*1000),"",
      "| 工况 | 末段最大 NE 误差(m) | 编队位置误差(m) | 编队速度误差(m/s) |","|---|---:|---:|---:|"]
    for name,r in [("无时延",baseline),("正态随机时延",normal)]:
        lines.append("| {} | {:.6g} | {:.6g} | {:.6g} |".format(name,*[r[k] for k in keys]))
    lines += ["","随机时延的旧包乱序丢弃计数：{}。源时间戳未改写，原超时阈值未放宽。".format(normal["out_of_order"]),
       "","分布统计覆盖 SCHEDULED 后全部注入；实际控制使用的数据年龄还包含传输与采样等待。此次是单种子、单次场景验证，不能据此证明任意时延下稳定或实物精度。"]
    (args.output/"REPORT.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
    print(args.output)


if __name__=="__main__":main()
