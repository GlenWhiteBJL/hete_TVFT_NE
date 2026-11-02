#!/usr/bin/env python3
"""Summarize every ROS attempt; faults and startup failures are never convergence."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import numpy as np


def summarize(results, manifest, output):
    state=json.loads(Path(manifest).read_text())
    cases=state["cases"]
    rows=[];hist=np.zeros(20,dtype=np.int64)
    for case in cases:
        run=Path(results)/case["directory"] if case.get("directory") else None
        row={k:case[k] for k in ["trial","initial_set","seed","signal_timeout_s","kind","duration_s","exit_code"]}
        row.update(outcome="process_failure",reason="",session_id=None,formal_samples=0,
                   ne_tail_m=None,position_tail_m=None,velocity_tail_m_s=None,
                   converged_005=False,draws=0,mean_delay_s=None,std_delay_s=None,
                   first_fault_source_age_s=None)
        def load(name):
            p=run/name if run else None
            return json.loads(p.read_text()) if p and p.exists() else {}
        status,smoke=load("run-status.json"),load("smoke-report.json")
        row["reason"]=smoke.get("error","")
        sid=smoke.get("session_id")
        if not sid and run and (run/"output").exists():
            candidates=list((run/"output").iterdir())
            if len(candidates)==1:sid=candidates[0].name
        row["session_id"]=sid
        latest={}
        if run and sid:
            meta=load("output/"+sid+"/metadata.json")
            saved=meta.get("config",{})
            if saved and (saved["runtime"]["signal_timeout"]!=case["signal_timeout_s"]
                    or [x["initial_xy"] for x in saved["groups"]]!=case["initial_xy"]):
                raise ValueError("Actual configuration differs from trial design")
            file=run/"output"/sid/"telemetry.jsonl"
            if file.exists():
                with file.open() as f:
                    for line in f:
                        p=json.loads(line)
                        if p.get("session_id")==sid:
                            latest[p["id"]]=p
                            if p.get("fault_reason") and row["first_fault_source_age_s"] is None:
                                ages=p.get("communication",{}).get("neighbor_source_age_s",{})
                                if ages:row["first_fault_source_age_s"]=max(ages.values())
                            if p.get("phase")=="RUNNING" and p["stamp"]>=float(meta.get("t0") or 0):row["formal_samples"]+=1
            links={}
            for p in latest.values():links.update(p.get("communication",{}).get("generated_per_link",{}))
            count=sum(x["count"] for x in links.values())
            if count:
                mean=sum(x["sum_s"] for x in links.values())/count
                var=max(0.,sum(x["sum_sq_s2"] for x in links.values())/count-mean**2)
                row.update(draws=count,mean_delay_s=mean,std_delay_s=math.sqrt(var))
                if case["kind"]=="delay":hist+=np.sum([x["histogram"] for x in links.values()],axis=0)
        if status.get("success") and smoke.get("success") and case["exit_code"]==0:
            metrics=load("picture/"+sid+"/metrics.json")
            counts=smoke["recording_validation"]["formal_counts"]
            if len(counts)!=9 or any(metrics["agents"][name]["formal_samples"]!=n for name,n in counts.items()):
                raise ValueError("Formal data count mismatch")
            row.update(outcome="completed",formal_samples=sum(counts.values()),
                ne_tail_m=max(v["ne_error_m"]["tail_max_abs"] for v in metrics["agents"].values() if "ne_error_m" in v),
                position_tail_m=max(v["formation_position_error_m"]["tail_max_abs"] for v in metrics["agents"].values() if "formation_position_error_m" in v),
                velocity_tail_m_s=max(v["formation_velocity_error_m_s"]["tail_max_abs"] for v in metrics["agents"].values() if "formation_velocity_error_m_s" in v))
            row["converged_005"]=all(row[k] is not None and row[k]<=.05 for k in ["ne_tail_m","position_tail_m","velocity_tail_m_s"])
        else:
            reason=row["reason"].lower()
            if "neighbor" in reason:row["outcome"]="communication_guard"
            elif "speed limit" in reason:row["outcome"]="speed_guard"
            elif "deadline" in reason or "clock" in reason:row["outcome"]="timing_fault"
            elif "scheduled" in reason or "ack" in reason:row["outcome"]="startup_barrier"
            elif "fault" in reason:row["outcome"]="fault"
        log=run/"unit-tests.log" if run else None
        match=re.search(r"Ran (\d+) tests in .*\n\s*\nOK\s*$",log.read_text()) if log and log.exists() else None
        row["unit_tests_passed"]=int(match.group(1)) if match else None
        rows.append(row)
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    groups={}
    for key,predicate in [("baseline",lambda r:r["kind"]=="baseline"),
        ("guard300",lambda r:r["kind"]=="delay" and r["signal_timeout_s"]==.3),
        ("study600",lambda r:r["kind"]=="delay" and r["signal_timeout_s"]==.6)]:
        group=[r for r in rows if predicate(r)]
        complete=[r for r in group if r["outcome"]=="completed"]
        groups[key]={"attempts":len(group),"completed":len(complete),
                     "within_005":sum(r["converged_005"] for r in group),
                     "worst_completed_position_tail_m":max([r["position_tail_m"] for r in complete],default=None),
                     "worst_completed_ne_tail_m":max([r["ne_tail_m"] for r in complete],default=None)}
    report={"scope":"ROS Noetic ideal mock; no PX4 or hardware", "cases":rows,"groups":groups,
            "batch_stage":state["stage"],"executed_trials":len(rows),"planned_trials":14,
            "parameter_distribution":{"mean_s":.25,"std_s":.05,"min_s":0.,"max_s":.5},
            "injection_histogram":hist.tolist(),"completion_of_batch_is_not_success_of_all_trials":True}
    (output/"summary.json").write_text(json.dumps(report,indent=2)+"\n")
    with (output/"trials.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,list(rows[0]));w.writeheader();w.writerows(rows)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(15,4),constrained_layout=True)
    labels=list(groups)
    axes[0].bar(labels,[groups[k]["attempts"] for k in labels],alpha=.3,label="Attempts")
    axes[0].bar(labels,[groups[k]["completed"] for k in labels],width=.4,label="Completed 80 s")
    axes[0].set(ylabel="Trials",title="Complete outcomes, including failures");axes[0].legend(fontsize=8)
    for initial,marker in [("A","o"),("B","s")]:
        for kind,color,position in [("baseline","#2563eb",0),("guard300","#f59e0b",1),("study600","#16a34a",2)]:
            values=[r["position_tail_m"] for r in rows if r["initial_set"]==initial and r["position_tail_m"] is not None and
                    ((kind=="baseline" and r["kind"]=="baseline") or (r["kind"]=="delay" and r["signal_timeout_s"]==(.3 if kind=="guard300" else .6) and kind!="baseline"))]
            axes[1].scatter([position]*len(values),values,marker=marker,color=color)
    axes[1].set_xticks([0,1,2]);axes[1].set_xticklabels(labels);axes[1].set_yscale("log")
    axes[1].set(ylabel="Last-10-s max formation error (m)",title="Only completed runs; failures omitted")
    from matplotlib.lines import Line2D
    axes[1].legend(handles=[Line2D([],[],marker=m,color="gray",linestyle="",label="Initial "+i)
                           for i,m in [("A","o"),("B","s")]],fontsize=8)
    if hist.sum():axes[2].bar(np.arange(20)*25+12.5,hist/hist.sum()/25,width=22)
    axes[2].set(xlabel="Generated injection (ms)",ylabel="Density (1/ms)",title="All delay attempts, including startup")
    fig.savefig(output/"matrix_comparison.png",dpi=180);plt.close(fig)
    lines=["# 250 ms 正态随机时延多次 ROS 验证","","参数：均值250 ms、标准差50 ms、拒绝采样截断到0–500 ms；种子11/29/47，初始位置A/B。","",
        "300 ms为原保护组；600 ms为用户确认的纯mock研究组。其余定位/监督器超时、执行限幅、控制增益、目标和拓扑保持一致。","",
        "| 分组 | 尝试数 | 完整80秒 | 末段误差满足0.05m/0.05m/s |","|---|---:|---:|---:|"]
    for key,g in groups.items():lines.append("| {} | {} | {} | {} |".format(key,g["attempts"],g["completed"],g["within_005"]))
    lines += ["","| 初始位置 | 种子 | 通信超时(ms) | 结果 | 编队位置末段最大误差(m) |","|---|---:|---:|---|---:|"]
    for r in rows:lines.append("| {} | {} | {} | {} | {} |".format(r["initial_set"],r["seed"],int(r["signal_timeout_s"]*1000),r["outcome"],"--" if r["position_tail_m"] is None else "{:.6g}".format(r["position_tail_m"])))
    lines += ["","所有失败保留在trials.csv/summary.json中，含原始错误原因；未完成或未启动的试验不被记为收敛，图中没有用零填充失败误差。",
        "0.05阈值指最后10秒的三个最大误差都满足阈值，并不是控制律的理论收敛证明。全部为理想动力学ROS mock，不能作为PX4或实物精度证明。"]
    (output/"REPORT.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
    print(json.dumps(groups))


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("results",type=Path);p.add_argument("--manifest",type=Path,required=True);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();summarize(a.results,a.manifest,a.output)
