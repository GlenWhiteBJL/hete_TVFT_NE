#!/usr/bin/env python3
"""Offline delay comparison; no ROS, FCU or physical network is exercised."""
import argparse
import copy
import csv
from datetime import datetime, timezone, timedelta
import json
import math
from pathlib import Path
import sys
import yaml
from offline_demo import simulate, ROOT


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=80.)
    p.add_argument("--dt", type=float, default=.005)
    p.add_argument("--delays-ms", type=float, nargs="+", default=[0,20,50,100])
    p.add_argument("--output-root", type=Path, default=ROOT/"output")
    args = p.parse_args()
    if (not math.isfinite(args.duration) or args.duration <= 0 or not math.isfinite(args.dt)
            or not 0 < args.dt <= .1 or any(not math.isfinite(x) or x < 0 for x in args.delays_ms)):
        p.error("Require finite positive duration, 0 < dt <= 0.1, and nonnegative delays")
    base = yaml.safe_load((ROOT/"config/experiment.yaml").read_text(encoding="utf-8"))
    sid = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")+"_delay_sweep"
    out = args.output_root.resolve()/sid
    out.mkdir(parents=True)
    results = []
    for i, delay in enumerate(args.delays_ms):
        config = copy.deepcopy(base)
        config["communication"] = {"enabled": True, "delay_s": delay/1000., "seed":7}
        name = "case_{}_{:g}ms".format(i,delay)
        case = out/name
        case.mkdir()
        metadata = {"session_id": name,"t0":0.,"config":config,"source":"offline_delay_sweep",
                    "history":"constant initial signals for t<0; zero-order hold until first release",
                    "scope":"offline sampled integrators; does not exercise ROS watchdogs"}
        (case/"metadata.json").write_text(json.dumps(metadata,indent=2),encoding="utf-8")
        (case/"config_used.yaml").write_text(yaml.safe_dump(config),encoding="utf-8")
        try:
            result = simulate(config,args.duration,args.dt,case,name,apply_limits=True)
            result.update(delay_ms=delay,status="completed",case=name)
        except (ValueError, FloatingPointError,OverflowError) as exc:
            result = {"delay_ms":delay,"status":"failed","error":str(exc),"case":name}
        (case/"offline_summary.json").write_text(json.dumps(result,indent=2,allow_nan=False),encoding="utf-8")
        results.append(result)
        print("{} ms: {}".format(delay,result["status"]),flush=True)
    (out/"comparison.json").write_text(json.dumps(results,indent=2,allow_nan=False),encoding="utf-8")
    fields = ["delay_ms","status","leader_ne_error_max_m","formation_position_error_max_m",
              "formation_velocity_error_max_m_s","maximum_uav_speed_m_s","uav_speed_limit_violation_samples"]
    with (out/"comparison.csv").open("w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fields,extrasaction="ignore");writer.writeheader();writer.writerows(results)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(13,4),constrained_layout=True)
    for ax,key in zip(axes,fields[2:5]):
        good=[r for r in results if key in r]
        ax.semilogy([r["delay_ms"] for r in good],[max(r[key],1e-12) for r in good],"o-")
        ax.set(xlabel="Injected delay (ms)",ylabel=key,title="Terminal error (offline)");ax.grid(True,alpha=.3)
    fig.savefig(out/"comparison.png",dpi=180)
    plt.close(fig)
    print(out,flush=True)
    return 0 if all(r["status"] == "completed" for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
