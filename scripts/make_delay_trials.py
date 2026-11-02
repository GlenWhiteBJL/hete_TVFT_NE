#!/usr/bin/env python3
"""Generate matched 250ms-delay ROS mock trials; never change hardware configs."""
import argparse
import copy
import json
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]
SEEDS=[11,29,47]
POSITIONS={"A":[[-1.6,-1.4],[1.6,-1.4],[0.,1.6]],
           "B":[[-1.8,.5],[1.5,.3],[0.,-1.4]]}


def generate(destination):
    base=yaml.safe_load((ROOT/"config/experiment.yaml").read_text(encoding="utf-8"))
    normal=yaml.safe_load((ROOT/"config/communication_normal250.yaml").read_text(encoding="utf-8"))
    destination=Path(destination)
    destination.mkdir(parents=True,exist_ok=True)
    manifest=[]
    for name,points in POSITIONS.items():
        cases=[("baseline",0.3,None)]+[("delay",timeout,seed) for timeout in [.3,.6] for seed in SEEDS]
        for kind,timeout,seed in cases:
            tag="{}_{}_{}ms_seed{}".format(name,kind,int(timeout*1000),seed if seed is not None else "none")
            config=copy.deepcopy(base)
            for group,xy in zip(config["groups"],points):group["initial_xy"]=xy
            config["runtime"]["signal_timeout"]=timeout
            config["communication"]=copy.deepcopy(normal["communication"])
            config["communication"]["enabled"]=kind=="delay"
            config["communication"]["seed"]=seed if seed is not None else 11
            # Full config and explicit communication overlay carry identical values.
            path=destination/(tag+".yaml")
            path.write_text(yaml.safe_dump(config,sort_keys=False),encoding="utf-8")
            overlay=destination/(tag+"_communication.yaml")
            overlay.write_text(yaml.safe_dump({"communication":config["communication"]}),encoding="utf-8")
            manifest.append({"trial":tag,"initial_set":name,"initial_xy":points,"seed":seed,
                "signal_timeout_s":timeout,"kind":kind,"duration_s":80,
                "config":path.name,"communication":overlay.name})
    (destination/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")
    return manifest


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(generate(args.output),indent=2))
