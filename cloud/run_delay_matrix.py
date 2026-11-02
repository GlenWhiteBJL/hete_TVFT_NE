#!/usr/bin/env python3
"""Run all 14 delay trials serially on a Linux Docker host; retain every failure."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New batch directory")
    parser.add_argument("--image", default="ne-tvft-delay:250")
    parser.add_argument("--sudo", action="store_true", help="Use sudo -n for the local Docker socket")
    parser.add_argument("--skip-build", action="store_true", help="Use an already verified image")
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    docker = (["sudo", "-n"] if args.sudo else []) + ["docker", "--host=unix:///var/run/docker.sock"]
    state = dict(root=str(root), package=str(package), stage="build", cases=[], success=False)

    def save():
        temporary = root / "trial-status.json.tmp"
        temporary.write_text(json.dumps(state, indent=2) + "\n")
        temporary.replace(root / "trial-status.json")

    def run(command, log):
        with (root / log).open("w") as stream:
            return subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT).returncode

    save()
    try:
        if not args.skip_build:
            code = run(docker + ["build", "-f", str(package / "cloud/Dockerfile"),
                                "-t", args.image, str(package)], "build.log")
            if code:
                raise RuntimeError("Build failed; see build.log")
        common = docker + ["run", "--rm", "--init", "--network", "none", "--shm-size", "256m",
                           "--user", "{}:{}".format(os.getuid(), os.getgid())]
        mount = ["--mount", "type=bind,src={},dst=/batch".format(root)]
        code = run(common + mount + ["--entrypoint", "python3", args.image,
                   "scripts/make_delay_trials.py", "--output", "/batch/trials"], "manifest-generation.log")
        if code:
            raise RuntimeError("Trial generation failed")
        manifest = json.loads((root / "trials/manifest.json").read_text())
        results = root / "cloud-results"
        results.mkdir()
        for index, trial in enumerate(manifest, 1):
            state.update(stage=trial["trial"], index=index, total=len(manifest))
            save()
            if shutil.disk_usage(root).free < 2 * 1024 ** 3:
                raise RuntimeError("Less than 2 GiB free; stopping before another trial")
            log = trial["trial"] + ".log"
            code = run(common + ["--mount", "type=bind,src={},dst=/artifacts".format(results),
                "--mount", "type=bind,src={},dst=/trials,readonly".format(root / "trials"),
                "-e", "NE_TVFT_EXPERIMENT_CONFIG=/trials/" + trial["config"],
                "-e", "NE_TVFT_COMMUNICATION_CONFIG=/trials/" + trial["communication"],
                args.image, str(trial["duration_s"])], log)
            found = re.findall(r"Run artifacts: /artifacts/([^\s]+)", (root / log).read_text())
            state["cases"].append(dict(trial, exit_code=code, directory=found[-1] if found else None))
            save()
        state.update(stage="completed", success=True)
        save()  # Batch completion is separate from each trial's outcome.
        code = run(common + mount + ["--entrypoint", "python3", args.image,
            "scripts/summarize_delay_trials.py", "/batch/cloud-results", "--manifest",
            "/batch/trial-status.json", "--output", "/batch/summary"], "summary.log")
        if code:
            raise RuntimeError("Summarization failed; raw attempts have been retained")
        print(root / "summary/REPORT.md")
    except Exception as exc:
        state.update(stage="failed", error=str(exc))
        save()
        raise


if __name__ == "__main__":
    main()
