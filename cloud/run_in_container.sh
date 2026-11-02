#!/usr/bin/env bash
# Run inside the built Noetic container. Never connects to a hardware master.
set -euo pipefail

if (( $# > 1 )); then
    printf 'Usage: %s [control_duration_seconds]\n' "$0" >&2
    exit 2
fi
duration="${1:-20}"
python3 - "$duration" <<'PY'
import math
import sys
seconds = float(sys.argv[1])
if not math.isfinite(seconds) or not 1 <= seconds <= 90:
    raise SystemExit("Control duration must be between 1 and 90 seconds")
PY

artifact_root="${NE_TVFT_ARTIFACT_ROOT:-/artifacts}"
if [[ "$artifact_root" != /* ]]; then
    printf 'Artifact root must be an absolute path\n' >&2
    exit 2
fi
mkdir -p -- "$artifact_root"
run_id="$(date -u +%Y%m%dT%H%M%S)_${RANDOM}_ros_mock"
run_dir="$artifact_root/$run_id"
mkdir -- "$run_dir"
mkdir -- "$run_dir/output" "$run_dir/picture" "$run_dir/ros" "$run_dir/matplotlib"

stage=initializing
launch_pid=''
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    set +e
    # setsid below gives only this launch tree its own process group.
    if [[ -n "$launch_pid" ]] && kill -0 -- "-$launch_pid" 2>/dev/null; then
        kill -INT -- "-$launch_pid" 2>/dev/null
        for (( attempt=0; attempt<50; attempt++ )); do
            kill -0 -- "-$launch_pid" 2>/dev/null || break
            sleep 0.2
        done
        if kill -0 -- "-$launch_pid" 2>/dev/null; then
            kill -TERM -- "-$launch_pid" 2>/dev/null
            for (( attempt=0; attempt<25; attempt++ )); do
                kill -0 -- "-$launch_pid" 2>/dev/null || break
                sleep 0.2
            done
        fi
        kill -KILL -- "-$launch_pid" 2>/dev/null
        wait "$launch_pid" 2>/dev/null
    fi
    python3 - "$run_dir/run-status.json" "$status" "$stage" "$duration" <<'PY'
import datetime
import json
from pathlib import Path
import sys
path, code, stage, duration = sys.argv[1:]
Path(path).write_text(json.dumps({
    "success": int(code) == 0,
    "exit_code": int(code),
    "last_stage": stage,
    "requested_control_duration_s": float(duration),
    "finished_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "verification_scope": "ROS Noetic + ideal mock fleet; no PX4 SITL or hardware",
}, indent=2) + "\n", encoding="utf-8")
PY
    printf 'Run artifacts: %s (exit %s, stage %s)\n' "$run_dir" "$status" "$stage"
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# ROS setup scripts do not promise nounset compatibility.
set +u
source /opt/ros/noetic/setup.bash
source /opt/ne_tvft_ws/devel/setup.bash
set -u
cd /opt/ne_tvft_ws/src/ne_tvft_experiment
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
export MPLBACKEND=Agg
export MPLCONFIGDIR="$run_dir/matplotlib"
export ROS_HOME="$run_dir/ros"
export ROS_LOG_DIR="$run_dir/ros/log"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export NO_PROXY="${NO_PROXY:+$NO_PROXY,}localhost,127.0.0.1,::1"
export no_proxy="${no_proxy:+$no_proxy,}localhost,127.0.0.1,::1"
master_port="$(python3 - <<'PY'
import socket
with socket.socket() as listener:
    listener.bind(("127.0.0.1", 0))
    print(listener.getsockname()[1])
PY
)"
export ROS_MASTER_URI="http://127.0.0.1:$master_port"

stage=unit_tests
printf 'Running Python tests...\n'
python3 -B -m unittest discover -s tests -v >"$run_dir/unit-tests.log" 2>&1
stage=preflight
python3 -B scripts/preflight.py >"$run_dir/preflight.json" 2>"$run_dir/preflight.log"

stage=ros_mock_launch
printf 'Starting 3 UGV + 6 UAV ROS mock fleet...\n'
set +m
setsid roslaunch ne_tvft_experiment experiment.launch \
    mock:=true allow_hardware:=false \
    "communication:=$PWD/config/${NE_TVFT_DELAY_PROFILE:-communication_none.yaml}" \
    "output_root:=$run_dir/output" "picture_root:=$run_dir/picture" \
    >"$run_dir/roslaunch.log" 2>&1 &
launch_pid=$!

stage=ros_mock_workflow
python3 -B scripts/smoke_mock.py \
    --duration "$duration" \
    --report "$run_dir/smoke-report.json" \
    --output-root "$run_dir/output" \
    >"$run_dir/smoke.log" 2>&1

stage=analysis
session_directory="$(python3 - "$run_dir/smoke-report.json" <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)
if report.get("success") is not True:
    raise SystemExit("ROS workflow did not report success")
print(report["session_directory"])
PY
)"
python3 -B scripts/analyze_experiment.py "$session_directory" \
    --picture-root "$run_dir/picture" >"$run_dir/analysis.log" 2>&1
stage=completed
printf 'ROS mock workflow and data analysis completed.\n'
