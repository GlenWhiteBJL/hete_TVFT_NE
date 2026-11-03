#!/usr/bin/env bash
# Run an isolated, headless end-to-end SSS test inside xgc.
set -eo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="${NE_TVFT_WS_ROOT:-$(cd "$repo_root/../.." && pwd)}"
duration="${1:-80}"
profile="${2:-communication_none.yaml}"
source /opt/ros/noetic/setup.bash
source "$workspace_root/sss_experiment_ws/devel/setup.bash"
mkdir -p "$repo_root/output"
run_dir="$(mktemp -d "$repo_root/output/sss_run_XXXXXXXX")"
chmod 755 "$run_dir"
mkdir -p "$run_dir/output" "$run_dir/picture" "$run_dir/ros"
export ROS_HOME="$run_dir/ros" ROS_HOSTNAME=127.0.0.1 MPLBACKEND=Agg
unset ROS_IP
port="$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1]); s.close()')"
export ROS_MASTER_URI="http://127.0.0.1:$port"
export PYTHONDONTWRITEBYTECODE=1
launch_pid=
cleanup() {
    status=$?
    trap - EXIT INT TERM
    if [[ -n "$launch_pid" ]]; then
        kill -INT -- "-$launch_pid" 2>/dev/null || true
        for attempt in $(seq 1 80); do
            kill -0 -- "-$launch_pid" 2>/dev/null || break
            sleep .2
        done
        kill -TERM -- "-$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
    echo "SSS test artifacts: $run_dir (exit $status)"
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid roslaunch ne_tvft_experiment sss.launch "open_rviz:=${NE_TVFT_OPEN_RVIZ:-false}" \
    "config:=${NE_TVFT_SSS_CONFIG:-$repo_root/config/sss_experiment.yaml}" \
    "enable_safety:=${NE_TVFT_SAFETY:-true}" \
    "communication:=$repo_root/config/$profile" \
    "output_root:=$run_dir/output" "picture_root:=$run_dir/picture" \
    >"$run_dir/roslaunch.log" 2>&1 &
launch_pid=$!
python3 -B "$repo_root/scripts/smoke_sss.py" --duration "$duration" \
    --output-root "$run_dir/output" --report "$run_dir/smoke-report.json"
session_dir="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["session_directory"])' "$run_dir/smoke-report.json")"
python3 -B "$repo_root/scripts/analyze_experiment.py" "$session_dir" --picture-root "$run_dir/picture" > "$run_dir/analysis.log" 2>&1
