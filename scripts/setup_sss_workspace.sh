#!/usr/bin/env bash
# Run inside xgc. Keep SSS and experiment overlays outside xgc_ws/src.
set -eo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="${1:-$(cd "$repo_root/../.." && pwd)}"
platform="$workspace_root/swarm_sync_sim"
overlay="$workspace_root/sss_experiment_ws"
revision=4837b511c3d2bcaa36448e5e5af51335004bafd8
if [[ ! -d "$platform/.git" ]]; then
    git -c http.version=HTTP/1.1 clone https://github.com/shupx/swarm_sync_sim.git "$platform"
fi
# Explicitly select this checkout across the Docker bind mount. Older Noetic
# Git backports do not support per-command safe.directory configuration.
git_sss() { git -C "$platform" --git-dir="$platform/.git" --work-tree="$platform" "$@"; }
if [[ "$(git_sss rev-parse HEAD)" != "$revision" ]]; then
    if [[ -n "$(git_sss status --porcelain)" ]]; then
        echo "SSS has local changes at another revision: $platform" >&2; exit 1
    fi
    git_sss checkout --detach "$revision"
fi
patch_file="$repo_root/integration/swarm_sync_sim.patch"
if git_sss apply --reverse --check "$patch_file" 2>/dev/null; then
    echo 'SSS land-state patch is already applied.'
elif git_sss apply --check "$patch_file"; then
    git_sss apply "$patch_file"
else
    echo 'SSS source does not match the pinned patch; inspect local changes.' >&2; exit 1
fi
source /opt/ros/noetic/setup.bash
(cd "$platform" && catkin_make -j2 -l2 \
    '-DCATKIN_WHITELIST_PACKAGES=sss_sim_env;px4_rotor_sim;ugv_sim' -DCMAKE_BUILD_TYPE=Release)
source "$platform/devel/setup.bash"
mkdir -p "$overlay/src"
if [[ -e "$overlay/src/ne_tvft_experiment" && "$(readlink -f "$overlay/src/ne_tvft_experiment")" != "$repo_root" ]]; then
    echo 'Experiment overlay points to another source checkout.' >&2; exit 1
fi
ln -sfn "$repo_root" "$overlay/src/ne_tvft_experiment"
(cd "$overlay" && catkin_make -j2 -l2)
printf '\nSource this overlay before launching:\nsource %q/devel/setup.bash\n' "$overlay"
