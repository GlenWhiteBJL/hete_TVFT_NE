#!/usr/bin/env bash
# Linux host/Codex Cloud entry point. Uses only the local Docker socket.
set -euo pipefail

if (( $# > 1 )); then
    printf 'Usage: bash cloud/run_docker.sh [control_duration_seconds]\n' >&2
    exit 2
fi
duration="${1:-20}"
delay_profile="${NE_TVFT_DELAY_PROFILE:-communication_none.yaml}"
case "$delay_profile" in
    communication_none.yaml|communication_20ms.yaml|communication_50ms.yaml|communication_100ms.yaml) ;;
    *) printf 'Unknown communication profile\n' >&2; exit 2 ;;
esac
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
package_root="$(cd -- "$script_dir/.." && pwd -P)"
image="${NE_TVFT_IMAGE:-ne-tvft-noetic:local}"
artifact_root="$package_root/cloud-results"

docker_local() {
    env -u DOCKER_HOST -u DOCKER_CONTEXT -u DOCKER_TLS \
        -u DOCKER_TLS_VERIFY -u DOCKER_CERT_PATH \
        docker --host=unix:///var/run/docker.sock "$@"
}
if ! command -v docker >/dev/null 2>&1; then
    printf 'Docker is missing; provision Docker in the cloud environment first.\n' >&2
    exit 2
fi
if ! docker_local info >/dev/null; then
    printf 'The local Docker daemon is unavailable; recover the managed environment before retrying.\n' >&2
    exit 2
fi

mkdir -p -- "$artifact_root"
build_log="$artifact_root/build_$(date -u +%Y%m%dT%H%M%S)_${RANDOM}.log"
ca_dir=''
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    set +e
    # This path is exactly the private directory returned by mktemp below.
    if [[ -n "$ca_dir" ]]; then
        rm -f -- "$ca_dir/ca-bundle.pem"
        rmdir -- "$ca_dir"
    fi
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

build_args=(--file "$script_dir/Dockerfile" --tag "$image")
# Preserve the reachable proxy endpoint supplied by the environment. APT also
# needs lowercase proxy variables; do not replace endpoints with a guessed host.
if [[ -n "${HTTP_PROXY:-}" && -z "${http_proxy:-}" ]]; then export http_proxy="$HTTP_PROXY"; fi
if [[ -n "${HTTPS_PROXY:-}" && -z "${https_proxy:-}" ]]; then export https_proxy="$HTTPS_PROXY"; fi
if [[ -n "${NO_PROXY:-}" && -z "${no_proxy:-}" ]]; then export no_proxy="$NO_PROXY"; fi
for proxy_name in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
    if [[ -v "$proxy_name" ]]; then
        build_args+=(--build-arg "$proxy_name")
    fi
done

if [[ -n "${CODEX_PROXY_CERT:-}" ]]; then
    if [[ ! -r "$CODEX_PROXY_CERT" || ! -r /etc/ssl/certs/ca-certificates.crt ]]; then
        printf 'The configured proxy CA or system CA bundle is not readable.\n' >&2
        exit 2
    fi
    ca_dir="$(mktemp -d "${TMPDIR:-/tmp}/ne-tvft-ca.XXXXXXXX")"
    chmod 700 "$ca_dir"
    cat /etc/ssl/certs/ca-certificates.crt "$CODEX_PROXY_CERT" >"$ca_dir/ca-bundle.pem"
    build_args+=(--secret "id=ca_bundle,src=$ca_dir/ca-bundle.pem")
fi

export DOCKER_BUILDKIT=1
printf 'Building ROS Noetic container; log: %s\n' "$build_log"
docker_local build "${build_args[@]}" "$package_root" >"$build_log" 2>&1
docker_local image inspect --format '{{.Id}}' "$image" >"$build_log.image-id"

printf 'Running mock workflow in an isolated container...\n'
# No host network, published ports, GPU, privileged mode, or host ROS connection.
# The controller, mock fleet, ROS master, and recorder share container loopback.
docker_local run --rm --init --network none --shm-size 256m \
    -e "NE_TVFT_DELAY_PROFILE=$delay_profile" \
    --user "$(id -u):$(id -g)" \
    --mount "type=bind,src=$artifact_root,dst=/artifacts" \
    "$image" "$duration"
