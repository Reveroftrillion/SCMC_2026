#!/usr/bin/env bash
# Usage: bash docker/run.sh [command ...]; default is a shell, never driving.
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo=$(cd "$here/../.." && pwd)
mkdir -p "$here/artifacts"
chmod g+ws "$here/artifacts"
options=(--rm --init --gpus all --network host --shm-size 1g
  --user "$(id -u):$(id -g)" -e HOME=/tmp/rmt-home
  -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1
  -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1
  --mount "type=bind,src=$repo,dst=/workspace"
  --workdir /workspace/ros1-port)
if [[ -t 0 && -t 1 ]]; then options+=(-it); fi
if [[ ${1:-} == --receive-root ]]; then
  # Competition status is UDP 909 (<1024) on this host. Limit root to binding.
  # Source is read-only for runtime; build as the host user first.
  shift
  options+=(--user 0:0 --group-add "$(id -g)" --cap-drop ALL --cap-add NET_BIND_SERVICE
    --mount "type=bind,src=$repo/2026_CTRL_competition,dst=/workspace/2026_CTRL_competition,readonly"
    --mount "type=bind,src=$repo/ros1-port/src,dst=/workspace/ros1-port/src,readonly")
fi
if [[ ${1:-} == --capture-control ]]; then
  shift
  options+=(--cap-add NET_RAW)
fi
if [[ $# == 0 ]]; then set -- bash; fi
exec docker run "${options[@]}" rmt-noetic-py310:local "$@"
