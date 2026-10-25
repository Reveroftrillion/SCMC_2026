#!/usr/bin/env bash
# 기본: 센서 수신·표시만. drive:=true 를 붙이면 차량 제어 송신.
#   bash run_low_speed.sh drive:=true continuous:=true rviz:=true
set -e
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
exec ros2 launch foscar_bringup low_speed.launch.py "$@"
