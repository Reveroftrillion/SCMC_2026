#!/usr/bin/env bash
# RViz(BEV) 만 따로 띄운다. 주행 스택(run_competition.sh / status.sh / 반복 시험)을 껐다 켜도
# 이 창은 그대로 두면 되고, 토픽이 다시 나오면 알아서 이어서 그린다.
# 스택은 rviz:=false 로 띄울 것 (기본값). 끌 때는 이 터미널에서 Ctrl-C.
set -e
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
ros2 run foscar_planning bev_node --ros-args -p station_ticks:=${STATION_TICKS:-false} &
bev=$!
trap 'kill -INT $bev 2>/dev/null' EXIT
rviz2 -d "$ws/src/foscar_planning/rviz/bev.rviz"
