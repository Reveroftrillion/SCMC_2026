#!/usr/bin/env bash
# 관찰 전용 실행. 시뮬레이터에서 직접 운전하고, 인지·판단·RViz 는 run_competition.sh 와 똑같이 띄운다.
# 제어 송신은 항상 꺼져 있다 (drive:=false 를 맨 뒤에 강제 — drive:=true 를 넘겨도 무시).
# 조종 패킷도, 종료 시 Manual/P 인계 패킷도 보내지 않는다.
#   bash status.sh                  # 인지 + 판단 + BEV RViz, bag 기록
#   bash status.sh bag:=false       # 기록 없이
#   bash status.sh rviz:=false      # RViz 없이
# 다른 인자는 competition.launch.py 와 같다.
set -e
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
for arg in "$@"; do
  case "$arg" in drive:=*) echo "status.sh: '$arg' 무시 — 이 스크립트는 제어를 보내지 않는다 (drive:=false 고정)" >&2;; esac
done
exec ros2 launch foscar_bringup competition.launch.py rviz:=true "$@" drive:=false
