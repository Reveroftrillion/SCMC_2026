#!/usr/bin/env bash
# 대회용 통합 실행. 기본으로 차가 출발한다 (drive:=true). 인자는 competition.launch.py 참조.
#   bash run_competition.sh                 # 주행 + 라이다 회피 + 터널 + 속도구간 + 카메라 7번 신호 정지
#   bash run_competition.sh rviz:=true      # + BEV
#   bash run_competition.sh drive:=false    # 제어 송신 없이 인지·판단만
#   bash run_competition.sh camera:=false   # 카메라 없이 (신호에서 서지 않음)
# 회전교차로 정차·진입 판단/추종 기본 활성. 끄려면 roundabout_control:=false
set -e
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
# YOLO(onnxruntime)는 .camera-deps 에 설치한다 (README §1). numpy 1.24 가 먼저 잡혀야 onnxruntime·sklearn·scipy 가 같이 뜬다.
if [ -d "$ws/.camera-deps" ]; then export PYTHONPATH="$ws/.camera-deps:${PYTHONPATH:-}"; fi
exec ros2 launch foscar_bringup competition.launch.py "$@"
