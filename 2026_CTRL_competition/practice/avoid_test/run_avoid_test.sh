#!/usr/bin/env bash
# 회피 실차 시험 1회: 런치 → 모니터가 끝나면 런치를 **직접 종료** → 포트 해제 확인.
#   bash run_avoid_test.sh <speed_kmh> [tag] [rviz true/false]
#   END_S=520 LIMIT=200 bash run_avoid_test.sh 30 ped     # 구간·시간 제한 조절
# 수동으로 Ctrl-C 해도 trap 이 런치·bag record 까지 정리한다 (좀비 런치 방지).
# 로그는 이 폴더에 run_<tag>.log / mon_<tag>.log 로 남는다 (.gitignore 대상).
set +u
SPEED=$1; TAG=${2:-s$1}; RVIZ=${3:-true}
S=$(cd "$(dirname "$0")" && pwd)
cd /home/foscar/2026Morai/2026foscar
source /opt/ros/humble/setup.bash; source install/setup.bash

# 0) 이전 런치가 포트를 쥐고 있으면 시작 자체가 무의미 → 먼저 정리
for pid in $(ps -eo pid,cmd | awk '/[r]os2 launch foscar_bringup competition/{print $1}'); do kill -INT $pid 2>/dev/null; done
sleep 2
if ss -lun | grep -qE ':(9102|9104|9099) '; then
  echo "!! 포트 9102/9104/9099 가 아직 사용 중. 잔여 프로세스:"; ss -lunp | grep -E ':(9102|9104|9099) '; exit 1
fi

# 1) 런치를 별도 프로세스 그룹으로
setsid bash run_competition.sh speed_kmh:=$SPEED rviz:=$RVIZ > $S/run_$TAG.log 2>&1 &
LPID=$!
echo "런치 시작 pid=$LPID  speed=$SPEED"
cleanup() {
  trap - EXIT INT TERM
  echo "런치 정리 중..."
  kill -INT -- -$LPID 2>/dev/null; kill -INT $LPID 2>/dev/null
  for i in $(seq 1 6); do kill -0 $LPID 2>/dev/null || break; sleep 1; done
  # ros2 launch 가 INT 에 6 s 안에 안 끝나면 (FAULT 상태의 노드가 안 죽는 경우) 노드부터 TERM → KILL
  pids() { ps -eo pid,cmd | awk '/[r]os2 launch foscar_bringup competition|install\/foscar_[a-z]*\/lib|[r]os2 bag record|[r]viz2 -d/{print $1}'; }
  if [ -n "$(pids)" ]; then kill -TERM $(pids) 2>/dev/null; sleep 3; fi
  if [ -n "$(pids)" ]; then kill -KILL $(pids) 2>/dev/null; sleep 1; fi
  if ss -lun | grep -qE ':(9102|9104|9099|9302) '; then echo "!! 포트가 아직 안 풀림"; ss -lunp | grep -E ':(9102|9104|9099|9302) '; else echo "포트 해제 확인 (9102/9104/9099/9302)"; fi
}
# 러너가 Ctrl-C/터미널 종료로 죽어도 런치를 같이 끝낸다 (좀비 런치, 2026-09-20)
trap cleanup EXIT INT TERM

# 2) 모니터 (s>350, tracker FAULT 3 s, 또는 170 s 에 스스로 끝남) — Ctrl-C 도 여기서 잡혀 cleanup 으로 간다
timeout $(python3 -c "print(int(${LIMIT:-170})+30)") python3 $S/avoid_mon.py $TAG > $S/mon_$TAG.log 2>&1
echo "모니터 종료 → 런치 정리"

# 3) 런치 그룹 종료 (SIGINT 로 정상 종료 → 안 되면 TERM)
cleanup
echo "=== 결과 ==="; sed -n '/=== /,$p' $S/mon_$TAG.log
