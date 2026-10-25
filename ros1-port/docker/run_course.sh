#!/usr/bin/env bash
# Same first-lap launch, concise terminal, persistent raw/ROS/follower logs.
# No arguments means drive=false. Explicitly pass drive:=true to drive MORAI.
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$here/artifacts"
run_output=$(mktemp -d "$here/artifacts/manual-$(date +%Y%m%d-%H%M%S)-XXXXXX")
chmod g+rwx "$run_output"
mounted_output="/workspace/ros1-port/docker/artifacts/$(basename "$run_output")"
# Keep the legacy wrapper's receive-only default; the competition launch drives
# by default. An explicit caller value always wins.
launch_args=(drive:=false)
for argument in "$@"; do
  if [[ "$argument" == drive:=* ]]; then launch_args=(); break; fi
done
printf '기록 폴더: %s\n화면 반복 출력만 줄입니다. 종료: Ctrl+C 한 번, Manual/P 인계 확인.\n' "$run_output"
bash "$here/run.sh" --receive-root \
  env ROS_HOME="$mounted_output/ros-home" ROS_LOG_DIR="$mounted_output/ros-logs" \
  roslaunch --no-summary ctrl_start.launch "run_dir:=$mounted_output" "${launch_args[@]}" "$@" 2>&1 \
  | python3 "$here/console_filter.py" --log "$run_output/console.log"
