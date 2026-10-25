#!/usr/bin/env bash
# ROS1 counterpart of 2026_CTRL_competition/run_competition.sh.
# The original default drive:=true is preserved.
set -e
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
if [ -f "$ws/devel/setup.bash" ]; then
  source "$ws/devel/setup.bash"
elif [ -f "$ws/install/setup.bash" ]; then
  source "$ws/install/setup.bash"
else
  echo "Build ros1-port first: catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3" >&2
  exit 1
fi
if [ -d "$ws/.camera-deps" ]; then export PYTHONPATH="$ws/.camera-deps:${PYTHONPATH:-}"; fi
exec roslaunch ctrl_bringup ctrl_competition.launch "$@"
