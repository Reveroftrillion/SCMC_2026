#!/usr/bin/env bash
set -e
umask 0002
mkdir -p "$HOME"
# catkin setup reads positional arguments; isolate the launched command's flags.
source_ros_environment() {
  source /opt/ros/noetic/setup.bash
  if [[ -f /workspace/ros1-port/devel/setup.bash ]]; then
    source /workspace/ros1-port/devel/setup.bash
  fi
}
source_ros_environment
unset -f source_ros_environment
exec "$@"
