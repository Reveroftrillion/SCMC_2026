#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
cd /workspace/ros1-port
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release -j4
source devel/setup.bash
python3 tools/check_noetic_environment.py --json
