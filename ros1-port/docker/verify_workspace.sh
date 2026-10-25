#!/usr/bin/env bash
# Actual catkin builds and real generated/native modules; no ROS master/UDP.
set -eo pipefail
source /opt/ros/noetic/setup.bash
cd /workspace/ros1-port
output=docker/artifacts/workspace
mkdir -p "$output"
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release -j4 > "$output/devel-build.log" 2>&1
source devel/setup.bash
set +e
python3 tools/check_noetic_environment.py --json > "$output/devel-environment.json" 2>&1
environment=$?
python3 docker/check_native_serialization.py > "$output/native-serialization.json" 2>&1
native=$?
python3 -m unittest discover -s src/ctrl_ros1/test -v > "$output/adapter.log" 2>&1
adapter=$?
python3 -m unittest discover -s tools -p test_packaging.py -v > "$output/packaging.log" 2>&1
packaging=$?
roslaunch --nodes docker/local.launch drive:=false > "$output/devel-nodes.txt" 2>&1
nodes=$?
set -e
catkin_make install -DPYTHON_EXECUTABLE=/usr/bin/python3 -j4 > "$output/install-build.log" 2>&1
set +e
bash -c 'source /opt/ros/noetic/setup.bash; source install/setup.bash; python3 tools/check_noetic_environment.py --json' \
  > "$output/install-environment.json" 2>&1
install_environment=$?
bash -c 'source /opt/ros/noetic/setup.bash; source install/setup.bash; roslaunch --nodes ctrl_bringup ctrl_competition.launch drive:=false' \
  > "$output/install-nodes.txt" 2>&1
install_nodes=$?
set -e
printf '{"devel_environment":%s,"native_serialization":%s,"adapter":%s,"packaging":%s,"devel_nodes":%s,"install_environment":%s,"install_nodes":%s}\n' \
  "$environment" "$native" "$adapter" "$packaging" "$nodes" "$install_environment" "$install_nodes" | tee "$output/exit-codes.json"
[[ $environment == 0 && $native == 0 && $adapter == 0 && $packaging == 0 && $nodes == 0 \
   && $install_environment == 0 && $install_nodes == 0 ]]
