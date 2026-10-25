# Source from the host's ~/.bashrc. Keep the Python 3.10 catkin overlay in Docker.
_CTRL_ROS1_REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)

roslaunch() {
  if [[ ${1:-} == ctrl_start.launch ]]; then
    shift
    command bash "$_CTRL_ROS1_REPO/ros1-port/docker/run.sh" --receive-root \
      roslaunch ctrl_start.launch "$@"
  else
    command roslaunch "$@"
  fi
}
