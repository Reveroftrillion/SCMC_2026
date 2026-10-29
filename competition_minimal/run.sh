#!/usr/bin/env bash
set -euo pipefail
package_dir="$(cd "$(dirname "$0")" && pwd)"
workspace_dir="$(cd "$package_dir/.." && pwd)"
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo 'ROS Noetic not found at /opt/ros/noetic' >&2; exit 1
fi
set +u
source /opt/ros/noetic/setup.bash
if [[ -f "$workspace_dir/devel/setup.bash" ]]; then
  source "$workspace_dir/devel/setup.bash"
fi
if ! python3 -c 'from simul_msgs.msg import VehicleStatus, ControlCmd; from morai_msgs.msg import GPSMessage' 2>/dev/null; then
  (cd "$workspace_dir" && catkin_make -j2 --only-pkg-with-deps simul_msgs morai_msgs -DPYTHON_EXECUTABLE=/usr/bin/python3)
  source "$workspace_dir/devel/setup.bash"
fi
set -u
# This package lives beside src/, outside catkin's source-space crawl.  Add
# the package itself, not the workspace root, so rospack can find it without a
# duplicate src/ symlink or discovering unrelated nested packages twice.
export ROS_PACKAGE_PATH="$package_dir:${ROS_PACKAGE_PATH:-}"
found_package="$(rospack find competition_minimal)"
if [[ "$(cd "$found_package" && pwd -P)" != "$(cd "$package_dir" && pwd -P)" ]]; then
  echo "Wrong competition_minimal package found: $found_package" >&2; exit 1
fi
chmod +x "$package_dir"/scripts/*.py
python3 "$package_dir/scripts/preflight.py" "$@"
exec roslaunch competition_minimal competition.launch "$@"
