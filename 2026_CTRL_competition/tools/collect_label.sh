#!/usr/bin/env bash
# One condition per invocation: collect until Ctrl+C, then label that session.
set -e
camera_train_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$camera_train_repo/install/setup.bash" ]]; then
    echo 'ROS 2 Humble과 워크스페이스 빌드가 필요합니다.' >&2
    echo '레포에서 source /opt/ros/humble/setup.bash && colcon build --symlink-install 을 실행하세요.' >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
source "$camera_train_repo/install/setup.bash"
cd "$camera_train_repo"
# Read the new helper from source without requiring another colcon build.
export PYTHONPATH="$camera_train_repo/src/foscar_camera_train${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -u -m foscar_camera_train.collect_label "$@"
