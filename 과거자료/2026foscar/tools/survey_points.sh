#!/usr/bin/env bash
set -e
survey_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$survey_repo/install/setup.bash"
cd "$survey_repo"
export PYTHONPATH="$survey_repo/src/foscar_camera_train${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m foscar_camera_train.survey "$@"
