#!/usr/bin/env bash
# Explicit opt-in: default observation only, no control output.
set -e
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# No PYTHONNOUSERSITE: obstacle_node needs the user-site scikit-learn (it died without it, 2026-09-28).
# .camera-deps (numpy 1.24) goes first so onnxruntime, sklearn and system scipy share one numpy.
export PYTHONPATH="$ws/.camera-deps:${PYTHONPATH:-}"
exec bash "$ws/run_competition.sh" camera:=true drive:=false speed_kmh:=10.0 "$@"
