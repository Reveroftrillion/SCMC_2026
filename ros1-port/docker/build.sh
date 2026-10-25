#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec docker build --progress=plain -t rmt-noetic-py310:local "$here" "$@"
