#!/usr/bin/env bash
# Practice only: full laps per weather x hour (rule set: sunny/foggy, 11/13/15 h) with every camera signal
# forced red and released 5 s after the stop (signal_director.py).
#   bash practice/sim/weather_matrix.sh <runs per condition> [conditions...] -- [loop_runner args]
#   conditions: sunny_11 sunny_13 sunny_15 foggy_11 foggy_13 foggy_15 (default: all six)

ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ws"
runs=$1; shift
conds=(); extra=()
while [ $# -gt 0 ]; do [ "$1" = "--" ] && { shift; extra=("$@"); break; }; conds+=("$1"); shift; done
[ ${#conds[@]} -eq 0 ] && conds=(sunny_11 sunny_13 sunny_15 foggy_11 foggy_13 foggy_15)
source /opt/ros/humble/setup.bash; source install/setup.bash
grpcdeps=/home/foscar/Desktop/2026Morai/2026foscar/practice/collection/.deps
export PYTHONPATH="$ws/.camera-deps:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
batch="practice/sim/runs/matrix_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$batch"
for c in "${conds[@]}"; do
  w=${c%_*}; h=${c#*_}; t=$([ "$w" = sunny ] && echo 0 || echo 2)
  PYTHONPATH="$grpcdeps:$PYTHONPATH" python3 -c "
import sys;sys.path.insert(0,'practice/sim')
from morai_rpc import Morai
from sim_scenario import sim_ip
print('environment',Morai(sim_ip()+':7789').set_environment($h,$t))" || exit 1
  PYTHONPATH="$grpcdeps:$PYTHONPATH" python3 practice/sim/signal_director.py --out "$batch/${c}_signals.jsonl" \
      > "$batch/${c}_director.log" 2>&1 &
  director=$!
  python3 practice/sim/loop_runner.py Practice3_START --runs "$runs" --end-s 2165 --watch-from 0 --stop-s 60 \
      --timeout 480 --launch-arg camera:=true "${extra[@]}" > "$batch/${c}_runner.log" 2>&1
  echo "$c runner rc=$? $(grep -o 'results: .*' "$batch/${c}_runner.log")"
  kill -INT $director; wait $director 2>/dev/null
done
echo "batch $batch"
