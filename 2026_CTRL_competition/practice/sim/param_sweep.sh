#!/usr/bin/env bash
# PRACTICE ONLY: path_follower parameter sweep on HW980 without NPCs (same speeds every run). For each
# "name=value" sets that parameter in low_speed.yaml (floats only), runs, and restores the file at the end.
# Do not edit low_speed.yaml while this runs.
#   bash practice/sim/param_sweep.sh 2 lateral_ki=0.0 steer_predict_s=0.3 > sweep.log
set -u
cd "$(dirname "$0")/../.."
runs=$1;shift
yaml=src/foscar_bringup/config/low_speed.yaml
cp $yaml /tmp/low_speed.yaml.param_sweep.bak
trap 'cp /tmp/low_speed.yaml.param_sweep.bak '"$yaml" EXIT
for kv in "$@"; do
  cp /tmp/low_speed.yaml.param_sweep.bak $yaml
  for one in ${kv//,/ }; do
    name=${one%%=*};f=$(printf '%.3f' ${one#*=})   # floats: an integer is rejected by a DOUBLE parameter
    sed -i -E "s/^(    $name: )-?[0-9.]+/\1$f/" $yaml
    grep -q "^    $name: $f" $yaml || { echo "could not set $name=$f"; exit 1; }
  done
  for k in $(seq $runs); do
    out=$(python3 practice/sim/loop_runner.py Practice3_HW980 --runs 1 --no-npc --end-s 1745 --watch-from 1110 --stop-s 25 --timeout 300 2>&1)
    dir=$(echo "$out" | sed -n 's/^results: \(.*\)\/results.jsonl/\1/p')
    echo "SWEEP $kv run $k $dir $(echo "$out" | grep -o '"result": "[A-Z_:?a-z0-9]*"' | head -1)"
  done
done
