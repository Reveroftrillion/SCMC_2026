#!/usr/bin/env bash
# PRACTICE ONLY: highway steering preview sweep (2026-10-01, user: sway at 100-117 km/h in lane 2).
# Sets reference_lookahead_max_m (highway references only) in low_speed.yaml for each value, runs HW980 without
# NPCs (same speeds every run) and restores the file. Do not edit low_speed.yaml while this runs.
#   bash practice/sim/lookahead_sweep.sh "14 18 22 26 30" 2 > sweep.log
set -u
cd "$(dirname "$0")/../.."
values=${1:-"14 18 22 26 30"};runs=${2:-2}
yaml=src/foscar_bringup/config/low_speed.yaml
cp $yaml /tmp/low_speed.yaml.lookahead_sweep.bak
trap 'cp /tmp/low_speed.yaml.lookahead_sweep.bak '"$yaml" EXIT
for v in $values; do
  f=$(printf '%.1f' $v)   # a float: an integer here is rejected by the DOUBLE parameter and path_follower dies
  sed -i -E "s/^(    reference_lookahead_max_m: )[0-9.]+/\1$f/" $yaml
  grep -q "reference_lookahead_max_m: $f" $yaml || { echo "could not set $f"; exit 1; }
  for k in $(seq $runs); do
    out=$(python3 practice/sim/loop_runner.py Practice3_HW980 --runs 1 --no-npc --end-s 1745 --watch-from 1110 --stop-s 25 --timeout 300 2>&1)
    dir=$(echo "$out" | sed -n 's/^results: \(.*\)\/results.jsonl/\1/p')
    echo "LOOKAHEAD $v run $k $dir $(echo "$out" | grep -o '"result": "[A-Z_:?a-z0-9]*"' | head -1)"
  done
done
