#!/usr/bin/env bash
# PRACTICE ONLY: one full lap per global a_lat value (zones disabled), then a section report each.
#   bash practice/sim/sweep_a_lat.sh 3.0 3.5 4.0 4.5
# Temporarily edits src/foscar_bringup/config/low_speed.yaml and restores it on exit.
cd "$(dirname "$0")/../.." || exit 1
cfg=src/foscar_bringup/config/low_speed.yaml
cp "$cfg" /tmp/low_speed.yaml.sweep_backup
trap 'cp /tmp/low_speed.yaml.sweep_backup "$cfg"' EXIT
source /opt/ros/humble/setup.bash && source install/setup.bash
for a in "$@"; do
  cp /tmp/low_speed.yaml.sweep_backup "$cfg"
  sed -i "s/^    a_lat_mps2: .*/    a_lat_mps2: $a/; s/^    a_lat_zones: .*/    a_lat_zones: [0.0, 0.0, 0.0]/" "$cfg"
  echo "===== a_lat $a ====="
  python3 practice/sim/loop_runner.py Practice3_START --runs 1 --end-s 2165 --watch-from 0 --stop-s 25 --timeout 400 >/dev/null 2>&1
  run=$(ls -td practice/sim/runs/*/ | head -1)
  python3 -c "import json;r=[json.loads(l) for l in open('${run}results.jsonl')][-1];print(r['result'],r['drive_s'],'coll',r['collisions'],'stops',r['full_stops'])"
  python3 practice/sim/analysis/section_report.py "$(ls -t ~/foscar_logs/follow_*.csv | head -1)" "${run}results.jsonl"
  python3 practice/sim/analysis/speed_limit_check.py "$(ls -td ~/foscar_logs/bags/*/ | head -1)" 2>/dev/null
done
