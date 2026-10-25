#!/usr/bin/env bash
# Fails if anything outside practice/ still refers to practice-only simulator channels,
# and lists every port number used outside practice/ for a final manual check.
cd "$(dirname "$0")/.." || exit 1
paths=(src run_competition.sh run_low_speed.sh status.sh tools docs README.md)
bad=$(grep -rnE "practice/|CollisionData|ScenarioLoad|MoraiObjInfo|ObjectInfo|MultiEgoSetting|EgoGhost|\b(9091|9092|9095|9096|9101|9106|9111|7505|7604|7605)\b" \
      "${paths[@]}" --include='*.py' --include='*.sh' --include='*.yaml' --include='*.xml' --include='*.launch.py' 2>/dev/null \
      | grep -v '/test/')
echo "== ports referenced outside practice/ (check against the rules) =="
grep -rhoE "\b(7[0-9]{3}|9[0-9]{3})\b" src/*/config src/*/launch src/*/*/*.py run_competition.sh status.sh 2>/dev/null \
  | sort | uniq -c | sort -rn
if [ -n "$bad" ]; then
  echo "!! practice-only channel references outside practice/:"; echo "$bad"; exit 1
fi
echo "OK: no practice-only channel references outside practice/"
