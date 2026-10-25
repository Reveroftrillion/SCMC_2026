cd /home/foscar/Desktop/2026Morai/2026foscar-competition
for v in 50; do
  sed -i "s|^    speed_kmh: [0-9.]*  # user 2026-10-10: 40 (was a 20 trial cap)|    speed_kmh: $v.0  # user 2026-10-10: 40 (was a 20 trial cap)|" src/foscar_planning/config/missions.yaml
  grep -c "speed_kmh: $v.0  # user 2026-10-10" src/foscar_planning/config/missions.yaml
  timeout 2800 python3 practice/sim/loop_runner.py DifficultObstacle_CP13 --runs 10 --end-s 2020 > /tmp/claude-1000/-home-foscar-Desktop-2026Morai/b6a5750e-76e4-4f1d-a256-fa63bad12051/scratchpad/sweep3_entry_$v.log 2>&1
  echo "$v $(ls -td practice/sim/runs/* | head -1) $(grep -o '\"result\": \"[A-Z]*\"' /tmp/claude-1000/-home-foscar-Desktop-2026Morai/b6a5750e-76e4-4f1d-a256-fa63bad12051/scratchpad/sweep3_entry_$v.log | tr '\n' ' ')"
done
git checkout src/foscar_planning/config/missions.yaml
