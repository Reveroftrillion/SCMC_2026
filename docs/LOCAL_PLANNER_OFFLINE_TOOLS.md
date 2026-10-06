# Local Planner 오프라인 개발 도구

저장소 루트에서 실행한다. Python 도구는 ROS/MORAI 없이 동작하며 NumPy, SciPy, PyYAML이 필요하다.
기존 `local_planning_core.py`, 장애물 기억, 복귀 타이머를 사용한다. Controller·차량 운동·TF를 실행하지 않는다.
원본 Global Path, 실행 YAML, Controller, 신호등/합류/GPS 음영 로직을 자동 변경하지 않는다.

## 빠른 실행

```bash
python3 -B scripts/check_local_planner_config.py \
  --path src/planning/paths/global_path.txt \
  --json reports/local_planner/preflight.json

python3 -B scripts/analyze_global_path.py \
  --csv reports/local_planner/global_path.csv

python3 -B scripts/run_local_planner_scenarios.py \
  --csv reports/local_planner/scenarios.csv

python3 -B scripts/sweep_local_planner_params.py \
  --csv reports/local_planner/sweep.csv

python3 -B -m unittest discover -s src/planning/tests -v
```

`reports/local_planner/`와 `debug_logs/`는 생성물로 git에서 제외한다.
CSV/JSON 출력이 입력 path/config와 같으면 거부한다. 기본 `global_path.txt`와 실행 기본 YAML도 보호한다.
모든 waypoint/segment index는 **0부터 시작**한다. segment i는 waypoint i→i+1이다.

## Preflight

```bash
python3 -B scripts/check_local_planner_config.py --config /tmp/local-planner-test.yaml \
  --path src/planning/paths/global_path.txt --json reports/local_planner/preflight.json
```

- `PASS`: 해당 정적 설정 검사 통과.
- `WARNING`: 빈 zone, overlap, 불가능한 일부 offset, 외부 TF 필요 등의 확인 항목.
- `ERROR`: 잘못된 타입/필수값/범위/길이/geometry. **실제 주행에 사용하면 안 된다.**
- exit code: 오류 2, PASS/WARNING만 있으면 0. WARNING 0은 미션 준비 완료를 의미하지 않는다.

zone 문법·역전·경로 범위·동일/다른 미션 overlap, finite 수치, 차량 길이/폭,
도로 corridor, horizon ≥ transition+return, offsets, cost weights, 속도/시간 상한,
LiDAR extrinsics 길이·수치를 한 번에 검사한다. 중복 YAML key도 거부한다.
offset의 음수는 우측 후보, extrinsics의 음수는 좌표/각도 방향이므로 허용한다.

index/rectangle 혼합 overlap 검사에는 `--path`가 필요하다. rectangle 경계를 선분이 통과하는 경우도 잡는다.
파일 XY를 map으로 변환하는 기본 offset `(302595, 4124145)`는 기존 `GlobalPathPlanner.py`와 같다.
이미 map 좌표로 기록한 파일에는 `--map-offset 0 0`을 사용한다. 실제 경로 loader와 좌표계를 일치시켜야 한다.
zone overlap은 WARNING이다. 현재 노드에서는 dynamic zone이 static zone보다 우선한다.

`lidar_xyz_rpy: []`는 외부 TF 공급자가 필요하다는 WARNING이다.
실제 외부 공급자를 사용하면 `--external-lidar-tf`로 선언할 수 있지만 live TF 존재/취득 시각 정합은 검사하지 못한다.
extrinsics와 외부 공급자를 동시에 선언하면 중복 TF 가능성을 경고한다.

## Global Path 품질

```bash
python3 -B scripts/analyze_global_path.py --path src/planning/paths/global_path.txt \
  --config src/planning/config/local_planner.yaml --closure-distance 1.0 \
  --csv reports/local_planner/global_path.csv
```

waypoint 수·전체 길이·spacing 평균/중앙값, 전체 동일 좌표 수와 연속 중복 선분 수,
near-duplicate/짧은 선분, yaw 변화, 이산 곡률, closure gap, 시작/끝 좌표를 출력한다.
셋째 열은 기존 Global loader와 동일하게 사용하지 않는다. 좌표 출력은 파일 XY다.
closure는 시작/끝 거리가 기준 이내라는 뜻이며, 마지막 waypoint에서 첫 waypoint로 Local reference를 감아 이어붙이지 않는다.

곡률은 세 점의 외접원 곡률 `2|cross|/(a*b*c)`로 계산한다. 중복/완전 역전의 정의 불가능한 곡률은 별도 표시한다.
reference spike 기준과 candidate 곡률 기준을 구분한다. 이산 곡률과 cubic spline 곡률은 서로 다를 수 있다.
기본 실행은 **모든 index**에서 실제 `core.reference()`와 남은 forward 길이를 검사한다.
그 결과가 `local_spline_risks`이며 reason을 함께 저장한다. 차량 heading/장애물/모든 candidate 검증을 대체하지 않는다.
`--skip-reference-check`는 기하 통계만 빠르게 볼 때 사용한다.

CSV에는 모든 segment와 위험 reference 행이 들어간다. 같은 이름의 `.summary.json`에는 전체 통계와 index 목록이 들어간다.
콘솔의 긴 목록은 30개까지만 표시한다. 품질 문제가 있어도 분석 성공이면 exit 0, 입력/설정 오류면 2다.

현재 기본 파일/기본 설정으로 확인한 예시:

| 지표 | 결과 |
|---|---:|
| waypoint | 4,430 |
| 길이 | 약 2,184.612m |
| 연속 중복 선분 | 38 |
| 0.05m 미만의 비중복 선분 | 5 |
| Local reference/forward 길이 위험 index | 414 |

414는 현재 검사 설정에서 발생한 reference 위험 수이며 실제 주행 실패 횟수가 아니다.
폐곡선 파일의 끝에서도 현재 planner가 경로를 wrap하지 않아 길이 부족을 보고할 수 있다.
원본을 수정하거나 위험을 숨기도록 상한을 자동 완화하지 않는다.

## Synthetic scenario

```bash
python3 -B scripts/run_local_planner_scenarios.py --csv reports/local_planner/scenarios.csv
python3 -B scripts/run_local_planner_scenarios.py \
  --scenario center_obstacle --scenario return_to_global
```

직선/없음, 중앙, 좌측, 우측, 다중, 전체 차단, 좁은 corridor, 곡선, 중복점,
급곡률, 장애물 소실, Global 복귀의 12개 scenario/18개 frame을 포함한다.
장애물은 **map 좌표의 `(x,y,radius)`**다. fixture의 차량/도로/장애물 위치는 합성값이다.
obstacle_disappears는 기억 유지→만료→복귀, return_to_global은 측방향 오차→연속 정렬 완료를 검사한다.

출력: 유효 후보 수, 선택 offset/cost, 선택 경로 최소 clearance, 전체 후보 중 최소 clearance,
곡률/조향 변화율 peak, stop, state/expected state, 탈락 이유, PASS/FAIL.
clearance는 차량 footprint·margin까지 반영한 여유 거리다. 장애물 없을 때 `inf`는 정상이다.
차단 정지/HOLD가 예상된 scenario는 그 동작을 하면 PASS다. 잘못된 실행 설정은 CONFIG_ERROR로 표시한다.
FAIL/CONFIG_ERROR exit 1, 입출력 오류 2, 전부 예상 동작이면 0이다.

state는 **오프라인 harness의 예상 상태**다. 실제 ROS `/local_plan/state` 발행을 검증한 결과가 아니다.
중심 복귀 우선 선택과 완료 타이머를 재현하며, 실제 ROS adapter 테스트는 기존 별도 테스트가 담당한다.

## Parameter sweep

기본은 한 번에 한 파라미터만 바꾸며 baseline을 포함해 24개 설정, 기본 3개 scenario에서 72행을 만든다.
지원값은 safety_margin, offsets, transition_length, return_length, horizon, road_half_width,
max_curvature, max_steering_rate_per_m이다. 자동 최적화나 실행 YAML 저장은 하지 않는다.

사용자 grid 예시:

```yaml
safety_margin: [0.1, 0.3, 0.6]
offsets:
  - [-2.5, 0.0, 2.5]
  - [-3.0, -2.5, 0.0, 2.5, 3.0]
transition_length: [10.0, 12.0]
return_length: [10.0, 12.0]
horizon: [24.0, 30.0]
```

```bash
python3 -B scripts/sweep_local_planner_params.py --grid /tmp/local-grid.yaml \
  --mode cartesian --scenario center_obstacle --max-runs 500 \
  --clearance-warning 0.25 --csv reports/local_planner/custom_sweep.csv
```

`--mode one-at-a-time`이 기본이며 cartesian은 조합을 모두 비교한다. 실행 수 상한을 먼저 검사한다.
선택하지 않으면 center_obstacle/all_blocked/curved_road를 사용한다.
fixture의 narrow corridor 설정은 sweep에서 덮어쓰지 않고 **실제 sweep 값**으로 실행한다.

| assessment | 의미 |
|---|---|
| CONFIG_ERROR | horizon/geometry 등 설정 자체가 실행 불가능 |
| HOLD | reference 또는 입력 오류 |
| ALL_REJECTED | 후보 전부 탈락; 이유별 개수 확인 |
| LOW_CLEARANCE | 선택 후보의 clearance가 보고 기준 미만 |
| NEAR_CURVATURE_LIMIT | 선택 곡률이 설정 상한의 80% 이상 |
| NEAR_STEERING_RATE_LIMIT | 거리당 조향 변화율이 설정 상한의 80% 이상 |
| OK | 해당 보고 기준에 걸리지 않음 |

조향 변화율은 core와 같은 reference 거리 좌표로 계산한다. 실제 actuator의 시간당 조향 속도가 아니다.
infeasible 설정/차단은 비교 결과로 남기고 sweep을 계속한다. 이 결과만으로 최적값이나 주행 허가를 결정하지 않는다.

## 팀원용 Diagnostic collector

Ubuntu ROS PC에서 ROS 환경을 source한 뒤 한 번 실행한다:

```bash
bash scripts/collect_local_planner_debug.sh --config /tmp/local-planner-test.yaml \
  --timeout 8 --lidar-frame velodyne
```

`debug_logs/YYYYMMDD_HHMMSS_XXXXXX/`에 branch/commit/status, package 경로, 지정 YAML와 hash,
관련 노드의 runtime rosparam, node/topic 목록, topic 주기, LocalPlan/ControlCmd sample,
map→LiDAR TF, manifest와 summary.tsv를 저장한다. 충돌 방지를 위해 시간 뒤에 임의 suffix가 붙는다.
`--output-root`로 저장 위치를 바꿀 수 있다. timestamp는 수집 PC의 현지 시각이다.

각 명령은 GNU timeout으로 제한하며 ROS probe는 동시에 실행한다. ROS/topic/command가 없어도
나머지를 계속 수집하고 해당 파일에 `unavailable`을 기록한다. 시간 창 내 주기/TF 출력이 있으면
bounded sample로 기록한다. YAML 복사본과 runtime rosparam을 모두 보내야 실제 적용값을 판단할 수 있다.
snapshot은 원자적이지 않으며 실험 중 여러 시점의 정보가 섞일 수 있다.
collector는 rosbag recording이나 차량 명령 송신을 시작하지 않는다.

현재 Windows sandbox에서는 Git Bash가 signal pipe 권한 오류로 실행되지 않아 collector를 실행 검증하지 못했다.
POSIX Bash/GNU timeout 환경용 missing-ROS/fake-ROS 회귀 검사 2개를 추가했으며 Windows에서는 skip한다.

## 실제 시험 설정과 결과 반영

```bash
cp src/planning/config/local_planner_test.yaml /tmp/local-planner-test.yaml
nano /tmp/local-planner-test.yaml
python3 -B scripts/check_local_planner_config.py --config /tmp/local-planner-test.yaml \
  --path src/planning/paths/global_path.txt
```

TODO_MEASURE geometry/extrinsics는 실측값으로 채운다. zones도 실제 코스로 채워야 한다.
YAML namespace/구조는 기존 launch와 같지만 **미완성 템플릿은 의도적으로 ERROR/startup reject**된다.
외부 TF를 검증한 경우 extrinsics만 `[]`로 바꿀 수 있다. 나머지 수치는 기존 tuning seed다.
offset/속도/approach/곡률 제한도 현장 응답에 맞춰 확인한다.

오류를 해결한 복사본은 기존 launch의 `local_planner_config:=/tmp/local-planner-test.yaml`로 전달한다.
시험 담당자는 [LOCAL_PLANNER_TEST_RESULTS.md](LOCAL_PLANNER_TEST_RESULTS.md)에 phase별 결과와
diagnostic 폴더/스크린샷/rosbag 경로를 기록한다. 결과를 받으면 먼저 YAML·commit·runtime params를 맞춘 뒤
문제를 오프라인 fixture나 회귀 테스트로 재현하고 Local 내부 변경을 검토한다.
