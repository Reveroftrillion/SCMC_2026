# SCMC 2026

2026 대학생 창작 모빌리티 경진대회 AI융합자율주행 부문용 ROS1 프로젝트.
현재 개발 기준은 `feat/local-planner`이며, Local Planner의 코드·오프라인 검증을 진행했다.
**실제 MORAI 검증과 main merge 조건은 아직 완료되지 않았다.**

## 현재 개발 상태

✅ 구현 및 확인 완료 · 🧪 코드 구현 완료, MORAI 검증 필요 · 🟠 일부 구현 · ❌ 미구현 · 📏 실제 측정값 필요

✅의 확인 범위는 아래 근거에 한정한다. 자동 테스트 통과를 실제 주행 성공으로 표시하지 않는다.

| 항목 | 상태 | 현재 코드와 확인 범위 |
|---|---|---|
| MORAI UDP | 🧪 코드 구현 완료, MORAI 검증 필요 | parser/receiver/송신 watchdog 구현; 기존 UDP 자동 검사 13개 확인. 실제 패킷 규격·네트워크 재검증 필요 |
| GPS | 🧪 코드 구현 완료, MORAI 검증 필요 | NMEA 수신·WGS84→UTM52N pose 생성. fresh Vehicle Status yaw 필요 |
| IMU | 🧪 코드 구현 완료, MORAI 검증 필요 | `/imu` 수신 구현; 현재 위치 추정에 융합하지 않음 |
| Vehicle Status | 🧪 코드 구현 완료, MORAI 검증 필요 | Ego 배치 parser 구현. Competition 배치 호환은 미확정; layout 확인 플래그 필요 |
| Global Path | 🧪 코드 구현 완료, MORAI 검증 필요 | 고정 파일 로딩·yaw/곡률 계산. 중복/짧은 선분과 reference 위험 잔존 |
| Control Path | 🧪 코드 구현 완료, MORAI 검증 필요 | 현재 pose 기반 부분 경로 생성; 0.5m 간격 가정으로 40점, 약 20m |
| Controller | 🧪 코드 구현 완료, MORAI 검증 필요 | Pure Pursuit/heading 보정·속도 제어·LocalPlan 선택/정지 구현; 최신 C++ 빌드와 차량 응답 검증 필요 |
| YOLO | 🧪 코드 구현 완료, MORAI 검증 필요 | custom weight 포함, CPU/conf=0.4, 최고 confidence class 발행. 대상 신호 선택/미검출 처리 미완성 |
| Lane Detection | 🟠 일부 구현 | `lanenet.py`의 실제 방식은 HSV/warp/slide-window; 카메라 보정·freshness 검증 필요 |
| LiDAR Detection | 🧪 코드 구현 완료, MORAI 검증 필요 | static DBSCAN·timestamp/frame·빈 scan·overflow 보호. ROI/TF/검출 품질 실검증 필요 |
| Local Planner Core | ✅ 구현 및 확인 완료 | ROS 없는 수학·후보/충돌/reference 방어를 합성 입력으로 확인; 차량 운동 검증과 별개 |
| Local Planner 자동 테스트 | ✅ 구현 및 확인 완료 | **68개 중 66개 PASS, Windows에서 Bash 관련 2개 SKIP** |
| Local Planner ROS 연동 | 🧪 코드 구현 완료, MORAI 검증 필요 | `/local_plan`과 Controller 연결·ROS smoke 구현. 최신 ROS 실행은 미검증 |
| Local Planner RViz | 🧪 코드 구현 완료, MORAI 검증 필요 | 경로·후보·장애물·상태 marker 구현; 실제 화면/TF 정합 미검증 |
| Static Obstacle | 🟠 일부 구현 | 회피/차단/기억/복귀 합성 확인; 실제 zone과 실주행 tuning 필요 |
| Dynamic Obstacle | 🟠 일부 구현 | 중심 경로 차단 정지/기본 10km/h 진행 정책; tracking/예측 없음 |
| Traffic Light | 🟠 일부 구현 | YOLO 연결. Controller 정지 이벤트는 sentinel만 남아 있음; 최종 미션 통합은 별도 개발 과제 |
| GPS Shadow | 🟠 일부 구현 | 차선 조향/45km/h/시간 기반 index 증가; 추측항법·안정적 복구 미완성 |
| Merging | 🟠 일부 구현 | legacy 함수 존재, `in_merging_zone()`은 false; 시간 후 장애물 무시 로직 재설계 필요 |
| 전체 코스 통합 | ❌ 미구현 | 현재 branch의 모든 미션을 통합한 주행·검증과 Mission Manager 미완료 |
| 실제 시험 설정 | 📏 실제 측정값 필요 | zone/센서 TF/current_pose 기준점/차량 치수/도로 폭 미확정 |

## 구조와 데이터 흐름

| 경로 | ROS package / 역할 |
|---|---|
| `src/MORAI_UDP_NetworkModule` | `morai_udp`: GPS/IMU/카메라/차량 상태 수신, `/control_cmd` UDP 송신 |
| `src/MORAI-ROS_morai_msgs`, `src/simul_msgs` | MORAI 및 프로젝트 메시지; `simul_msgs/LocalPlan` 포함 |
| `src/gps`, `src/camera`, `src/lidar` | `gps`, `camera`, `lidar_object_detection`: 위치·영상·DBSCAN |
| `src/planning` | `planning`: Global/Control/Local 경로와 RViz |
| `src/control`, `src/main` | `control`, `main`: 제어와 전체 launch |
| `scripts/`, `docs/` | ROS 없는 개발 도구, 알고리즘/시험 문서 |

```text
MORAI GPS → /gps ───────────────┐
MORAI Status → /vehicle_status ─┴→ gps_to_utm → /current_pose
global_path.txt → PathPlanner 내부 GlobalPathPlanner → /global_path
                                   + /current_pose → /control_path
LiDAR → /velodyne_points → static DBSCAN → /obstacle_info_static
  + /global_path + /current_pose + /curr_idx + TF → Local Planner → /local_plan
/control_path + /local_plan + /vehicle_status → Controller → /control_cmd → UDP → MORAI
Front camera → /camera/front/image/compressed → YOLO / Lane Detection
```

`simulator.launch`는 `PathPlanner.py`를 실행한다. `GlobalPathPlanner.py`는 별도 node가 아닌 내부 클래스다.
`object_detection_crossing.launch`는 실제로 static/car 검출기를 실행한다.

## 새 Ubuntu ROS PC 준비와 빌드

Ubuntu 20.04 / ROS Noetic의 Bash 터미널을 기준으로 한다. ROS 설치와 apt 저장소 설정을 먼저 준비한다.
아래 명령은 **저장소 루트**에서 실행한다. `morai_msgs`는 저장소의 `src/MORAI-ROS_morai_msgs`로 제공한다.

```bash
source /opt/ros/noetic/setup.bash
sudo apt-get update
sudo apt-get install -y build-essential python3-rosdep python3-pip \
  python3-numpy python3-scipy python3-yaml python3-pyproj python3-rospkg python3-opencv \
  ros-noetic-cv-bridge ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  ros-noetic-velodyne-pointcloud ros-noetic-rviz

# 새 PC에서 rosdep 초기화가 안 된 경우에만 실행
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic

# YOLO 실행 의존성: 팀의 Noetic/Python 환경에 호환되는 버전을 사용
python3 -m pip install --user torch ultralytics
python3 -c "import numpy, scipy, yaml, pyproj, rospkg, cv2, torch, ultralytics; print('Python imports OK')"

catkin_make -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
rospack find morai_udp
rospack find morai_msgs
rospack find planning
rospack find control
rospack find lidar_object_detection
```

Python dependency 버전은 아직 lock되어 있지 않다. ROS 노드가 쓰는 `/usr/bin/python3` 환경에서도 import를 확인한다.
rosdep 오류는 설치 완료로 간주하지 말고 해결한다. 모델은 `src/camera/models/1027_40epoch.pt`에 있다.
메시지 변경 시 모든 관련 노드를 종료하고 전체 catkin 빌드 후 재시작한다.

## Local Planner 구현 요약

- reference sanitization: duplicate/near-duplicate 제거, spline 입력·길이·yaw·미분·곡률 검증.
- Cartesian→Frenet 투영과 quintic lateral candidate, 차량 footprint/segment collision 검사.
- road corridor·curvature·steering-rate-per-m 제한, candidate cost와 continuity/hysteresis.
- obstacle memory, 안전한 중심 후보 복귀, `RETURN_TO_GLOBAL → NORMAL`, 오류 시 `HOLD`.
- stale/invalid LocalPlan fail-safe, 구간 접근 속도 상한, throttled 진단·marker.

Local disabled이면 기존 `/control_path`를 사용한다. enabled일 때 inactive plan은 Global 경로를 유지하며
양수 속도 상한만 줄 수 있다. active/valid plan은 Local 경로를 사용하고, stop/stale/invalid이면
`accel=0, brake=1, steering=0`을 요청한다. LocalPlan timeout 기본값은 0.5초다.
구간 설정이 모두 비면 `NORMAL`, active=false, 추가 속도 제한 없음이 정상이다.
시간 기반 동적 예측·최적 속도 궤적은 구현하지 않았다.
상세 알고리즘과 TF 계약은 [LOCAL_PLANNING.md](docs/LOCAL_PLANNING.md)를 따른다.

## 개발 PC 검사와 오프라인 도구

ROS/MORAI 없는 PC에서도 NumPy·SciPy·PyYAML이 있으면 실행할 수 있다. Windows에서는 `python3` 대신 `python`을 사용한다.
다음 순서로 실행한다. 처음 네 도구의 기본 입력 설정은 `src/planning/config/local_planner.yaml`이다.
코드 블록은 Bash 문법이다. PowerShell에서는 명령을 한 줄로 입력하고 unit test 환경변수는 먼저
`$env:OPENBLAS_NUM_THREADS='1'`로 설정한다.

```bash
# 1. unit tests
OPENBLAS_NUM_THREADS=1 python3 -B -m unittest discover -s src/planning/tests -v

# 2. YAML preflight
python3 -B scripts/check_local_planner_config.py \
  --path src/planning/paths/global_path.txt --json reports/local_planner/preflight.json

# 3. Global Path 품질
python3 -B scripts/analyze_global_path.py --csv reports/local_planner/global_path.csv

# 4. synthetic scenarios
python3 -B scripts/run_local_planner_scenarios.py --csv reports/local_planner/scenarios.csv

# 5. parameter sensitivity
python3 -B scripts/sweep_local_planner_params.py --csv reports/local_planner/sweep.csv
```

| 도구 | 목적 / 입력 | 위 명령의 출력 |
|---|---|---|
| `scripts/check_local_planner_config.py` | YAML preflight, optional path; `--config`로 실제 시험 YAML 지정 | `preflight.json`, PASS/WARNING/ERROR; ERROR exit 2 |
| `scripts/analyze_global_path.py` | 원본 XY 경로·Local 설정 기반 기하/reference 품질 분석; `--path`, `--config` 지원 | `global_path.csv`, `global_path.summary.json` |
| `scripts/run_local_planner_scenarios.py` | Local core 합성 시나리오; `--config`, 반복 `--scenario` 지원 | `scenarios.csv`: 후보 수/offset/cost/clearance/stop/예상 state |
| `scripts/sweep_local_planner_params.py` | 기본 grid 또는 `--grid` YAML; `--mode one-at-a-time`/`cartesian` | `sweep.csv`: 전체 탈락/작은 clearance/곡률·조향 변화율 비교 |
| `scripts/collect_local_planner_debug.sh` | Ubuntu live ROS·git·지정 YAML; 아래 수집 명령 사용 | `debug_logs/YYYYMMDD_HHMMSS_XXXXXX/`, 진단 파일·`summary.tsv` |

생성 보고서는 `reports/local_planner/`에 저장되고 git에서 제외된다. 도구는 원본 경로나 실행 YAML을 고치지 않는다.
상세 옵션·exit code·CSV 해석은 [LOCAL_PLANNER_OFFLINE_TOOLS.md](docs/LOCAL_PLANNER_OFFLINE_TOOLS.md)에 있다.

### 현재 오프라인 결과

| 검사 | 확인된 결과 |
|---|---|
| Local 자동 테스트 | **68개 중 66개 PASS, Windows에서 Bash 관련 2개 SKIP** |
| 기본 config preflight | ERROR 0 / WARNING 3: static zone 비어 있음, dynamic zone 비어 있음, LiDAR 외부 TF 또는 실측 extrinsics 필요 |
| Global Path | 연속 중복 segment 38 / 매우 짧은 segment 5 / Local reference 위험 index 414 |
| reference 위험 이유 | spline curvature 관련 363 / reference 길이 부족 51 |
| synthetic scenario | 12 scenarios / 18 frames, 현재 예상 동작 확인 |
| 기본 sweep | 24개 설정 × 3개 scenario = 72행 |

**원본 `global_path.txt`는 수정하지 않았다.** 결과는 현재 기본 설정의 오프라인 검사이며 MORAI 실제 주행 성공을 의미하지 않는다.
위험 index는 실제 실패 횟수가 아니다. 폐곡선 끝의 reference wrap도 현재 구현되지 않았다.
Windows Bash 실행 제한으로 collector 실행 검사는 남아 있다. 최신 C++/ROS smoke/MORAI/RViz는 미검증이다.

## 📏 실제 측정/확정 필요

**TUNING DEFAULT = 코드의 조정용 기본값, MEASURED = 해당 차량·센서·코스에서 확인한 값.**
현재 기본값을 MEASURED로 간주하지 않는다. 시험용 [local_planner_test.yaml](src/planning/config/local_planner_test.yaml)의
`TODO_MEASURE`는 실측 후 채운다. 채우기 전에는 preflight ERROR와 startup reject가 정상이다.

| 확정할 항목 | 현재 상태 / 팀장이 제공할 정보 |
|---|---|
| `static_zones`, `dynamic_zones` | 빈 배열. 실제 포함 범위 index(start/end, 0-based) 또는 map/UTM rectangle |
| `lidar_xyz_rpy` | 빈 배열. `[x,y,z,roll,pitch,yaw]` m/rad 실측 또는 검증된 외부 TF 공급자 |
| `current_pose` 기준점 | 차량에서의 물리적 기준점 미확정; front/rear/extrinsics를 같은 기준으로 측정 |
| `vehicle_width`, `vehicle_front`, `vehicle_rear` | TUNING DEFAULT 1.9 / 3.5 / 1.0m; 범퍼까지의 실측값 필요 |
| `wheelbase` | Local/Controller 코드 기본 3.0m. 실제 차량 모델과 일치 확인; Local YAML만 바꾸면 Controller 상수는 바뀌지 않음 |
| `road_half_width` | TUNING DEFAULT 4.5m; 허용 주행 폭·차선 침범 조건 확인 |
| LiDAR ROI / DBSCAN / obstacle size | YAML·검출기 기본값은 tuning seed; 지면·자체영역·검출 크기를 실제 cloud로 확인 |
| static obstacle safe speed / approach distance | TUNING DEFAULT 20km/h / 15m; 실제 제동거리·추종오차로 조정 |

## MORAI 시험 준비와 실행

먼저 아래 정보를 저장한다. dirty worktree이면 사용한 diff도 함께 보관한다.

```bash
git branch --show-current
git rev-parse HEAD
git status
```

현재 [network.yaml](src/MORAI_UDP_NetworkModule/config/network.yaml)의 **실행 기본값**은
Ego Status 수신 9111, 제어 목적지 9093/source 9094, GPS 1111, IMU 1112,
카메라 Front/Left/Right 9291/9293/9295다. LiDAR launch 기본은 2368/rpm=600이다.
Competition Status 9099 등 저장된 항목은 현재 독립 수신 node가 실행되지 않는다.
MORAI 센서 Destination IP는 ROS PC 주소, `morai_ip`는 MORAI PC 주소다. `bind_ip=0.0.0.0`을 센서 목적지로 쓰지 않는다.
패킷 field/단위/layout을 확인한 환경에서만 `status_layout_confirmed:=true`를 사용한다.
[UDP_CONNECTION.md](docs/UDP_CONNECTION.md)의 오래된 9082 포트 예시는 현재 YAML의 9111과 다르므로 현재 YAML을 기준으로 확인한다.

Phase 0부터 진행한다. 실측 준비는 송신을 끄고 센서/TF/RViz부터 확인한다.

```bash
# Phase 0: Ubuntu 빌드 후 isolated ROS smoke (MORAI UDP 송신/실제 driver 없음)
source /opt/ros/noetic/setup.bash
catkin_make -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
python3 -B src/planning/tests/ros_local_planner_smoke.py

# Phase 2: 기본 empty zones 설정 준비
cp src/planning/config/local_planner.yaml /tmp/local-planner-empty.yaml

# Phase 3~10: 실측 시험 설정 복사본 준비
cp src/planning/config/local_planner_test.yaml /tmp/local-planner-test.yaml
nano /tmp/local-planner-test.yaml
python3 -B scripts/check_local_planner_config.py --config /tmp/local-planner-test.yaml \
  --path src/planning/paths/global_path.txt

# 실제 MORAI IP 입력 후, 센서/TF 정합을 송신 OFF로 먼저 확인
read -rp "MORAI IP: " MORAI_IP
roslaunch main simulator.launch morai_ip:="$MORAI_IP" \
  status_layout_confirmed:=true enable_control:=false \
  enable_local_planner:=true rviz:=true \
  local_planner_config:=/tmp/local-planner-test.yaml
```

한 phase가 끝나면 launch를 종료하고 아래 명령 중 해당 모드로 재시작한다. 동일 노드/UDP 포트를 중복 실행하지 않는다.
실제 주행 명령은 상태 layout과 해당 phase의 설정을 확인한 뒤 사용한다.

```bash
# Phase 1: Local OFF baseline
roslaunch main simulator.launch morai_ip:="$MORAI_IP" \
  status_layout_confirmed:=true enable_control:=true enable_local_planner:=false

# Phase 2: Local ON + empty zones
roslaunch main simulator.launch morai_ip:="$MORAI_IP" \
  status_layout_confirmed:=true enable_control:=true enable_local_planner:=true rviz:=true \
  local_planner_config:=/tmp/local-planner-empty.yaml

# Phase 3~10: 실측 zone/geometry/TF 적용
roslaunch main simulator.launch morai_ip:="$MORAI_IP" \
  status_layout_confirmed:=true enable_control:=true enable_local_planner:=true rviz:=true \
  local_planner_config:=/tmp/local-planner-test.yaml
```

별도 driver/rosbag이 `/velodyne_points`를 발행하면 해당 명령에 `start_lidar_driver:=false`를 추가한다.
이 옵션은 DBSCAN 검출기를 끄지 않는다. 기본 launch 값은 control 송신 OFF, Local OFF, RViz OFF, LiDAR driver ON이다.

### Phase 0~10 목적과 성공 기준

| Phase | 시험 | 성공 기준 |
|---|---|---|
| 0 | Ubuntu catkin build + ROS smoke | 빌드·isolated smoke PASS, 준비 설정의 ERROR 해결 |
| 1 | Local Planner OFF baseline | 기존 `/control_path` 추종과 속도/조향 기준 기록 |
| 2 | Local ON + empty zones | NORMAL/inactive/추가 속도 제한 없음, 기존 경로 추종 유지 |
| 3 | Static zone + no obstacle | 정상 입력/TF로 HOLD 없음, center/Global 경로 유지와 zone speed cap 확인 |
| 4 | Center obstacle | valid 측방향 후보 선택, 실제 충돌/도로 이탈 없음 |
| 5 | Left obstacle | 충돌 없는 우측 또는 center 선택, 실제 clearance 확인 |
| 6 | Right obstacle | 충돌 없는 좌측 또는 center 선택, 실제 clearance 확인 |
| 7 | All candidates blocked | stop=true, accel=0/brake=1/steering=0, 실제 정지 확인 |
| 8 | LiDAR input loss | 활성 구간에서 stale→HOLD/full brake, 복구 후 재검사 |
| 9 | TF failure | fresh LiDAR는 유지하면서 scan 시각 TF 실패→HOLD/full brake |
| 10 | RETURN_TO_GLOBAL | 회피 후 중심 복귀·연속 정렬 완료→NORMAL/inactive→Global 추종 |

입력 loss/TF 실패 검사는 실제 zone이 활성이고 다른 입력이 정상인 조건에서 수행한다.
상세 입력 조건·측정값·증거는 [LOCAL_PLANNER_TEST_RESULTS.md](docs/LOCAL_PLANNER_TEST_RESULTS.md)에 기록한다.

### RViz와 진단

RViz Fixed Frame은 `planning_origin`. 후보는 파랑=유효, 빨강=탈락, 초록=선택이다.

| 표시 | 토픽 |
|---|---|
| Global / Control / 선택 Local 경로 | `/viz_planner/viz_global_path`, `/viz_planner/viz_control_path`, `/viz_planner/viz_local_path` |
| 후보·offset·cost·clearance·탈락 이유 | `/static_obstacle_avoidance_planner/candidates` |
| 장애물 / state·HOLD 이유 | `/static_obstacle_avoidance_planner/obstacles`, `/static_obstacle_avoidance_planner/state_marker` |
| 차량 위치 / LiDAR / boxes | `/viz_planner/viz_current_pose`, `/velodyne_points`, `/bounding_box_static` |

```bash
rostopic hz /current_pose
rostopic hz /obstacle_info_static
rostopic echo /local_plan/state
rostopic echo /local_plan/stop
rostopic echo /local_plan/speed_limit_kmh
rostopic echo /control_cmd
rosrun tf tf_echo map velodyne
```

상태 로그는 `[LOCAL CONFIG]`, `[LOCAL REFERENCE]`, `[LOCAL PLANNER]`, `[LOCAL HOLD]`로 구분된다.
NORMAL에서 선택 Local Path가 비는 것은 정상이다. `/local_plan`에는 selected offset/clearance 전용 필드가 없으므로 marker/로그와 함께 기록한다.

### 담당자가 반드시 남길 증거

branch·commit·git status/diff, 실제 YAML, static zone, `lidar_xyz_rpy`, vehicle geometry,
RViz screenshot, terminal log, collector 결과, 가능하면 rosbag, phase별 PASS/FAIL과 실패 재현 조건을 보관한다.

```bash
bash scripts/collect_local_planner_debug.sh --config /tmp/local-planner-test.yaml \
  --timeout 8 --lidar-frame velodyne
```

Phase 2에서는 `--config /tmp/local-planner-empty.yaml`을 사용한다. collector는 runtime rosparam·topic 주기·plan/cmd sample·TF·node/topic 목록을 저장한다.
없는 항목은 unavailable로 기록하며 rosbag/스크린샷을 자동 생성하지 않는다.
rosbag을 별도로 기록한다면 `/current_pose`, `/vehicle_status`, `/gps`, `/velodyne_points`, `/obstacle_info_static`,
`/global_path`, `/control_path`, `/curr_idx`, `/local_plan`, `/control_cmd`, `/tf`, `/tf_static`을 포함한다.

## 알려진 문제와 남은 작업

- Global Path duplicate/short segment와 Local reference 위험 index가 남아 있다. Local 방어가 원본 Global 제어 경로까지 정리하지 않는다.
- 실제 LiDAR TF와 vehicle reference point 미확정, MORAI 추종오차·제동거리 미검증.
- Traffic Light는 별도 개발/최종 통합 과제다. 대상 신호·정지선·freshness·재출발 검증이 남았다.
- GPS Shadow는 일부 구현이다. 활성 Local 중 GPS invalid는 정지하지만, 기존 음영 차선 분기에는 approach cap이 적용되지 않는다.
- Dynamic obstacle tracking/속도 예측 미구현. Merging legacy logic 재설계 필요.
- Mission priority/stop reason 통합과 전체 코스 회귀 검증이 필요하다.
- 새 PC의 Python dependency lock과 최신 Ubuntu 빌드/ROS smoke/collector 실행 확인이 남아 있다.

| 우선순위 | 팀 전체 작업 |
|---|---|
| P0 | ROS/MORAI Local 실검증, sensor/TF/geometry 확인 |
| P1 | Static obstacle/RETURN_TO_GLOBAL tuning, Global Path 품질 처리, velocity profile |
| P2 | Traffic Light 최종 통합, GPS Shadow 안정화 |
| P3 | Dynamic obstacle tracking, Merging, Mission Manager, 전체 코스 통합 |

## feat/local-planner merge 조건

아래 항목은 실험 증거를 받은 뒤 체크한다. 아직 main merge 완료로 표시하지 않는다.

- [ ] Ubuntu catkin build
- [ ] ROS smoke test
- [ ] Local Planner OFF baseline
- [ ] Local Planner ON + empty zone
- [ ] static zone + no obstacle
- [ ] center obstacle
- [ ] left obstacle
- [ ] right obstacle
- [ ] all candidates blocked → full brake
- [ ] LiDAR stale → HOLD + full brake
- [ ] TF failure → HOLD + full brake
- [ ] RETURN_TO_GLOBAL → NORMAL
- [ ] vehicle geometry 확인
- [ ] LiDAR TF 확인
- [ ] 테스트 결과 문서화

## 문서 안내

| 문서 | 역할 |
|---|---|
| [LOCAL_PLANNING.md](docs/LOCAL_PLANNING.md) | Local 알고리즘·상태·Controller·TF 상세 |
| [LOCAL_PLANNER_OFFLINE_TOOLS.md](docs/LOCAL_PLANNER_OFFLINE_TOOLS.md) | preflight/path/scenario/sweep/collector 사용법 |
| [LOCAL_PLANNER_TEST_RESULTS.md](docs/LOCAL_PLANNER_TEST_RESULTS.md) | MORAI Phase 0~10 조건·결과·증거 기록 |
| [PROJECT_STATUS.md](docs/PROJECT_STATUS.md) | 전체 프로젝트 진행현황; 별도 문서의 🟡는 README의 🧪에 해당 |
| [UDP_CONNECTION.md](docs/UDP_CONNECTION.md) | UDP 구조·패킷·watchdog; 포트 값은 현재 network.yaml 우선 |
