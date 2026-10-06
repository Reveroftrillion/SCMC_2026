# LiDAR local planning / RViz

ROS/MORAI 없이 실행하는 설정 검사·경로 분석·합성 scenario·parameter sweep과 팀원용 진단 수집은
[LOCAL_PLANNER_OFFLINE_TOOLS.md](LOCAL_PLANNER_OFFLINE_TOOLS.md)에 정리했다.
실측값을 채울 시험 템플릿은 `src/planning/config/local_planner_test.yaml`,
Phase 0~10 결과 기록은 [LOCAL_PLANNER_TEST_RESULTS.md](LOCAL_PLANNER_TEST_RESULTS.md)를 사용한다.
이번 도구 추가에서는 실행 Planner/Controller와 신호등·합류·GPS 음영 코드를 수정하지 않았다.

## 기존 코드 조사와 재사용

| 기능 | 기존 파일 / 현재 연결 |
|---|---|
| 전역 경로 저장 | `src/planning/paths/global_path.txt`, 기록 도구 `scripts/path_maker.py` (절대 UTM 기록을 현재 상대 XY loader에 그대로 넣으면 안 됨) |
| 전역 경로 생성 | `scripts/GlobalPathPlanner.py` → `/global_path` (`nav_msgs/Path`, map) |
| 제어용 전역 부분 경로 | `scripts/PathPlanner.py` → `/control_path` (`nav_msgs/Path`, map) |
| 전역 경로 추종 | `src/control/src/controller.cpp`, 기존 Pure Pursuit + heading correction 유지 |
| 종방향 제어 | `src/control/src/pid.cpp`, 기존 60km/h 상한 유지 |
| 사용한 Cubic Spline | `scripts/planning_geometry.py:CubicSpline2D` (SciPy). ROS 없는 수학 모듈로 분리; 기존 `utils.py`에서도 같은 이름으로 제공 |
| 다른 기존 spline 구현 | `scripts/interpolation/cubic_spline_planner.py` (데모/수학 모듈) |
| 기존 Frenet 코드 | `utils.py:catesian_to_frenet`, `interpolation/cartesian_frenet_converter.py`, `interpolation/frenet_optimal_trajectory.py` |
| 사용한 다항식 | `interpolation/quintic_polynomials_planner.py:QuinticPolynomial` 재사용. matplotlib는 데모 실행 때만 import |
| 확장한 Local Planner | `scripts/StaticObstacleAvoidancePlanner.py` 기존 클래스와 `/local_path`, `/local_path_done` 유지 |
| 사용하지 않은 예전 Local Planner | `scripts/LocalPathPlanner.py`: 저장소에 없는 `erp_drive`, `constant`, `frenet_frame` 의존. PathReference/Int16 인터페이스도 현재 Path/Bool과 다름 |
| LiDAR 수신 | 기존 VLP16 드라이버 → `/velodyne_points` (`sensor_msgs/PointCloud2`, velodyne) |
| 검출 | `src/lidar/src/object_detection_static.cpp` + `dbscan.h`: 기존 ROI/DBSCAN 재사용 |
| 장애물 출력 | `/obstacle_info_static` (`lidar_object_detection/ObjectInfo`), `/bounding_box_static`, `/cluster_static` |
| 위치·방향 | `/current_pose` (`PoseStamped`, map), `/vehicle_status`의 yaw; 기존 GPS/IMU 코드는 변경하지 않음 |
| waypoint | Controller의 `/curr_idx` (`Int16`) 그대로 사용 |
| 기존 동적 관련 코드 | `controller.cpp:MergingzoneControl`의 정지/시간 후 장애물 무시 로직과 `object_detection_car.cpp`의 `/car_info` 발견. 현재 구간은 비활성. 새 동적 정책에서 시간만 지나면 장애물을 무시하는 복구는 사용하지 않음 |
| RViz | 기존 `VizPlanner.py`의 `~viz_*` 토픽 재사용. 오프셋을 뺀 좌표에 map을 붙이던 문제를 실제 TF로 대체 |

실제 실행되는 라이다 launch는 이름이 crossing이지만 **static 검출기와 car 검출기**를 실행한다.
`ObjectInfo`에는 원래 Header가 없었다. static 검출기는 이제 입력 PointCloud의 시간·frame을 보존하며,
장애물 없는 스캔도 count=0을 발행한다. 100개 배열 한계를 넘는 검출은 count=-1로 알리고 planner는 정지한다.
메시지 정의가 바뀌므로 모든 ROS 노드를 종료한 뒤 전체 워크스페이스를 다시 빌드하고 재시작해야 한다.

## 알고리즘

1. 빈 구간 설정이면 NORMAL로 유지하여 기존 Global Path 추종을 그대로 사용한다.
2. 지정 구간에 들어가면 기존 waypoint 주변의 Global Path 일부에 기존 CubicSpline2D를 적용한다.
   전역 경로 파일이나 GlobalPathPlanner 출력 자체는 변경하지 않는다.
   마지막으로 남긴 점과의 거리가 `min_reference_segment`보다 짧으면 해당 점을 제거한다.
   입력 shape/index/유한 값, 최소 길이, segment yaw 변화, spline 미분 및 곡률을 검사한다.
   비정상 reference는 곡률을 0으로 감춰 사용하지 않고 HOLD한다. 전역 waypoint 번호는 그대로다.
3. 기존 Cartesian→Frenet 함수로 근접점을 찾고 선분 투영으로 s/d를 보정한다.
4. 현재 d와 heading에서 시작하는 5차 다항식 d(s)로 각 offset까지 이동하고, offset 유지 후
   5차 다항식으로 d=0에 복귀한다. 샘플 간격·구간 길이·offset·도로 반폭은 YAML에서 조정한다.
5. 모든 후보를 매 주기 재검사한다. 차량 앞/뒤 길이와 폭을 덮는 여러 원의 이동 선분과
   장애물 bounding circle 사이 거리를 계산한다. 스캔의 bounding box 8개 꼭짓점을 map으로
   변환해 원 반지름을 구하며, 차량 폭·safety margin·샘플 여유를 추가한다.
   경로 점 사이도 선분 거리로 검사하므로 점 샘플 사이 장애물을 건너뛰지 않는다.
6. 차량 앞/뒤 모서리까지 도로 반폭을 검사하고, 충돌·도로 반폭 초과·곡률 초과·거리당 조향 변화량 초과 후보는 제외한다.
7. 남은 후보의 비용은 `obstacle + offset + curvature + steering + continuity + return` 가중합이다.
   obstacle은 충돌 여유의 역수, offset은 d² 평균, curvature는 곡률² 평균,
   steering은 `atan(wheelbase*kappa)`의 거리당 변화율² 평균이다.
   continuity는 이전 선택 경로와의 공간 거리, return은 목표 offset²이다.
   좌우 반전 패널티와 같은 offset 유지 hysteresis를 더해 잦은 좌우 전환을 억제한다.
8. 회피 중 안전한 중심 복귀 후보가 다시 생기면 구간 안에서도 RETURN_TO_GLOBAL로 전환한다.
   구간 밖에서도 아직 횡방향 오차가 있으면 Local 제어를 유지한다.
   d/heading 오차가 허용 범위에 일정 시간 연속으로 머문 뒤 NORMAL로 복귀한다.
   HOLD/차단/정렬 이탈 중에는 복귀 타이머를 초기화한다.

현재 방식은 **공간상의 Frenet 후보 경로**다. 시간 기반 동적 장애물 예측이나 최적 속도 궤적은 구현하지 않았다.
기존 frenet_optimal_trajectory 데모의 전역 상수·점 장애물 충돌 검사 대신 실제 검출 크기와 현재 Controller에
맞춘 평가 모듈 `local_planning_core.py`를 추가했다. 실제 도로 경계 검출 대신 설정한 corridor를 사용한다.
차량 치수·도로 폭 기본값은 현장 실측값이 아니므로 적용 전에 확인해야 한다.

## 상태와 Controller 연결

- NORMAL: 기존 `/control_path` 선택. 미설정 구간에서는 속도 제한 0으로 기존 제어 유지.
  구간 접근 또는 정적 구간의 안전한 중심 경로에서는 비활성 LocalPlan으로 속도 상한만 전달한다.
- STATIC_OBSTACLE: 후보 생성/검사/선택, `/local_path` 발행. 모든 후보가 막히면 정지.
- RETURN_TO_GLOBAL: 중심선으로 부드럽게 복귀한 후 기존 제어 경로로 전환.
- DYNAMIC_OBSTACLE: 별도 `process_dynamic` 함수. 기본 정책 `stop_on_obstacle`은 중심 복귀 경로가
  막히면 정지, 비면 낮은 설정 속도로 진행한다. 정적 회피처럼 좌우로 우회하지 않는다.
  `observe_only`는 새 동적 구간에서 관찰만 하고 기존 제어를 유지하는 진단 옵션이다.
  실제 stop/follow/avoidance 미션 조건과 장애물 속도 추정은 추후 정의해야 한다.
- HOLD: 활성 구간에서 LiDAR/pose/index/TF가 없거나 오래됐거나, 경로가 유효하지 않을 때 정지.

새 `/local_plan` (`simul_msgs/LocalPlan`)에는 경로·활성 여부·정지 요청·속도 상한·상태를 **하나의 메시지**로 담는다.
기존 `/local_path`와 `/local_path_done`은 계속 발행한다. 서로 다른 토픽의 도착 순서로 경로만 먼저 바뀌는
경쟁을 피하기 위해 Controller의 새 모드만 `/local_plan`을 사용한다.

Controller의 `~enable_local_planner` 기본값은 false다. true일 때만 로컬 경로를 선택하며,
기존 `calcSteer` 함수는 수정하지 않았다. 로컬 구간에 20km/h(기본)와 곡률 기반 횡가속 상한을 적용한다.
구간 접근 시에도 설정한 속도 상한을 적용한다. 기본 주행 입력이 아직 없으면 새 모드에서 full brake를 발행한다.
동적 구간 기본 속도는 10km/h다. 0.5초 동안 새 플랜이 없으면 가속 0/제동 1로 정지한다.
기존 신호 정지는 로컬 경로보다 우선한다. GPS 음영 중 활성 로컬 계획은 정지한다.

`stop=true`는 active 값과 관계없이 full brake를 요청한다. stale/invalid 플랜도
accel=0/brake=1/steering=0으로 처리한다. active 경로의 map frame, plan/path/point timestamp,
최소 5점, 유한 좌표·비음수 곡률, 단위 quaternion, 연속 중복점, 양수 속도 상한을 검사한다.
inactive 플랜은 빈 path와 유한 비음수 속도 상한을 사용한다. 0은 추가 속도 제한 없음이다.
Local Planner를 끄면 이 검증·속도 제한은 적용되지 않는다.

## 좌표계 / TF

- 기존 `/global_path`, `/control_path`, `/current_pose`, `/local_path`: map (기존 UTM 좌표).
- LiDAR: PointCloud와 ObjectInfo의 실제 header.frame_id. 기본 드라이버는 velodyne.
- VizPlanner: `map → planning_vehicle`를 **current_pose 메시지 시간**으로 발행.
  planning_vehicle 원점은 current_pose의 XY 기준점이며, 후륜축이라고 가정하지 않는다. 지면 z=0의 평면 모델이다.
- `viz_planner/lidar_xyz_rpy`를 입력한 경우에만 `planning_vehicle → velodyne` 정적 TF를 발행한다.
  입력은 current_pose 기준점에서 LiDAR까지의 `[x,y,z,roll,pitch,yaw]`, m/rad다.
  z는 계획 평면에서의 장착 높이다. 기존 하드코딩 1m/3.9m는 근거가 충돌하여 재사용하지 않았다.
- 이미 올바른 map→LiDAR TF를 제공하는 노드가 있다면 lidar_xyz_rpy는 빈 배열로 유지한다.
  같은 TF를 두 노드가 중복 발행하지 않도록 한다.
- 장애물 변환은 스캔 취득 시각의 TF를 조회한다. 최신 TF나 현재 위치로 무조건 대체하지 않는다. LiDAR가 pose보다 빠른 경우를 위해 최근 스캔을 큐에 보관하고,
  유효시간 안에서 정확한 취득 시각의 TF를 조회할 수 있는 가장 최신 스캔을 사용한다.
- RViz Fixed Frame은 `planning_origin`이다. 첫 current_pose 위치에 고정된 `map → planning_origin` TF로
  큰 UTM 좌표의 표시 정밀도를 개선한다. 실제 메시지 좌표는 map에 그대로 남아 올바르게 변환된다.
- 기존 제어 경로의 position.z는 곡률이다. RViz 전용 경로 복사본에서만 z를 평면으로 바꾼다.

## 설정과 실행

`src/planning/config/local_planner.yaml`에서 다음 구간들을 입력한다. **실제 구간은 현재 비어 있다.**

```yaml
static_obstacle_avoidance_planner:
  static_zones: []
  dynamic_zones: []
```

각 리스트 원소는 `{start: 시작번호, end: 끝번호}` (포함 범위), 또는
`{xmin: 최소X, xmax: 최대X, ymin: 최소Y, ymax: 최대Y}` (map/UTM m) 중 하나다.
두 구간이 겹치면 동적 구간이 우선한다. 변경한 YAML은 노드 재시작으로 적용한다.

빌드:

```bash
cd ~/다운로드/SCMC_2026-main
source /opt/ros/noetic/setup.bash
catkin_make -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

전체 실행 (MORAI_IP를 실제 주소로 바꾼다. 기존 상태 메시지 규격을 확인한 환경):

```bash
roslaunch main simulator.launch morai_ip:=MORAI_IP \
  status_layout_confirmed:=true enable_control:=true \
  enable_local_planner:=true rviz:=true
```

이 명령은 기존 LiDAR/Localization/Global planner/Controller/UDP launch에 로컬 planner와 RViz를 추가한다.
`enable_control`의 기존 기본값은 false이며 변경하지 않았다. 정상적으로 상태·GPS가 수신되는 환경이 필요하다.
기존 노드들과 중복 실행하지 않는다.

현재 Windows 작업 환경에서는 ROS와 실제 VLP16 드라이버 포함 실행을 검증하지 못했다.
Ubuntu ROS PC에 `velodyne_pointcloud` 등 런타임 의존성을 설치한 뒤 위 전체 명령을 사용한다. 이미 별도 드라이버/rosbag이 `/velodyne_points`를
발행한다면 전체 명령에 `start_lidar_driver:=false`를 추가한다. 이 옵션은 클러스터링 노드를 끄지 않는다.
입력 PointCloud가 없으면 활성 장애물 구간에서 HOLD하는 것이 정상이다.

이미 센서·GPS·전역 planner가 실행 중이라면 Controller를 새 모드로 재시작하고:

```bash
roslaunch control control.launch enable_local_planner:=true
roslaunch planning local_planner.launch
```

두 명령은 서로 다른 터미널에서 실행한다. LiDAR DBSCAN의 넓힌 ROI 설정은 시작 시 읽으므로
이 방식에서는 검출기도 재시작하거나 rqt_reconfigure로 같은 값을 적용한다.
headless 환경에서는 `roslaunch planning local_planner.launch rviz:=false`를 사용한다.

## RViz에서 볼 토픽

| 표시 | 토픽 |
|---|---|
| 전역 경로 (흰색) | `/viz_planner/viz_global_path` |
| 기존 제어 부분 경로 (노란색) | `/viz_planner/viz_control_path` |
| 선택 Local Path (초록색) | `/viz_planner/viz_local_path` |
| 후보/탈락 사유/비용 | `/static_obstacle_avoidance_planner/candidates` (파랑=유효, 빨강=탈락, 초록=선택) |
| map 좌표 충돌용 장애물 원 | `/static_obstacle_avoidance_planner/obstacles` |
| 검출 bounding box | `/bounding_box_static` |
| 원본 LiDAR | `/velodyne_points` |
| 클러스터 PointCloud | `/cluster_static` |
| 현재 차량 위치·방향 | `/viz_planner/viz_current_pose` |
| NORMAL/STATIC/DYNAMIC/HOLD 상태 | `/static_obstacle_avoidance_planner/state_marker` |

원본 데이터/진단: `/global_path`, `/control_path`, `/local_path`, `/local_path_done`, `/local_plan`,
`/obstacle_info_static`, `/curr_idx`, `/current_pose`.
LiDAR TF가 아직 없으면 raw cloud와 boxes의 map 표시가 불가능하며, 활성 구간에서는 HOLD한다.
미설정 구간에서는 후보가 생성되지 않는 것이 정상이다.

## 검증

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
OPENBLAS_NUM_THREADS=1 python3 -B -m unittest discover -s src/planning/tests -v
python3 -B src/planning/tests/ros_local_planner_smoke.py
```

수학 테스트: 중앙 장애물, 완전 차단, 좌우 흔들림, 새 장애물로 기존 선택 무효화, 차량 폭,
샘플 사이 충돌, 곡선, 중복 waypoint, 경로 끝, 복귀, 구간 경계.
ROS 통합 테스트는 임시 포트의 별도 master를 사용하고 실제 Controller/Planner/VizPlanner/DBSCAN을 실행한다.
합성 입력만 발행하며 MORAI UDP 송신기와 실제 LiDAR 드라이버는 실행하지 않는다.
일반 모드 출력 비교, 로컬 추종, 차단/복구, TF 누락, 동적 정지, 플래너 종료, 빈 스캔 헤더/개수를 검사한다.
이전 문서에는 catkin 빌드·수학 15개·ROS 합성 16개 통과가 보고되어 있었다.
현재 feat/local-planner 변경 후 Windows Python에서 ROS 없이 수학/상태/실제 node adapter 합성 테스트
50개를 통과했다. adapter 테스트는 메시지·시계·TF API만 대체하며 ROS transport나 TF 보간을 검증하지 않는다.
오프라인 도구 회귀 검사를 추가한 현재 전체 suite는 68개 중 66개 통과,
POSIX Bash 수집기 실행 검사 2개 skip이다. 기존 core/adapter 50개는 그대로 포함한다.
현재 C++ 변경의 컴파일, 확장 ROS smoke test, MORAI 주행과 RViz 화면 정합은 이 환경에서 재검증하지 못했다.
이전 ROS 통과 보고를 현재 변경의 검증 결과로 해석하지 않는다.

ROS 없는 테스트는 ROS setup을 source할 필요가 없다. NumPy/SciPy/PyYAML이 필요하다:

```bash
OPENBLAS_NUM_THREADS=1 python3 -B -m unittest discover -s src/planning/tests -v
g++ -std=c++11 -Wall -Wextra -pedantic -I src/control/include \
  src/control/tests/local_plan_validation_test.cpp -o /tmp/scmc-local-plan-test
/tmp/scmc-local-plan-test
```

C++ standalone 계약 테스트는 Controller가 실제 사용하는 template validator를 호출한다.
현재 환경에는 컴파일러가 없어 실행하지 못했다. Ubuntu에서는 ROS 빌드 전에 이 검사를 실행할 수 있다.

## 추가 설정과 동작 계약

| parameter | 기본값 | 의미 |
|---|---:|---|
| `min_reference_segment` | 0.05m | 마지막 유지점에서 이 거리 미만인 점 제거 |
| `max_reference_yaw_change` | 1.2rad | reference segment 간 급격한 방향 변화 거부 |
| `max_reference_curvature` | 0.5m⁻¹ | spline 이상 곡률 거부; candidate 제한 0.13과 별개 |
| `global_path_timeout` | 2.5s | 1Hz static global path의 로컬 수신 시각 기준 timeout |
| `approach_distance` | 15m | 설정 구간 진입 전 경로상 접근 거리; 0이면 끔 |
| `approach_speed_kmh` | 20km/h | 접근 속도 상한; 해당 미션 상한과 작은 값 사용 |

이 값들은 튜닝 기본값이다. 도로 번호·차량 치수·센서 offset을 새로 추측하지 않았다.
index zone은 원본 waypoint 번호와 실제 segment 길이로 접근 거리를 계산한다.
UTM rectangle은 Global Path 선분과 사각형의 교점을 사용하므로 waypoint 사이 진입도 잡는다.
사각형을 임의로 확장하거나 차량 뒤의 지나간 구간을 접근 대상으로 선택하지 않는다.
잘못된 YAML zone/geometry는 원인을 출력하고 시작을 거부한다. path 수신 후 index zone의 끝이
path 범위를 벗어나면 경고와 HOLD를 낸다. 새 global path는 수신 시각을 갱신하되 static header 시간은 freshness에 사용하지 않는다.

접근 중에는 `NORMAL: approach ...`, active=false, 빈 path, 양수 speed_limit을 발행한다.
이는 제동거리 최적화나 점진적 속도 궤적이 아니라 **진입 전 속도 상한**이다.
현재 실제 속도와 제동 응답에 맞춰 margin을 늘려야 한다.
빈 구간 설정에서는 접근 감속과 센서/TF 요구가 생기지 않는다.

장애물 기억은 scan 처리 시각 대신 실제 로컬 수신 시각에서 만료를 계산한다.
같은 scan을 재사용해도 기억을 연장하지 않는다. 모든 box가 검증된 후 한 번에 갱신한다.
LiDAR stream이 stale이면 기억만으로 계속 주행하지 않고 HOLD한다. 빈 scan도 TF가 필요하다.
외부 TF provider가 있으면 `lidar_xyz_rpy: []` 그대로 쓸 수 있다.
없으면 실측한 extrinsics를 입력해야 하며, 누락 시 `HOLD: LiDAR TF map <- velodyne unavailable at scan time`이 정상이다.

정적 구간의 center 후보가 안전하고 차량이 정렬돼 있으면 NORMAL로 Global 제어를 사용하면서
미션 속도 상한과 곡률 기반 상한을 유지한다. 회피 후에는 안전한 center 후보를 우선해
RETURN_TO_GLOBAL로 복귀한다. HOLD 동안에는 연속 정렬 완료 시간을 쌓지 않는다.

### Ubuntu MORAI 테스트 절차

workspace 루트에서:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
python3 -B src/planning/tests/ros_local_planner_smoke.py
```

smoke test는 별도 master와 합성 센서만 사용한다. UDP 송신이나 실제 LiDAR driver를 시작하지 않는다.
Controller disabled/정상/Local 경로/접근 속도 상한/stop/stale/invalid/Global 복귀를 검사한다.

실제 값으로 설정할 복사본을 준비한다. index는 0부터 시작하며 시작·끝을 포함한다:

```bash
cp src/planning/config/local_planner.yaml /tmp/scmc-local-planner.yaml
nano /tmp/scmc-local-planner.yaml
```

`static_zones`/`dynamic_zones`, LiDAR TF, geometry/도로 폭을 실제 센서·코스 기준으로 입력한다.
형식은 `{start: 실제시작번호, end: 실제끝번호}` 또는
`{xmin: 실제최소X, xmax: 실제최대X, ymin: 실제최소Y, ymax: 실제최대Y}`다.
기본 치수는 검증된 실측값이 아니다. 기존 TF provider를 사용하면 동일 child TF를 추가하지 않는다.

```bash
export MORAI_IP=192.168.0.15  # 실제 MORAI IP로 교체
roslaunch main simulator.launch morai_ip:="$MORAI_IP" \
  status_layout_confirmed:=true enable_control:=false \
  enable_local_planner:=true rviz:=true \
  local_planner_config:=/tmp/scmc-local-planner.yaml
```

먼저 송신을 끈 상태에서 센서·state·TF를 확인하고, 같은 명령의 `enable_control:=true`로 재시작해 주행한다.
상태 패킷 layout을 확인한 환경에서만 `status_layout_confirmed:=true`를 사용한다.
이미 LiDAR driver나 rosbag이 있으면 `start_lidar_driver:=false`를 추가한다.

```bash
rostopic hz /current_pose
rostopic hz /global_path
rostopic hz /velodyne_points
rostopic hz /obstacle_info_static
rostopic echo /local_plan/state
rostopic echo /local_plan/stop
rostopic echo /local_plan/speed_limit_kmh
rostopic echo /control_cmd
rosrun tf tf_echo map velodyne
```

RViz Fixed Frame은 `planning_origin`. 위 RViz 표의 토픽을 그대로 사용한다.
candidate 라벨에 SELECTED/offset/cost/clearance/탈락 이유가 표시되고 state marker에도
선택 offset/cost 또는 HOLD/STOP 이유가 보인다. NORMAL에서는 selected Local Path가 비는 것이 정상이다.
startup geometry와 reference 요약은 각각 시작 로그와 5초 제한 로그로 출력한다.
상태 변경은 즉시, 같은 상태의 상세 값은 2초 제한, HOLD 원인은 1초 제한으로 출력한다.

정상 예시:

```text
[LOCAL CONFIG] width=1.90 front=3.50 rear=1.00 wheelbase=3.00 m; road_half_width=4.50; approach=15.0 m
[LOCAL PLANNER] NORMAL: approach STATIC_OBSTACLE in ... active=False stop=False speed_limit=20.0 ...
[LOCAL REFERENCE] input=... removed=... peak_curvature=...
[LOCAL PLANNER] STATIC_OBSTACLE active=True stop=False speed_limit=... offset=... cost=...
[LOCAL PLANNER] RETURN_TO_GLOBAL active=True stop=False ...
[LOCAL PLANNER] NORMAL: return complete active=False stop=False ...
```

차단은 `STATIC_OBSTACLE: no valid candidate (...)`와 stop=true,
입력 오류는 `HOLD: <reason>`와 stop=true로 구분한다.
신호등/합류/GPS 음영 로직은 이번 변경에서 수정하지 않았다.
GPS 음영 모드에서는 접근 속도 상한이 일반 calcVelocity처럼 적용되지 않는 기존 분기가 남아 있다.
활성 Local Plan 중 GPS invalid는 기존대로 정지한다. 이 충돌은 추후 통합 과제다.

## 변경 파일

아래는 이번 개발 시작 시점의 HEAD 대비 변경이다. 이전 Local Planner 도입 파일과 구분한다.

수정:
- `src/planning/scripts/StaticObstacleAvoidancePlanner.py`
- `src/planning/scripts/local_planning_core.py`, `src/planning/scripts/utils.py`
- `src/planning/config/local_planner.yaml`
- `src/control/src/controller.cpp`
- `src/planning/tests/test_local_planning.py`, `src/planning/tests/ros_local_planner_smoke.py`
- `docs/LOCAL_PLANNING.md`

신규:
- `src/planning/scripts/planning_geometry.py`, `src/planning/scripts/local_planning_state.py`
- `src/planning/tests/test_local_planning_state.py`, `src/planning/tests/test_local_planner_node.py`
- `src/control/include/control/local_plan_validation.h`
- `src/control/tests/local_plan_validation_test.cpp`
- `docs/PROJECT_STATUS.md`

## 사용자가 제공할 값

1. 정적·동적 구간의 waypoint 시작/끝 또는 map 좌표 사각형. 진입 구간은 감속·회피 준비 거리를 포함해야 한다.
2. 현재 시나리오와 차량에서의 LiDAR 장착 위치/각도 및 current_pose 기준점의 의미.
3. 실제 차량 폭, 기준점에서 앞/뒤 범퍼 거리, 허용 도로 폭/차선 침범 범위.
4. 동적 장애물 미션의 정지/재출발/추종/회피 조건.
5. LiDAR 높이에 맞는 지면 제거 z ROI와 차량 자체 제거 범위. 기존 기본값은 유지하되 실제 cloud와 맞춰야 한다.

모든 후보는 해당 주기에서 검출된 장애물과 단기 기억만 기준으로 평가한다. 가림, DBSCAN 누락,
도로 경계 미검출, 차량 모델 차이까지 보장하는 주행 검증을 대체하지 않는다.
