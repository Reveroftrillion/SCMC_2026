# SCMC 2026

2026 대학생 창작 모빌리티 경진대회 AI융합자율주행 부문용 ROS1 자율주행 코드입니다.

현재 저장소는 기존 **Global Path 기반 기본 경로추종**에 더해 **LiDAR 기반 Local Planning**, **RViz 시각화**, **YOLO 신호등 인식 모델**, **LocalPlan 통합 메시지**, **MORAI UDP 제어 연동**을 포함합니다.

> ROS Noetic 기준으로 개발하고 있습니다.  
> 실제 주행 전 MORAI Network Setting, 센서 포트, 차량/센서 파라미터와 미션 구간을 반드시 확인해야 합니다.

---

## 최근 주요 변경사항

### 1. Global Path 파일 정리

기존 경로 파일명 `zzinmak.txt`를 다음과 같이 변경했습니다.

```text
src/planning/paths/global_path.txt
```

`GlobalPathPlanner.py`도 `global_path.txt`를 직접 읽도록 수정했습니다.

Global Path는 각 waypoint에 대해 다음 정보를 계산합니다.

- 현재 위치에 가까운 waypoint 탐색
- Path yaw 계산
- 곡률 계산
- 현재 위치 기준 약 20 m 전방 Control Path 생성

---

### 2. LiDAR 기반 Local Planner 추가

기존 `/control_path`를 따라가는 기본 주행은 유지하면서, 설정된 장애물 구간에서만 Local Planner가 개입하도록 구성했습니다.

추가된 주요 파일:

```text
docs/LOCAL_PLANNING.md
src/planning/config/local_planner.yaml
src/planning/launch/local_planner.launch
src/planning/scripts/local_planning_core.py
src/planning/tests/
src/simul_msgs/msg/LocalPlan.msg
```

Local Planner의 기본 상태는 다음과 같습니다.

```text
NORMAL
  ↓ 장애물 구간 진입
STATIC_OBSTACLE
  ↓ 회피 완료
RETURN_TO_GLOBAL
  ↓ 중심 경로 복귀
NORMAL
```

입력 데이터가 유효하지 않거나 경로 생성이 불가능한 경우에는 `HOLD` 상태로 전환하여 정지합니다.

동적 장애물 구간용 `DYNAMIC_OBSTACLE` 상태도 분리되어 있으나, 현재 기본 정책은 정적 장애물처럼 좌우 회피하는 방식이 아니라 `stop_on_obstacle` 기반의 보수적인 정책입니다.

---

### 3. Local Planner 알고리즘

Local Planner는 Global Path 일부를 기준 경로로 사용하고 Frenet 좌표계에서 후보 경로를 생성합니다.

주요 흐름:

```text
Global Path
    ↓
현재 waypoint 주변 reference path 추출
    ↓
Cartesian → Frenet 변환
    ↓
여러 lateral offset 후보 생성
    ↓
5차 다항식 기반 차선 변경 / 복귀 경로 생성
    ↓
LiDAR 장애물과 충돌 검사
    ↓
도로 폭 / 곡률 / 조향 변화율 검사
    ↓
후보 비용 계산
    ↓
최적 Local Path 선택
    ↓
/local_plan 발행
```

후보 경로 비용은 다음 요소를 사용합니다.

- 장애물과의 거리
- 횡방향 offset
- 곡률
- 조향 변화량
- 이전 선택 경로와의 연속성
- Global Path 복귀 비용
- 좌우 경로 변경 hysteresis

현재 방식은 **공간상의 Frenet 후보 경로 생성**이며, 시간 기반 동적 장애물 예측이나 최적 속도 궤적 생성은 아직 포함하지 않습니다.

자세한 내용은 다음 문서를 참고합니다.

```text
docs/LOCAL_PLANNING.md
```

---

### 4. LocalPlan 통합 메시지

Local Planner와 Controller 사이의 상태를 하나의 메시지로 전달하기 위해 다음 메시지를 추가했습니다.

```text
src/simul_msgs/msg/LocalPlan.msg
```

`/local_plan`에는 다음 정보가 포함됩니다.

- Local Path
- Local Planner 활성 여부
- 정지 요청
- 속도 제한
- Planner 상태

기존 `/local_path`, `/local_path_done`도 유지하지만, 새로운 Controller 모드에서는 `/local_plan`을 기준으로 Local Planner 상태를 처리합니다.

---

### 5. Controller 연동

Controller는 기본적으로 기존 `/control_path`를 사용합니다.

```text
/control_path
    ↓
Pure Pursuit
    +
Adaptive Heading Correction
    ↓
Steering Command
```

Local Planner를 활성화한 경우 유효한 `/local_plan`이 들어오면 해당 Local Path를 사용합니다.

```text
/control_path
      ↓
  Controller
      ↑
/local_plan
```

현재 Controller에는 다음 기능이 포함되어 있습니다.

- Pure Pursuit 기반 횡제어
- Adaptive Heading Error Gain
- Steering Saturation
- Low-pass Filter
- 곡률 기반 기본 속도 제어
- Local Planner speed limit 적용
- Local Planner stale/invalid plan fail-safe stop
- GPS 음영 구간 LaneNet 제어
- 기존 신호등 정지 로직
- 합류 구간 로직

Local Planner는 기본값이 비활성화되어 있습니다.

```text
enable_local_planner:=false
```

---

### 6. IONIQ 5 기준 조향 제어 튜닝

기본 차량은 MORAI IONIQ 5를 기준으로 사용하고 있습니다.

현재 Controller 주요 파라미터:

```text
Wheelbase = 3.0 m
```

조향은 Pure Pursuit와 Heading Correction을 결합한 뒤 normalized steering command로 변환합니다.

Heading Error의 크기에 따라 correction gain을 다르게 적용하며, 최종 steering에는 saturation과 low-pass filter를 적용하여 좌우 oscillation을 줄입니다.

---

### 7. LiDAR / RViz 연동 추가

Local Planning 확인을 위해 LiDAR 및 RViz 관련 launch와 설정을 추가했습니다.

```text
src/lidar/launch/lidar_rviz.launch
src/lidar/rviz/lidar_view.rviz
src/planning/rviz/local_planning.rviz
```

Local Planner 활성 시 다음 정보를 RViz에서 확인할 수 있습니다.

| 항목 | 토픽 |
| --- | --- |
| Global Path | `/viz_planner/viz_global_path` |
| Control Path | `/viz_planner/viz_control_path` |
| Local Path | `/viz_planner/viz_local_path` |
| Local Planner 후보 | `/static_obstacle_avoidance_planner/candidates` |
| 장애물 | `/static_obstacle_avoidance_planner/obstacles` |
| LiDAR Bounding Box | `/bounding_box_static` |
| Raw LiDAR | `/velodyne_points` |
| Cluster PointCloud | `/cluster_static` |
| 현재 차량 위치 | `/viz_planner/viz_current_pose` |
| Planner State | `/static_obstacle_avoidance_planner/state_marker` |

---

### 8. YOLO 신호등 모델 포함

신호등 인식을 위한 Ultralytics YOLO custom weight를 저장소에 포함했습니다.

```text
src/camera/models/1027_40epoch.pt
```

YOLO node:

```text
src/camera/scripts/traffic_yolo.py
```

현재 설정:

```text
Confidence Threshold = 0.4
Inference Device = CPU
```

입력:

```text
/camera/front/image/compressed
```

`simulator.launch`에서 다음 remap을 통해 YOLO node에 연결합니다.

```text
/camera/front/image/compressed
        ↓
/image_jpeg/compressed
        ↓
traffic_yolo.py
```

출력:

```text
/traffic_light_status
/yolo_traffic_light/image_raw
```

현재 detector는 검출된 객체 중 confidence가 가장 높은 신호등 class를 `/traffic_light_status`로 발행합니다.

> 신호등 AI 인식 자체는 연결되어 있지만, 실제 정지선 및 미션 구간별 신호등 제어는 추가 검증이 필요합니다.

---

## 현재 개발 상태

| 항목 | 상태 |
| --- | --- |
| MORAI ↔ ROS UDP 통신 | ✅ 확인 |
| GPS 수신 | ✅ 확인 |
| IMU 수신 | ✅ 확인 |
| Ego Vehicle Status 수신 | ✅ 확인 |
| `global_path.txt` 로딩 | ✅ 확인 |
| `/control_path` 생성 | ✅ 확인 |
| 기본 경로추종 | ✅ 한 바퀴 주행 확인 |
| IONIQ 5 조향 튜닝 | ✅ 적용 |
| YOLO custom weight | ✅ 저장소 포함 |
| YOLO 신호등 영상 추론 | ✅ 확인 |
| Local Planner 코드 | ✅ 추가 |
| Local Planner ROS 연동 | ✅ 구현 |
| Local Planner RViz | ✅ 추가 |
| Local Planner 실제 미션 구간 | ⚙️ 미설정 |
| 정적 장애물 회피 실주행 | 🧪 검증 필요 |
| 동적 장애물 정책 | 🧪 개발/검증 중 |
| 신호등 정지 및 재출발 | 🧪 검증 중 |
| 합류 구간 | 🧪 재설정 필요 |

---

# 프로젝트 구조

```text
SCMC_2026/
├── docs/
│   ├── LOCAL_PLANNING.md
│   └── UDP_CONNECTION.md
│
├── src/
│   ├── camera/
│   │   ├── models/
│   │   │   └── 1027_40epoch.pt
│   │   └── scripts/
│   │       ├── traffic_yolo.py
│   │       └── lanenet.py
│   │
│   ├── control/
│   │   ├── launch/
│   │   └── src/
│   │       └── controller.cpp
│   │
│   ├── gps/
│   │
│   ├── lidar/
│   │   ├── launch/
│   │   ├── rviz/
│   │   └── src/
│   │
│   ├── main/
│   │   └── launch/
│   │       └── simulator.launch
│   │
│   ├── planning/
│   │   ├── config/
│   │   │   └── local_planner.yaml
│   │   ├── launch/
│   │   │   ├── planner.launch
│   │   │   └── local_planner.launch
│   │   ├── paths/
│   │   │   └── global_path.txt
│   │   ├── rviz/
│   │   ├── scripts/
│   │   │   ├── GlobalPathPlanner.py
│   │   │   ├── PathPlanner.py
│   │   │   ├── StaticObstacleAvoidancePlanner.py
│   │   │   ├── local_planning_core.py
│   │   │   └── VizPlanner.py
│   │   └── tests/
│   │
│   └── simul_msgs/
│       └── msg/
│           └── LocalPlan.msg
│
└── README.md
```

---

# 처음 실행하는 PC

```bash
cd ~/바탕화면/SCMC_2026
source /opt/ros/noetic/setup.bash
catkin_make -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

패키지 확인:

```bash
rospack find morai_udp
rospack find planning
rospack find control
rospack find gps
rospack find camera
```

Python YOLO 환경도 필요합니다.

```bash
python3 -c "import torch, cv2, ultralytics; print('YOLO environment OK')"
```

---

# 네트워크 구성

IP는 고정값이 아니며 실제 MORAI PC와 ROS PC의 네트워크 환경에 맞게 설정해야 합니다.

예시:

| 장치 | 예시 IP |
| --- | --- |
| MORAI 실행 PC | `192.168.0.15` |
| ROS 알고리즘 PC | `192.168.0.2` |

> 두 PC는 동일한 네트워크 대역에서 통신 가능해야 합니다.

주요 UDP Port:

| 항목 | 방향 | MORAI Host Port | ROS Destination Port |
| --- | --- | ---: | ---: |
| Ego Ctrl Cmd | User → Sim | `9093` | `9094` |
| Collision Data | Sim → User | `9091` | `9092` |
| Competition Vehicle Status | Sim → User | `9088` | `9099` |
| Ego Vehicle Status | Sim → User | `9100` | `9111` |
| Object Info | Sim → User | `7605` | `7505` |
| GPS | Sim → User | `1110` | `1111` |
| IMU | Sim → User | `1113` | `1112` |

---

# 권장 실행 방법

현재는 `main/simulator.launch`에서 주요 노드를 한 번에 실행할 수 있습니다.

기본값:

```text
enable_control = false
enable_local_planner = false
rviz = false
start_lidar_driver = true
```

## 기본 경로추종

```bash
roslaunch main simulator.launch \
  morai_ip:=192.168.0.15 \
  status_layout_confirmed:=true \
  enable_control:=true \
  enable_local_planner:=false
```

## Local Planner 포함

```bash
roslaunch main simulator.launch \
  morai_ip:=192.168.0.15 \
  status_layout_confirmed:=true \
  enable_control:=true \
  enable_local_planner:=true \
  rviz:=true
```

이미 `/velodyne_points`를 별도 드라이버나 rosbag에서 받고 있다면:

```bash
roslaunch main simulator.launch \
  morai_ip:=192.168.0.15 \
  status_layout_confirmed:=true \
  enable_control:=true \
  enable_local_planner:=true \
  start_lidar_driver:=false \
  rviz:=true
```

---

# Local Planner 설정

Local Planner 미션 구간은 다음 파일에서 설정합니다.

```text
src/planning/config/local_planner.yaml
```

현재 기본값:

```yaml
static_obstacle_avoidance_planner:
  static_zones: []
  dynamic_zones: []
```

현재는 실제 구간이 비어 있으므로 Local Planner를 활성화해도 미션 구간 밖에서는 기존 `/control_path` 주행을 유지합니다.

waypoint index 방식:

```yaml
static_zones:
  - {start: 1000, end: 1200}
```

또는 UTM 좌표 방식:

```yaml
static_zones:
  - {xmin: 302000.0, xmax: 302100.0, ymin: 4123000.0, ymax: 4123100.0}
```

실제 대회 구간을 확인한 뒤 값을 입력합니다.

---

# 기본 데이터 흐름

```text
MORAI GPS
    ↓
/gps
    ↓
gps_to_utm
    ↓
/current_pose

global_path.txt
    ↓
GlobalPathPlanner
    ↓
/global_path
    ↓
PathPlanner
    ↓
/control_path
    ↓
Controller
    ↓
/control_cmd
    ↓
morai_cmd_controller
    ↓
UDP
    ↓
MORAI IONIQ 5
```

Local Planner 활성 시:

```text
LiDAR
  ↓
/velodyne_points
  ↓
DBSCAN / Object Detection
  ↓
/obstacle_info_static
  ↓
StaticObstacleAvoidancePlanner
  ↓
Frenet candidate generation
  ↓
/local_plan
  ↓
Controller
```

신호등:

```text
MORAI Front Camera
    ↓
/camera/front/image/compressed
    ↓
YOLO
    ↓
/traffic_light_status
```

---

# 주요 토픽 확인

```bash
rostopic hz /gps
rostopic hz /imu
rostopic hz /vehicle_status
rostopic hz /current_pose
rostopic hz /global_path
rostopic hz /control_path
rostopic hz /control_cmd
```

Local Planner:

```bash
rostopic echo /local_plan
rostopic hz /obstacle_info_static
rostopic hz /velodyne_points
```

YOLO:

```bash
rostopic echo /traffic_light_status
rostopic hz /yolo_traffic_light/image_raw
```

영상 확인:

```bash
rqt_image_view
```

YOLO 결과 영상:

```text
/yolo_traffic_light/image_raw
```

---

# Local Planner 테스트

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

OPENBLAS_NUM_THREADS=1 \
python3 -B -m unittest discover -s src/planning/tests -v
```

---

# 주의사항

## `morai_cmd_controller` 중복 실행 금지

`morai_udp_nodes.launch` 내부에서 이미 `morai_cmd_controller`가 실행됩니다.

동일 노드를 따로 실행하면 다음 오류가 발생할 수 있습니다.

```text
new node registered with same name
```

## Local Planner Fail-safe

`enable_local_planner:=true`인 상태에서 다음 문제가 발생하면 차량은 정지하도록 구성되어 있습니다.

- Local Plan timeout
- invalid Local Path
- GPS invalid
- Local Planner stop request
- Local Path point NaN / Inf
- Local Plan frame 오류

## LiDAR TF

Local Planner에서 LiDAR obstacle을 map 좌표계로 사용하려면 올바른 TF가 필요합니다.

```text
map
 ↓
planning_vehicle
 ↓
velodyne
```

센서 장착 위치를 확인하지 않은 상태에서 임의의 LiDAR offset을 사용하지 마십시오.

---

# 현재 남은 작업

1. 실제 대회 코스에 맞는 `static_zones`, `dynamic_zones` 설정
2. Local Planner 실제 MORAI 주행 검증
3. 차량 및 도로 폭 관련 파라미터 실측/확인
4. LiDAR mounting transform 확인
5. 신호등 정지선 및 미션 구간 재검증
6. Green 전환 후 정상 경로 복귀 로직 검증
7. YOLO ROI 및 진행 방향 신호등 선택 로직 개선
8. 동적 장애물 속도 추정 및 정책 고도화
9. 합류 구간 waypoint 재설정
10. 속도 profile 고도화

---

# 현재 목표

```text
Global Path 기본 주행 안정화
        ↓
LiDAR 기반 Local Planning 연동
        ↓
정적 장애물 회피 검증
        ↓
Global Path 복귀 안정화
        ↓
YOLO 신호등 미션 안정화
        ↓
동적 장애물 / 합류 구간 통합
        ↓
전체 코스 통합 주행
```
