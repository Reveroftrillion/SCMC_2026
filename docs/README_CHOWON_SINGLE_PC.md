# 한 PC에서 MORAI chowon 시나리오 실행

MORAI Simulator와 ROS Noetic을 같은 Ubuntu PC에서 실행하는 절차다.
현재 수정한 작업 폴더는 `SCMC_2026-feat-local-planner`다.
다른 복사본인 `SCMC_2026-local-planner`를 실행하면 수정 사항이 적용되지 않을 수 있다.

## 1. MORAI에서 chowon 로드

1. MORAI Simulator를 실행한다.
2. 시나리오 로드 화면에서 저장된 `chowon` 시나리오를 선택해 로드한다.
   목록에 없으면 해당 시나리오 파일을 먼저 준비한다. 이 launch는 시나리오를 자동으로 로드하지 않는다.
3. 차량의 시작 위치와 주행 방향을 확인한다.
   ROS는 `src/planning/paths/global_path.txt`의 고정 경로를 사용하므로 시나리오의 도로와 시작 위치가 이 경로에 맞아야 한다.
4. 아래 네트워크·센서 설정을 확인하고 적용한다. 시나리오 로드 후 값이 바뀌었는지도 확인한다.
5. 시뮬레이션 일시정지를 해제하고, Ego 차량이 외부 UDP 제어 명령을 받는 모드로 설정한다.

## 2. 같은 PC의 네트워크 설정

Simulator와 ROS 모두 같은 PC이므로 IP는 `127.0.0.1`을 사용한다.
Simulator에서 송신하는 데이터의 Destination IP도 `127.0.0.1`로 맞춘다.

| 데이터 | MORAI 설정 / ROS 설정 |
|---|---|
| Ego Ctrl Cmd | MORAI Host 포트 `9093`, Destination 포트 `9094`; ROS는 `9094`에서 `9093`으로 명령 송신 |
| Ego Vehicle Status | MORAI Host 포트 `9100`, Destination 포트 `9111`; ROS 수신 `9111` |
| GPS | MORAI 송신 Destination 포트 / ROS 수신 `1111` |
| IMU | MORAI 송신 Destination 포트 / ROS 수신 `1112` |
| 전방 카메라 | MORAI 송신 Destination 포트 / ROS 수신 `9291` |
| 좌측 카메라 | MORAI 송신 Destination 포트 / ROS 수신 `9293` |
| 우측 카메라 | MORAI 송신 Destination 포트 / ROS 수신 `9295` |
| 신호등 카메라 | **MORAI Host `9090`, Destination `9297`; ROS 수신 `9297`** |
| VLP-16 LiDAR | MORAI 송신 Destination 포트 / ROS 수신 `2368` |

카메라는 현재 수신 parser에 맞는 영상 UDP 송신을 사용한다. GT 데이터는 영상 parser가 받지 않는다.
센서 송신을 활성화하고 각 카메라의 방향도 확인한다. 전방 카메라는 도로와 정지선, 신호등 카메라는 접근하는 신호등이 보여야 한다.

**같은 PC에서 카메라 Host와 Destination을 둘 다 9090으로 두지 않는다.**
이 PC에서는 Simulator가 `127.0.0.1:9090`을 점유해 ROS의 `0.0.0.0:9090` 수신과 충돌한 이력이 있다.
GPS/IMU/다른 카메라의 Host 포트도 ROS 수신 포트와 중복 점유하지 않도록 확인한다.

## 3. ROS 환경과 빌드

의존성이 설치된 PC에서 아래 명령을 실행한다. 새 PC는 루트 README의 설치 절차를 먼저 따른다.

```bash
cd /home/foscar/바탕화면/SCMC_2026-feat-local-planner
source /opt/ros/noetic/setup.bash
catkin_make -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
rospack find main
python3 -c "import rospy, cv2, torch, ultralytics; print('Python imports OK')"
```

`rospack find main` 결과가 현재 폴더의 `src/main`인지 확인한다.
가상환경을 별도로 만들지 않았다면 존재하지 않는 `.venv-yolo/bin/python` 경로를 지정하지 않는다.

## 4. 전체 실행

이전에 실행한 simulator/카메라 테스트 launch는 해당 터미널에서 `Ctrl+C`로 종료한다.
MORAI의 `chowon` 로드와 센서 송신 설정을 마친 뒤 아래 명령을 실행한다.

```bash
cd /home/foscar/바탕화면/SCMC_2026-feat-local-planner
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch main simulator.launch \
  morai_ip:=127.0.0.1 \
  traffic_camera_port:=9297 \
  yolo_python:=/usr/bin/python3 \
  enable_control:=true \
  status_layout_confirmed:=true \
  enable_local_planner:=false \
  enable_stop_line_detector:=true \
  view_image:=true
```

`status_layout_confirmed:=true`는 현재 검증한 Ego Vehicle Status 패킷 배치를 사용하는 환경용이다.
다른 Simulator 버전이나 패킷 설정에서는 배치를 먼저 확인한다.
`roslaunch`가 필요한 ROS master를 시작하므로 별도로 `roscore`를 실행할 필요는 없다.

시작 로그에서 `/camera_traffic_udp/port: 9297`을 확인한다.
영상 창에는 `/yolo_traffic_light/image_raw`와 `/stop_line/image_raw`가 표시된다.

## 5. 현재 신호등 제어 동작

1. 제어 노드 시작 후 실제 시간 **15초 동안** 신호등 감속·정지를 적용하지 않는다. 영상 인식은 계속 표시된다.
2. 서로 다른 프레임에서 빨간불을 **3회 연속 확인**하면 정지선 탐색에 들어간다.
3. 탐색·접근 목표 속도는 **10km/h**다. 장애물이나 다른 정지 조건은 이보다 낮추거나 정지시킬 수 있다.
4. 정지선은 원본 720p 기준 약 **8픽셀 이상의 두께**, 최근 **3프레임 중 2회 검출**을 요구한다.
5. 현재 전방 카메라에는 거리 보정값이 없어 화면 위치 `image_y >= 0.75`에서 최대 제동한다.
   실제 정지선 2m 앞 정차를 보장하는 보정값은 아니다. 실제 차량은 제동거리만큼 더 이동한다.
6. 정지선을 못 찾고 20m를 탐색하거나, 검출한 정지선을 0.35초 넘게 놓치면 정지한다.
7. 정지 후에는 허용된 초록 신호(`4green`, `4greenleft`)를 서로 다른 프레임에서 3회 확인하면 출발한다.
   경로·장애물 정지 조건이 있으면 출발하지 않는다. 초록불이 `UNKNOWN/MISSING`이면 정지를 유지한다.

설정 파일:

- `src/control/config/camera_traffic_control.yaml`: 15초 대기, 신호 확인 횟수, 접근 속도, 정차 기준
- `src/camera/config/stop_line_front.yaml`: 검출 ROI, 최소 두께, 2/3프레임 확정
- `src/MORAI_UDP_NetworkModule/config/network.yaml`: GPS/IMU/전방·좌·우 카메라와 차량 통신
- `src/main/launch/simulator.launch`: 신호등 카메라 수신 포트와 실행 구성

설정 변경 후에는 launch를 재시작한다. C++ 코드 변경 시에는 먼저 다시 빌드한다.
로컬 플래너는 위 실행에서 비활성화한다. 별도 시험 시 `enable_local_planner:=true`로 켤 수 있으나,
`src/planning/config/local_planner.yaml`의 zone/센서 TF가 실제 시나리오에 맞는지 먼저 확인한다.

## 6. 확인 및 문제 해결

다른 터미널에서도 먼저 현재 프로젝트 환경을 불러온다.

```bash
cd /home/foscar/바탕화면/SCMC_2026-feat-local-planner
source devel/setup.bash
rostopic echo /traffic_mission_state
```

주요 상태는 `STARTUP_DELAY`, `CONFIRM_RED`, `SEARCH_LINE`, `APPROACH_IMAGE`,
`WAIT_SIGNAL`, `WAIT_GREEN`, `SIGNAL_GO`다.

| 증상 | 확인할 내용 |
|---|---|
| `Address already in use` | 아래 명령으로 점유자를 확인한다. Simulator Host와 ROS 수신 포트, 중복 launch를 확인한다. |
| `No such file or directory`와 `.venv-yolo` 경로 | 위 실행 명령의 `yolo_python:=/usr/bin/python3`을 사용하고 패키지 import를 확인한다. |
| `NO_STOP_LINE` | 빨간불 이후 탐색 한도까지 정지선을 못 찾은 상태다. 전방 영상, ROI, 두께 기준을 확인한다. |
| `STOP_LINE_LOST` | 검출한 선이 사라져 정지한 상태다. 얇은 선 오검출 가능성도 확인한다. |
| `UNKNOWN`, `MISSING`, `NO_TARGET` | 신호등 영상에 대상이 실제로 보이는지와 검출 박스·카메라 방향을 확인한다. |
| `invalid camera header/length` | 해당 카메라의 영상 UDP 형식, GT 송신 여부, Destination 포트를 확인한다. |
| `VEHICLE_STALE` | Ego Vehicle Status 송신과 Destination `9111`을 확인한다. |
| 수정 전 동작이 계속됨 | `rospack find main`으로 실행 중인 작업 폴더를 확인하고 다시 source한다. |

```bash
sudo lsof -nP -iUDP:9090 -iUDP:9297
rostopic hz /camera/traffic/image/compressed
rostopic hz /camera/front/image/compressed
rostopic hz /vehicle_status
rostopic hz /gps
```

종료할 때는 roslaunch 터미널에서 `Ctrl+C`를 누른다.
시나리오를 처음부터 다시 시험할 때는 ROS launch도 재시작해야 15초 대기와 신호 확인 상태가 초기화된다.
