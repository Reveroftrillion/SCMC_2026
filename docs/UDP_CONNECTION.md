# MORAI UDP 연결 단계

이번 변경은 UDP 센서 입력과 제어 출력 연결을 위한 것입니다. 올해 경로 교체,
미션 인덱스 변환, GPS 음영 위치 추정, 고정 카메라 보정은 포함하지 않습니다.

## 실행 전 설정

- MORAI 센서/차량 상태의 Destination IP는 참가팀 PC의 실제 IPv4 주소입니다.
- 참가팀 수신 bind는 기본 0.0.0.0입니다. MORAI 목적지에 이 주소를 넣지 않습니다.
- MORAI Host IP와 제어 명령 목적지는 MORAI PC의 실제 IPv4 주소입니다.
- 포트는 src/MORAI_UDP_NetworkModule/config/network.yaml에서 수정합니다.
- 카메라 파일은 세 카메라만 포함하므로 GPS/IMU/VLP16을 별도로 설정해야 합니다.

| 항목 | 방향 | 기본 포트 |
|---|---|---|
| 차량 상태 | MORAI → 참가팀 PC | 9082 |
| 제어 명령 | 참가팀 PC → MORAI | 9093 |
| GPS | MORAI → 참가팀 PC | 1111 |
| IMU | MORAI → 참가팀 PC | 1112 |
| Front / Left / Right | MORAI → 참가팀 PC | 9291 / 9293 / 9295 |
| VLP16 | MORAI → 참가팀 PC | 2368 |

카메라 포트는 제공 센서셋의 값입니다. 다른 포트는 개발용 기본값이며 본선 할당을 뜻하지 않습니다.
카메라 위치·각도·FOV는 고정 센서셋을 유지합니다. Ground Truth / BBox는 끕니다.
VLP16 기본 rpm=600은 10Hz에 해당합니다.

## 상태 패킷 확인이 먼저 필요한 이유

현재 보유한 일반 EgoVehicleStatus 구조체 크기는 229바이트입니다.
기본 후보 헤더는 #MoraiInfo$입니다. 대회용 Competition Vehicle Status의 정확한
필드 배치가 이 구조체와 같다는 확인은 아직 없습니다.

따라서 기본 실행은 길이와 앞 11바이트만 진단 로그에 표시하고 /vehicle_status를
발행하지 않습니다. 길이만 같다고 호환이 확인된 것은 아닙니다. 운영측 정의 및
실제 패킷의 필드 위치·속도/각도 단위를 확인한 뒤에만 status_layout_confirmed를
true로 설정합니다. 다르다면 lib/network/parsers.py 및 구조체를 실제 규격에
맞춰 먼저 수정해야 합니다. 짧은 패킷을 0으로 채우거나 필드를 추측해 삭제하지 않습니다.

## Ubuntu / ROS1 실행

ROS와 필요한 패키지를 설치한 참가팀 PC의 워크스페이스 루트에서:

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

# 통신 진단: 제어 UDP 송신 꺼짐, 차량 상태는 패킷 진단만.
roslaunch morai_udp morai_udp_nodes.launch morai_ip:=192.168.0.100
```

주소는 예시입니다. 별도 설정 파일은 network_config:=/absolute/path/network.yaml로
선택할 수 있습니다. enable_gps, enable_imu, enable_cameras로 센서 그룹을 선택합니다.
진단 중 /gps, /imu, /camera/front/image/compressed 등을 확인합니다.

```bash
rostopic hz /gps
rostopic hz /imu
rostopic hz /camera/front/image/compressed
rostopic hz /camera/left/image/compressed
rostopic hz /camera/right/image/compressed
```

상태 규격을 확인했다면 기존 실행을 종료한 뒤:

```bash
roslaunch morai_udp morai_udp_nodes.launch morai_ip:=192.168.0.100 status_layout_confirmed:=true
rostopic echo -n 1 /vehicle_status
```

전체 실행도 기본 제어 송신은 꺼져 있습니다. 단독 통신 launch와 동시에 실행하지 않습니다.

```bash
roslaunch main simulator.launch morai_ip:=192.168.0.100 status_layout_confirmed:=true
```

enable_control:=true를 추가하면 AutoMode / D / cmd_type=1의 UDP 송신을 시작합니다.
초기 입력이 없거나 타임아웃이면 브레이크 명령입니다. 활성화 시점은 현장 출발 절차와
맞춰야 하며, 판정 프로그램 시작을 자동 감지하는 코드는 이번 변경에 없습니다.
종료 시 브레이크 한 번을 보내지만 UDP이므로 도착 보장은 없습니다.

## 구현 내용과 한계

- Receiver: 수신 스레드 1개, 최신 스냅샷, monotonic 수신 시각, 길이 검사,
  오류 로그 제한, 송신 IP 필터, 명시적 종료. 수신 전/타임아웃에는 None.
  SO_REUSEADDR로 포트 중복을 숨기지 않습니다.
- 차량 상태: 확인된 기존 배치에 한해 크기·헤더·tail·유한 값 검증 후 새 패킷만 발행.
- GPS: 체크섬이 있는 GGA/GNGGA에서 좌표와 고도를 함께 발행. RMC는 위치를
  덮어쓰지 않습니다. GGA 무효 fix는 0,0 및 status=0을 발행합니다.
  NMEA가 없거나 손상되면 수신 유효시간이 갱신되지 않습니다.
- IMU: timestamped_115(기본, 로컬 예제)와 legacy_107 중 layout을 명시적으로 선택.
  실제 장비와 일치하는지 확인해야 합니다. quaternion은 정규화합니다.
- 카메라: JPEG 길이, timestamp, 순번, 마지막 조각, 최대 프레임 크기를 확인.
  누락/역순 프레임은 버리고 다음 index=0부터 재시작합니다. 가변 길이와
  65000바이트 패딩 패킷을 처리합니다. 실제 MORAI 패킷으로 추가 검증이 필요합니다.
- ROS 메시지 시간은 로컬 발행 시각입니다. 시뮬레이터와 정밀 시간 동기화하지 않습니다.
- 제어 송신: 기본 비활성. 활성 시 50Hz wall-clock 송신. 명령 0.3초,
  차량 상태 또는 GPS 메시지 0.5초 초과 시 accel=0 / brake=1 / steer=0.
  유효한 blackout 보고와 GPS 패킷 수신 중단을 구분합니다.
  NaN/Inf 명령은 거부하고 범위 제한·브레이크 우선 처리를 합니다.
- 조향: controller.cpp의 경로 조향만 radians / radians(max_steering_deg)로 변환.
  차선 PID 출력은 정규화 입력으로 유지하고 송신 경계에서 -1~1로 제한합니다.
  기본 최대각은 규정의 40도이며 control/max_steering_deg로 설정합니다.
  실제 차량에서 좌우 부호와 응답을 확인해야 합니다.
- GPS 위치 노드: 0,0/비정상 GPS를 UTM으로 변환하지 않고, 신선한 차량 yaw를
  자세로 사용합니다. IMU 융합·음영 구간 추측항법을 구현한 것은 아닙니다.
- Front 영상을 YOLO와 차선 노드에 remap합니다. 차선 ROI/원근 변환은 아직
  기존 설정이므로 고정 카메라에 대한 보정 없이 주행 성능을 보장하지 않습니다.
- YOLO 가중치 경로는 패키지 상대 경로로 변경했습니다. 기존 CPU 설정은 유지합니다.
- 기존 Sensor/GPS.py, Camera.py 등의 단독 표시 예제 대신 sensor_udp_node.py를
  사용합니다. 예전 가변 길이 센서 예제는 엄격해진 Receiver와 직접 호환되지 않습니다.

## 변경 파일

| 파일 | 변경 |
|---|---|
| morai_udp/lib/network/UDP.py | 수신/송신 수명과 최신 데이터 관리 |
| morai_udp/lib/network/parsers.py | GPS/IMU/카메라/상태/명령 검사 |
| morai_udp/EgoNetwork/Publisher/competition_info_node.py | 상태 진단과 발행 |
| morai_udp/EgoNetwork/CmdControl/morai_cmd_controller_node.py | 주기 송신과 watchdog |
| morai_udp/Sensor/sensor_udp_node.py | 공통 센서 ROS 노드 |
| morai_udp/config/network.yaml | 포트/토픽/타임아웃/IMU 배치/최대 조향각 |
| morai_udp/launch/morai_udp_nodes.launch | 센서별 인스턴스 및 실행 인자 |
| morai_udp/CMakeLists.txt, package.xml | 실행 등록, 공유 lib 설치, 의존성 |
| main/launch/simulator.launch | 전체 설정 전달 및 카메라 remap |
| lidar/launch/object_detection_crossing.launch | device_ip/port/rpm 전달 |
| control/src/controller.cpp, include/control/controller.h | 조향 단위 및 초기값 |
| gps/scripts/gps_to_utm.py, CMakeLists.txt, package.xml | GPS 유효성·차량 yaw 연결 |
| camera/scripts/traffic_yolo.py, package.xml | 가중치 경로 및 rospkg 의존성 |
| morai_udp/tests/test_udp.py | 파싱/누락/타임아웃/실제 loopback UDP 테스트 |

표의 morai_udp 실제 디렉터리는 src/MORAI_UDP_NetworkModule입니다.

## 검증

```bash
python -B -m unittest discover -s src/MORAI_UDP_NetworkModule/tests -v
```

Windows Python 환경에서 테스트 13개 통과. Python 3.8 문법, XML 및 YAML 검사 통과.
ROS/catkin과 C++ 컴파일러가 없어 ROS 메시지 생성, launch 실행, C++ 빌드,
MORAI 실제 패킷 송수신 및 주행은 아직 검증하지 않았습니다.
