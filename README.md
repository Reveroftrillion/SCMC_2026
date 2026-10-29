# SCMC 2026 — MORAI 시험

- 권장 시험 대상: `competition_minimal/` · 기존 프로그램: `src/` ([기존 설명](docs/LEGACY_README.md))
- 실제 MORAI 완주는 미검증이며 기본 제어 UDP 송신은 비활성입니다.
- 보행자 시작점, 신호등 4, 회전교차로 경계는 미검증으로 해당 구간 앞에서 정지합니다. 자동 회피도 비활성입니다. 검증 없이 설정을 우회하지 마세요.

## 1. 설치와 빌드

Ubuntu 20.04, ROS Noetic, Python 3.8.10 및 MORAI가 필요합니다.

```bash
git clone https://github.com/Reveroftrillion/SCMC_2026.git
cd SCMC_2026
sudo apt update
sudo apt install -y python3-pip python3-numpy python3-scipy python3-pyproj \
  python3-opencv python3-yaml ros-noetic-velodyne-pointcloud ros-noetic-velodyne-driver
python3 -m pip install --user -r src/camera/requirements-yolo.txt
source /opt/ros/noetic/setup.bash
catkin_make -j2 --only-pkg-with-deps simul_msgs morai_msgs -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
export ROS_PACKAGE_PATH="$PWD/competition_minimal:${ROS_PACKAGE_PATH:-}"
rospack find competition_minimal
```

`competition_minimal`은 저장소 루트에 있고 `run.sh`가 ROS 패키지 경로를 설정합니다. `src/`에 중복 링크를 만들지 마세요. 모델은 `competition_minimal/weights/1027_40epoch.pt`에 있습니다. 빌드 또는 모델 로딩 오류가 나면 다음 단계로 진행하지 마세요.

## 2. MORAI 설정과 수신 전용 실행

MORAI의 송신 대상 IP를 Ubuntu PC 주소로 맞추고 아래 IP를 실제 MORAI PC 주소로 변경하세요. 기본 UDP 포트는 GPS 1111, IMU 1112, Competition Status 909, Camera-7 9297, LiDAR 2368, 제어 출발 9094 / 목적 9093입니다. 설정이 다르면 `gps_port:=`, `imu_port:=`, `status_port:=`, `camera_port:=`, `lidar_port:=`, `command_source_port:=`, `command_port:=`를 실행 명령에 추가하세요. 포트 909는 권한이 필요할 수 있으므로 MORAI와 함께 사용 가능한 포트로 변경하세요.

```bash
export MORAI_IP=192.168.0.15  # 실제 MORAI PC IP로 변경
ip -br addr
ss -lun
bash competition_minimal/run.sh morai_ip:="$MORAI_IP" drive:=false
```

기존 제어 프로그램과 동시에 실행하지 마세요. 시작 전 preflight는 의존성, 모델, 경로·설정과 UDP 바인딩을 검사합니다. 실제 수신을 증명하는 검사는 아닙니다. `drive:=false`에서는 제어 UDP를 전송하지 않습니다.

## 3. 다른 터미널에서 수신 확인

```bash
cd ~/SCMC_2026  # 실제 클론 위치로 변경
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_PACKAGE_PATH="$PWD/competition_minimal:${ROS_PACKAGE_PATH:-}"
python3 competition_minimal/scripts/live_status.py --seconds 10
python3 competition_minimal/scripts/diagnose_gates.py
python3 competition_minimal/scripts/geometry_audit.py
rostopic echo -n 1 /vehicle_status
rostopic echo -n 1 /competition/progress
rostopic echo -n 1 /competition/control_cmd
```

`live_status.py`에서 GPS·IMU·차속·카메라·LiDAR와 위치·진행거리·정지 판단이 `FRESH`인지 확인합니다. 실행 터미널의 `Competition status` 로그에서 181바이트 패킷의 속도·yaw·조향·Gear·Mode, `Traffic PyTorch`와 `Traffic detection`/`Traffic decision`에서 모델 로딩과 신호 인식을 확인하세요. 신호 구간 밖에서는 검출 로그가 없을 수 있습니다. `drive=false`의 제어 상태는 `live_status.py` 출력 `UDP control sender configured: False`와 네트워크 캡처로 확인하세요. `/competition/control_cmd` 발행 자체는 UDP 송신을 뜻하지 않습니다.

`MISSING`/`STALE`이면 MORAI IP, 방화벽, 포트, 센서 설정과 `ss -lun`의 충돌을 확인합니다. `Rejected status packet`이면 181바이트 Competition Status와 시간값을 확인하고, YOLO가 실패하면 모델 경로와 Python 의존성을 확인하세요. 로그는 실행 터미널 및 `~/.ros/log/`에 있습니다. 모델 실패나 영상 누락은 UNKNOWN 정지입니다.

## 4. 주행 전 확인 및 저속 시험

실제 패킷 필드와 시간 증가, Auto/D, GPS 위치 기준점, IMU 방향·보정, LiDAR TF `(2.0, 0.0, 2.11)`, Camera-7 ROI, 도로 경계와 정지선, 제동·수동 인계를 현장에서 확인하세요. `status_layout_confirmed`는 launch 인자이며 `sensor_reference_verified`는 `competition_minimal/config/vehicle_sensor.yaml`의 설정값입니다. `missions.yaml`의 미검증 정지와 `allow_avoidance: false`는 실제 검증 전 유지합니다.

수신 전용 프로세스를 Ctrl+C로 종료한 뒤 검증된 센서 설정에서 최대 속도를 낮춰 통제된 장소에서 실행합니다.

```bash
cp competition_minimal/config/vehicle_sensor.yaml /tmp/competition_sensor_low.yaml
# 검증 후 sensor_reference_verified: true, max_speed_kmh: 10.0으로 직접 수정
nano /tmp/competition_sensor_low.yaml
bash competition_minimal/run.sh morai_ip:="$MORAI_IP" drive:=true \
  status_layout_confirmed:=true sensor_config:=/tmp/competition_sensor_low.yaml
```

저속 시험에서 조향 방향·가속·제동·신호 정지와 재출발, 제어 전환을 확인하세요. 미검증 설정은 주행을 막거나 해당 구간 앞에서 정지시킵니다.

## 5. 전체 코스 시험과 종료

저속 시험과 미션·도로 검증을 통과한 뒤 검증된 센서 설정을 사용합니다. 미검증 구간 및 회피 설정은 실제 근거가 있을 때만 수정하세요.

```bash
cp competition_minimal/config/vehicle_sensor.yaml /tmp/competition_sensor_full.yaml
# 현장 검증 후 sensor_reference_verified: true로 직접 수정
nano /tmp/competition_sensor_full.yaml
bash competition_minimal/run.sh morai_ip:="$MORAI_IP" drive:=true \
  status_layout_confirmed:=true sensor_config:=/tmp/competition_sensor_full.yaml
```

정상 종료 시 정지 후 P/Manual 인계를 시도합니다. 실제 인계는 MORAI에서 확인해야 합니다. 수동 종료 시 Ctrl+C를 누르고 MORAI에서 차량 정지와 제어권을 직접 확인하세요. 긴급 시 MORAI 수동 제어로 즉시 인계하세요.
