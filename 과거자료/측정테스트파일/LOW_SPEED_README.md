# 저속 전역경로 추종 — 2026-09-12

ROS2 Humble 작업공간: `../2026foscar`. 기본 속도 5 km/h, 시작 시 매칭한 경로 진행거리부터
20 m 이동 후 정지한다. 기본 경로는 현재 스폰 도로에서 시작해 공식 전역경로로 합류하는
`spawn_path.txt`다. 신호·장애물 인식 및 GPS 음영 주행은 아직 포함하지 않는다.

## 실행

기존 `sensor_monitor.py`, `measure.py`, `go.py`와 다른 UDP 수신/제어 프로그램은 종료한다.

센서 수신·차량 위치 확인 (차량 제어 송신 없음):

```bash
bash ~/Desktop/2026Morai/onsite/run_low_speed.sh
```

경로와 차량 위치를 RViz에서 함께 확인:

```bash
bash ~/Desktop/2026Morai/onsite/run_low_speed.sh rviz:=true
```

2026-09-12 사용자 확인 및 `Readme/센서설정/2026foscarGPSIMU.json` 기준으로
GPS·IMU 위치/회전은 모두 0이며 기준점은 후륜축 중앙이다. 이를 반영해
`gps_offset_x/y: 0.0`, `imu_yaw_offset_deg: 0.0`, `sensor_mount_confirmed: true`로 설정했다.
IMU roll/pitch 장착이 0이 아닌 경우, 현재 yaw-only 장착 보정을 확장한 뒤 사용한다.

설정: `../2026foscar/src/foscar_bringup/config/low_speed.yaml`

- `gps_offset_x/y`: **후륜축에서 GPS 안테나까지**, 차량 전방 +x / 좌측 +y, m.
  시뮬 센서 설정의 기준점이 다른 경우 기준점 차이를 먼저 반영해야 한다.
- `imu_yaw_offset_deg`: 차량에서 IMU까지 yaw, CCW+, 도.
- 맵: EPSG:32652, 원점 E=302595 / N=4124145.
- 수신: GPS 9102, IMU 9104, Competition Status 9099.
- 송신: 192.168.10.1:9093, 23B payload, cmd type=1 / Auto=2 / D=4.

현재 스폰 위치에서 앞 20m에 장애물이 없는 상태로 실행한다. 공식 출발점으로 옮길 필요 없다.
매 실행 시 현재 위치의 진행거리부터 20m를 새로 계산하므로, 반복 실행하면 추가로 전진한다.

```bash
bash ~/Desktop/2026Morai/onsite/run_low_speed.sh drive:=true speed_kmh:=5.0 distance_m:=20.0 rviz:=true
```

화면: `HANDSHAKE` → `CRUISE` → `FINISH`. FINISH는 브레이크를 유지한다.
`FAULT`도 정지 유지하며 자동 재출발하지 않는다. 원인을 해결하고 런치를 재시작한다.
Ctrl+C 종료 시 브레이크 패킷을 1초 동안 보낸다. 이후 정지 유지 송신은 끝난다.

## 구성

- `foscar_interfaces`: VehicleStatus(속도 m/s, yaw rad, 기어/모드), ControlCommand(페달, 조향 rad).
- `foscar_control/sensor_bridge`: 실제 패킷 검증, 센서 토픽, 별도 스레드 50Hz 송신.
- `foscar_control/localization`: GPS→맵 좌표, IMU yaw, GPS 장착 보정, odometry 및 TF.
- `foscar_control/path_follower`: 순서가 있는 경로 매칭, Pure Pursuit 20Hz, 속도 피드백 제어,
  상태 전환, CSV 로그.
- `foscar_bringup`: 런치, YAML, 경로 복사본, RViz 설정.

부모 폴더는 Git 저장소가 아니며 확인된 다른 저장소는 별도 프로젝트여서,
오늘 구현은 이 작업공간 안에 작성했다. 팀 저장소 위치가 확인되면 패키지를 옮길 수 있다.

## 주요 동작과 한계

- 초기 전역 최근접 매칭 이후 진행거리 주변(-3m/+8m)에서만 매칭한다.
- 원본 4430점 중 연속 중복점을 제거해 4392점, 길이 2184.6m로 읽는다.
- 횡오차 1.5m 초과, 경로 대비 방향차 60° 초과, IMU/상태 yaw 차이 10° 초과 시 정지.
- 제어 명령 만료 0.25초, 센서 만료 0.5초. 송신 스레드는 제어 노드와 독립적으로 검사한다.
- GPS 무효 fix 즉시 송신부 브레이크. GPS/IMU/상태의 반복된 동일 시각은 새 데이터로
  취급하지 않는다. GPS NMEA UTC가 0.1초 단위면 유효 갱신 주기는 수신 패킷 주기보다 낮다.
- Auto/D 상태 확인 후 가속한다. 5초 이내 확인 실패 시 정지한다.
- 최초 시험 제한: 속도 최대 10 km/h, 거리 최대 150m, 액셀 최대 0.25.
- 조향 rad→정규화는 브리지 한 곳에서만 변환한다. 실측 0/15.2/34.9° ↔ 0/0.5/1.0을
  임시 보간한다. 15.2° 점은 17km/h에서 측정된 값이므로 저속 정밀 보정은 미완료다.
- GPS 직접 위치 + IMU 방향 버전이며 EKF/터널 추측항법은 없다. GPS 손실 시 정지한다.
- 차량 제어는 UDP 전달과 송신 프로세스 생존에 의존한다. SIGKILL이나 네트워크 단절에서
  종료 브레이크 전달을 보장하는 기능은 아니다.

## 빌드·검증

현재 노트북에는 ROS2 Humble, colcon, NumPy가 있었고 pyproj 3.7.1을 사용자 환경에 설치했다.
다른 PC에서는 ROS2 Humble 및 패키지 의존성을 먼저 설치한다.

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/2026Morai/ros2_ws
colcon build --symlink-install
source install/setup.bash
python3 -m unittest discover -s src/foscar_control/test -v
python3 src/foscar_control/test/integration_udp.py
```

통합 테스트는 ROS_DOMAIN_ID=73, loopback 주소, 별도 19xxx 포트만 사용한다.
가상 입력으로 실제 ROS 노드를 실행하며 MORAI에 제어를 보내지 않는다.

2026-09-12 확인 결과:

- 3개 패키지 빌드 성공.
- 단위/운동학 테스트 8개 통과.
- UDP 통합 테스트 3개 통과: 지정 거리 정지 / 제어 노드 일시정지 watchdog / GPS 무효 정지.
- 실제 센서 수신 전용 런치 성공. GPS·IMU·Competition Status 파싱 오류 0.
- 실제 수신 당시 안테나 맵 위치 (-12.39, -220.88), 경로까지 약 58.20m.
  이후 리셋 후 GPS가 시작 진행거리 0.47m, 경로 오차 0.008m인 것을 확인했다.
- 장착값은 제공된 저장본과 사용자 설명으로 모두 0임을 확인했다.
  저장본의 GPS 목적지가 192.168.10.1로 되어 있어 노트북 수신용 복사본
  `Readme/센서설정/2026foscarGPSIMU_laptop.json`에서는 GPS 목적지만 192.168.10.2로 수정했다.
  카메라 설정은 원본 그대로다.
- 저장본 확인 후 GPS 9102 및 상태 9099가 일시 미수신이었으나 재연결 후 세 채널 모두 복구됨.
- 19:23 현재 위치 (-14.24, -224.17)는 경로에서 57.50m 떨어져 있어 FAULT로 출발 차단됨.
  제어 송신은 하지 않았으며 출발점 재리셋 후 실제 저속 주행 검증 대기 중이다.

로그: 이 폴더의 `logs/follow_*.csv`. `s_m`, `cross_track_m`, 속도, 조향, 페달,
상태와 정지 사유를 저장한다.


## 현재 스폰 위치 지원 및 실제 주행 결과 (19:50 갱신)

사용자가 공식 출발점으로 스폰을 변경하기 어려워 현재 도로에서 출발하도록 변경했다.
원본 맵 좌표와 공식 전역경로는 그대로 유지한다.

- 기본 `route:=spawn`: MGeo 방향성 도로 연결 + 합류점부터 공식 경로 한 바퀴.
- `route:=global`: 기존 공식 전역경로만 사용.
- 현재 도로: A2256W000172. 입력 스폰 좌표와 도로 중심선 차이 0.025m.
- 공식 경로 합류까지 약 374.5m. 합류 위치 (-79.969, -230.394).
- 도로 노드 연결만 사용하며 차선 변경 링크와 도로 밖 직선 연결은 사용하지 않는다.
- 생성 경로 5214점, 총 2592.45m(스폰 뒤쪽 초기 링크 부분 포함), 최대 점 간격 0.5m.
- 재생성: `python3 onsite/build_spawn_route.py` (프로젝트 루트에서 실행).
- 진입 경로 전체의 신호·장애물 및 완주는 검증하지 않았다. 오늘 시험은 첫 20m다.

실제 MORAI 주행 검증 완료:

| 항목 | 결과 |
|---|---|
| 목표 | 5km/h, 현재 위치에서 20m 후 정지 |
| 경로 진행거리 | 19.763m (설정상 정지 여유 0.25m) |
| 최대 경로 횡오차 | 0.188m |
| 횡오차 p95 | 0.153m |
| 최고 속도 | 5.448km/h |
| 최종 속도 | 약 0km/h |
| 상태 | CRUISE → FINISH, FAULT 없음 |
| 파싱 오류 | GPS/IMU/상태 모두 0 |

근거 로그: `logs/follow_20260912_195033_802911.csv`,
`logs/spawn_drive_console.log`, `logs/spawn_drive_result.json`.
종료 직전 2초간 FINISH 브레이크 유지와 속도 0을 확인한 뒤 런치를 종료했다.
