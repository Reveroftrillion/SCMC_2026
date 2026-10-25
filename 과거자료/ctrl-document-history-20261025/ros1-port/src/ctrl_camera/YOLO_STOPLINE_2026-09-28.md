# 카메라 7 ONNX 신호 정지 — 2026-09-28

2026-09-29 부터 main 에 들어 있고 대회 런치의 기본값(`camera:=true`)이다. 카메라는 7번 하나만 쓴다.

## 실행

이 브랜치를 받은 워크스페이스에서 기존 ROS2 Humble 의존성을 준비하고 실행합니다.

```bash
python3 -m pip install --target .camera-deps -r src/ctrl_camera/requirements-onnx.txt
source /opt/ros/humble/setup.bash
colcon build --symlink-install
# 제어 송신 없이 관측 (기본)
bash run_camera.sh
# 사용자가 시험할 때만 자동 주행 시작
bash run_camera.sh drive:=true route:=global speed_kmh:=10.0 rviz:=true
```

`run_camera.sh`는 통합 competition 런치를 drive:=false 로 실행합니다(대회는 `run_competition.sh`, 카메라 기본 켜짐). 모델은 `ctrl_camera/models/best.onnx`로 설치됩니다. CPUExecutionProvider를 사용합니다.
옛 HSV `traffic_light_node`·`stop_line_node`·좌우/전방 카메라는 2026-09-29 에 삭제했습니다. `/behavior/target` 발행자는 `behavior_node` 하나입니다.

카메라 7 RGB 포트 9208, pitch 335°, 1280×720 수집 조건을 사용합니다. 포트 변경: `camera_port:=9208`.
카메라 8 semantic이나 시뮬 신호 정답 API는 주행에 사용하지 않습니다.

## 동작

- 현재 전역경로의 다음 정지점과 촬영 시작점 사이에서 해당 교차로 ROI의 YOLO 검출을 사용합니다.
- 기본 confidence 0.4, 서로 다른 처리 프레임에서 2회 같은 색이 나와야 새 상태로 채택합니다. 인식 설정은 `launch/ctrl_yolo_signal.launch.py`에 있습니다.
- 빨강/노랑은 정지. 직진 경로는 green/green_left 통과, 좌회전 경로는 left/green_left/red_left 통과합니다. 좌회전 경로에서 직진 초록만 있으면 정지합니다.
- 미검출·모순된 복수 색상·일시적 추론 실패는 새 판단을 만들지 않습니다. 마지막 확정 색을 시간 만료 없이 유지합니다. 첫 관측 전에는 신호 미검출만을 이유로 정지하지 않습니다.
- 다른 교차로로 넘어가거나 경로 재수신·큰 위치 점프/역방향 이동 시 신호 기억을 초기화합니다. 다른 교차로의 녹색을 가져오지 않습니다.
- 1초보다 오래된 프레임/관측은 새 상태로 채택하지 않습니다. latest-frame 슬롯 하나를 사용해 추론 대기열 누적을 막습니다.
- 타 장애물/보행자/미션 속도·정지·횡방향 목표는 유지하고 신호 정지 목표를 최소값으로 합칩니다.

## 정지 위치

`config/yolo_stoplines.json`은 촬영 당시 사용자가 확정한 **후륜축 정지 자세 s**입니다. 제어기 DrivingTarget.stop_s는 앞바퀴 기준이므로 3m를 더해 넘깁니다. 목표 바로 앞 0.3m부터는 정지 유지를 위해 v_max=0을 함께 적용합니다.

| 교차로 | 촬영 시작 s | 후륜축 정지 s | 현재 경로 방향 |
|---|---:|---:|---|
| 1 | 169.008 | 204.086 | 직진 |
| 2 | 271.991 | 311.475 | 직진 |
| 3 | 407.161 | 432.519 | 직진 |
| 4 | 569.056 | 605.322 | 좌회전 |

방향은 현재 경로에서 정지점 전 5m와 후 30m의 진행각 변화로 확인했습니다 (약 0°, 4°, 0°, 91°). 현재 경로 전용 설정입니다. global_path 파일 SHA256이 달라지면 잘못된 s를 쓰지 않도록 시작을 거부합니다. 경로를 바꾸면 정지점 투영과 maneuver를 다시 확인해야 합니다.
ROI는 촬영 설정의 교차로별 범위이며 모든 방향의 신호등을 지도와 정밀 대응하는 알고리즘은 아닙니다. 같은 ROI의 검출 색이 서로 다르면 이전 상태를 유지합니다. 3번의 두 대상이 같은 색이면 하나의 상태로 처리합니다.

## 사용자 시험 순서

1. `drive:=false`로 카메라 디버그 영상과 신호를 확인합니다.
2. 정지점 앞에서 10km/h로 접근하여 빨강·노랑 정지 위치를 확인합니다.
3. 빨강 관측 후 카메라 일시 미검출에도 정지 유지, 초록 관측 후 미검출에도 통과 판단 유지를 확인합니다.
4. 4번 교차로의 직진 초록 정지 / 좌회전 또는 빨강+좌회전 통과를 확인합니다.
5. 장애물·보행자 정지가 신호의 녹색으로 풀리지 않는지 확인합니다.

```bash
ros2 topic echo /perception/traffic_signal
ros2 topic echo /behavior/signal_stop
ros2 topic echo /behavior/target
ros2 topic info /behavior/target
```

실측 CPU 추론(i9-13900H)은 ONNX threads=4에서 약 0.26초/장입니다(threads=2는 0.43초, 다른 작업이 CPU를 쓰면 0.86초까지 늘어 1초 초과 결과가 모두 버려졌습니다 — 2026-09-28 부하 시험에서 빨간불 미인식). 주행 중 교차로 구간 추론은 약 2.8회/초입니다. 카메라 수신 10Hz가 추론 10Hz를 뜻하지 않습니다. 현재 신호가 보이는 구간 길이는 약 25~40m입니다. 고속에서 늦게 확인한 노랑/빨강에 대해 정지선 이전 정지를 물리적으로 보장하지 않습니다. 기본 제어기 감속값을 변경하지 않았으며, 실제 속도별 정지 오차·브레이크 성능은 사용자 시뮬 시험에서 확인해야 합니다.

검증: 신호 정책·좌표/NMS 및 기존 control/planning 회귀 테스트 총 **587개 통과**, ROS 패키지 빌드, 격리된 로컬 ROS 도메인에서 저장 RGB→ONNX→신호 토픽→판단 정지점 연결 시험. 시뮬 주행/UDP 제어 송신 시험 미실시.
