# 다른 PC에서 이어하기 — 신호등 YOLO / 위치 재측정

2026-09-27 기준. **다음 할 일은 촬영 대기 위치와 정지선 참고 위치 재측정**입니다.
아직 날씨별 본 수집이나 YOLO 학습 완료 상태가 아닙니다. 먼저 이 문서를 읽으세요.

## 1. 현재 상태와 다음 순서

1. 새 PC에 코드를 받고 ROS 패키지를 빌드합니다.
2. MORAI 시나리오·센서·네트워크를 복구합니다.
3. `survey_points.sh`로 기존 경로를 5km/h 자동 주행하며 직접 정차·위치 기록합니다.
4. 새 기록으로 정지선과 촬영 위치를 검수·구분합니다.
5. 그 이후 Sunny/Foggy × 11/13/15시의 6조건을 수집합니다.
6. 자동 라벨 검수 → 데이터 분할 확인 → YOLO26n 학습 → 같은 데이터로 YOLO26s 비교.

- 이전 dataset/out 시험 데이터와 survey 3개 세션은 사용자 요청으로 휴지통에 옮겼습니다.
  폐기한 0002/0003 좌표는 재사용하지 않습니다. 이후 새로 측정한 데이터는 별도 확인하세요.
- `src/foscar_camera/config/stop_lines.csv`는 **부정확한 옛 표**입니다. 유효한 정지선으로 간주하지 마세요.
- survey는 옛 표를 읽지 않습니다. 단, 기존 `collect_label.sh` 자동 주행과 extract의 분할은 옛 표를 사용합니다.
- **새 survey CSV를 자동 정차 경로에 연결하는 기능은 아직 없습니다.** 재측정·검수·좌표 보정 후 별도 구현이 필요합니다.

## 2. 다른 PC 준비

Ubuntu 22.04 + ROS 2 Humble 환경을 전제로 합니다. 아래 경로는 새 PC의 홈 기준입니다.

```bash
git clone https://github.com/26foscar-notebook/2026foscar_morai.git ~/2026foscar
cd ~/2026foscar
# 이미 clone했다면 해당 폴더에서 git pull --ff-only
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pytest \
  python3-numpy python3-opencv python3-yaml python3-pyproj \
  ros-humble-message-filters ros-humble-tf2-ros
# rosdep 초기화가 안 된 PC는 최초 한 번 sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
python3 -m pytest -q src/foscar_camera_train/test
```

기존 PC의 build/install/log 또는 Python 가상환경을 복사하지 말고 새 PC에서 빌드합니다.
GUI가 필요하므로 일반 데스크톱 터미널에서 실행합니다. headless SSH 전용 프로그램이 아닙니다.

### Git만으로 옮겨지지 않는 것

- MORAI PC의 `parking_scenario` 및 **현재 수정한 센서·네트워크 설정 파일**: 별도 저장/복사.
- 이 Git 저장소 바깥에 있던 임시 규정집 1.1.5, 주최측 샘플 JSON: 별도 복사.
- `~/foscar_logs/`의 새 실측 사진·좌표, 학습 이미지와 가중치: 별도 파일 전송.
- 이 저장소에는 이번 작업으로 새 학습 가중치를 추가하지 않았습니다.

## 3. MORAI와 네트워크

기존 프로그램 모두 종료 → parking_scenario 로드 → Keyboard + P → 센서/네트워크 복구 → Connect.
시나리오 로드 후 연결이 풀리거나 설정이 바뀔 수 있어 매번 확인합니다.

- MORAI PC: `192.168.0.11`
- 개발 PC: `192.168.0.10` (다른 PC에서도 같은 주소를 쓰려면 기존 PC와 중복되지 않도록 분리)
- 새 주소를 쓴다면 MORAI 센서의 Destination IP를 새 개발 PC IP로 바꿉니다.
- **최신 main의 low_speed.yaml은 다른 PC의 `192.168.1.2`가 기본값입니다. 반드시**
  `src/foscar_bringup/config/low_speed.yaml`의 `sensor_bridge.sim_ip`를 현재 MORAI IP로 맞춥니다.
  이 대화의 환경에서는 `192.168.0.11`로 바꿉니다. 다른 팀 PC의 기본값을 그대로 쓰지 마세요.

| 통신 | 수신 측 | 포트 |
|---|---|---:|
| GPS | 개발 PC | 9102 |
| IMU | 개발 PC | 9104 |
| Competition Vehicle Status | 개발 PC | 9099 |
| RGB 카메라 7 | 개발 PC | 9208 |
| Semantic 카메라 8 | 개발 PC | 9210 |
| Ego Ctrl Cmd | MORAI PC | 9093 |

위 표는 **수신 포트**입니다. 송신 Host Port와 혼동하지 마세요. 실제 사용 중인 송신 포트는 저장된 네트워크 설정을 따릅니다.
일반 Ego Vehicle Status는 현재 코드의 입력이 아닙니다. Competition과 같은 목적지 포트로 동시에 보내지 마세요.

## 4. 확정 카메라 설정

| 항목 | 7번 | 8번 |
|---|---|---|
| Ground Truth | None (RGB) | Semantic |
| 위치 X/Y/Z (m) | 0.40 / 0 / 1.80 | 동일 |
| Roll/Pitch/Yaw (deg) | 0 / 0 / 0 | 동일 |
| 해상도 | 1280×720 | 동일 |
| Horizontal FOV | 70° | 동일 |
| Frame Rate | 10Hz | 동일 |
| Viz Bounding Box 2D/3D | OFF | OFF |
| Enable Capture Sensor | OFF | OFF |

7·8번은 같은 장면을 정렬해서 RGB의 신호등 박스를 자동 생성하려는 구성입니다.
10Hz는 영상 송신 빈도이고, 연속 수집기의 저장 간격(현재 정차 중 기본 2초)과는 다릅니다.
카메라 1번은 기존 전방 카메라이며 survey 화면은 7번입니다.
GPS/IMU 장착 위치는 기존 베이스라인 센서셋을 유지하고, 변경 시 localization 오프셋과 일치시켜야 합니다.

## 5. 오늘 실행 — 자동 주행 + 영상 + 위치 기록

```bash
cd ~/2026foscar
./tools/survey_points.sh
```

스크립트가 ROS 환경을 불러옵니다. 별도 진단·카메라 런처를 같이 실행하지 않습니다.
시작 시 Auto/D 전환을 요청하되 브레이크를 유지합니다. 영상 창이 자동으로 열립니다.
**키 입력은 영상 창이 아니라 실행 터미널을 클릭한 뒤 합니다.**

| 입력 | 동작 |
|---|---|
| g + Enter | 5km/h 경로 주행 / 재출발 |
| Enter만 | 정차 요청 → 실제 정지 1초 확인 → capture 저장 |
| 소문자 l + Enter | 정차 요청 → stopline_reference 저장 |
| q + Enter | 정차 절차와 전체 종료 |

- 신호·장애물 자동 대응은 없습니다. 충분히 앞에서 Enter를 누르세요.
- Enter를 누른 순간이 아니라 **실제 멈춘 위치**가 저장됩니다.
- 저장 후에는 정차 유지. `저장:` 메시지를 보고 g로 재출발합니다.
- 카메라 창을 닫거나 창에서 q를 누르면 전체 런처도 종료됩니다.
- 운전 중 MORAI Q/P로 전환하면 기존 추종기가 FAULT가 될 수 있습니다.
- 우선 한 지점만 시험하고 사진·좌표를 확인합니다. 실제 정차 정확도는 자동 테스트만으로 보장되지 않습니다.

### 두 위치의 차이

- capture: 신호등 전체가 잘 보이는 **촬영 대기 위치**. 정지선 자체가 화면에 보일 필요는 없습니다.
- stopline_reference: 우리 경로에서 신호에 따라 멈춰야 하는 정지선의 **참고 차량 위치**.
  앞바퀴가 선을 넘지 않게 맞추고 외부 측면 시점으로 확인합니다.
- X/Y와 s는 코드상 **후륜축 기준**입니다. l을 눌러도 정지선 좌표를 자동 산출하지 않습니다.
  앞바퀴와 선 사이 간격, 축거, 진행 방향/경로 투영을 반영해 검수·보정해야 합니다.
- 시작 위치 s를 임의로 0으로 바꾸지 않습니다. 전체 경로와 같은 기준을 유지합니다.

### 출력

```text
~/foscar_logs/survey/날짜_시각/
  points.csv                 # ID, 종류, X/Y, s, yaw_rad, reference
  launch.log                 # 센서/추종기/GUI 로그
  0001_capture/
    rgb.jpg                  # 실제 수신 형식에 따라 .png 가능
    sem.jpg
    point.json               # 후륜축 좌표·s·방향·카메라 타임스탬프
  0002_stopline_reference/
    ...
```

GPS 추측항법 중, 데이터가 0.5초 이상 오래됨, 움직이는 중, RGB/Semantic 시각 차이 5ms 초과 시 저장을 기다립니다.
좌표·사진은 정차 상태의 최신 수신값이며, 모든 센서를 하나의 시뮬 타임스탬프로 정밀 동기화하는 구현은 아닙니다.

## 6. 문제별 확인

| 증상 | 확인 |
|---|---|
| g를 눌러도 안 움직임 | launch.log의 STATUS 수신 확인. status:0이면 Competition Connect/IP/9099 확인 |
| 연결을 복구하려 함 | 먼저 Enter로 이전 출발 요청 취소; 연결 즉시 출발하지 않게 함 |
| Auto/D handshake timeout | Cmd Control 9093 연결 확인 → 종료·재실행 |
| s 없음 | GPS·IMU·Competition 3개 연결, 경로 근처 위치 확인 |
| NO RECENT CAMERA DATA | 7번 송신/9208, 센서 활성화 확인 |
| 저장 대기만 지속 | 8번/9210 포함 양쪽 연결·동기화, GPS 기반 위치, 정지 여부 확인 |
| 포트 사용 중 | 기존 진단/수집/주행 프로그램 종료 |
| FAULT | launch.log 원인 확인; MORAI Q로 강제 출발하지 말고 정차 후 재실행 |

## 7. 재측정 완료 후 — 날씨별 데이터셋

임시 규정집 1.1.5 기준 Sunny/Foggy × 11/13/15시 (Cloudy 아님).
Foggy Density=1, Distance=0. 위치 측정은 Sunny 13시, 날짜는 샘플 기본값을 유지합니다.
구간별 환경 변화도 최종 주행에서 검증하며, 공식 1.2가 나오면 재확인합니다.
Semantic은 본선 인식 입력으로 사용하지 않습니다. 연습용 정답 생성 허용 범위는 별도 확인해야 합니다.

**현재 바로 가능한 것은 수동 주행 연속 수집**입니다. survey를 종료한 뒤 실행:

```bash
cd ~/2026foscar
./tools/collect_label.sh sunny_13h --no-drive
```

- MORAI의 실제 조건은 직접 설정합니다. 이름만 바꿔도 날씨가 바뀌지 않습니다.
- RGB/Semantic/s를 저장하며, Ctrl+C 한 번으로 종료하면 그 세션의 자동 라벨 초안을 생성합니다.
- 원본: `~/foscar_logs/dataset/날짜_시각_조건/` (rgb/, sem/, meta.csv).
- 결과: `out/날짜_시각_조건/` (dataset/, review/, frames.json, summary.json).
- **기존 자동 수집은 옛 정지선 표 + 고정 대기 후 신호색 무관 출발**이므로 현 상태에서 반복 자동 수집용으로 쓰지 마세요.
- 표를 교정하기 전 생성한 train/val 분할은 확정하지 마세요.
- 신호 상태 한 주기를 실제 관측하고 색별 누락을 확인합니다. 기본 2초 저장은 짧은 상태를 놓칠 수 있어
  필요하면 `src/foscar_camera_train/config/record.yaml`의 interval_s를 조정합니다.

## 8. 자동 라벨 검수와 YOLO 학습

1. review/ 전체 + 정상 판정 표본을 확인합니다. review 표시는 학습 데이터에서 자동 제외됐다는 뜻이 아닙니다.
2. 실제 색, 화살표, 차량등/보행자등, 작은 박스·잘림·누락을 검수합니다. Roboflow는 필수가 아닙니다.
3. 새 표로 분할 기준을 바로잡고, 같은 장면의 연속 프레임이 train/val에 섞이지 않게 합니다.
4. 각 클래스 수, 비어 있는 val, 누수 경고를 해결한 후 학습합니다. check-only의 종료 코드 0만으로 적합 판정하지 않습니다.

학습은 ROS 수집용 Python과 별도 환경을 권장합니다. Python 3.10 가상환경 예시:

```bash
cd ~/2026foscar
python3 -m venv ~/venvs/foscar-yolo
source ~/venvs/foscar-yolo/bin/activate
python -m pip install ultralytics numpy pyyaml opencv-python
export PYTHONPATH="$PWD/src/foscar_camera_train:$PWD/src/foscar_camera:$PWD/src/foscar_control${PYTHONPATH:+:$PYTHONPATH}"
# 실제 검수 완료 결과 경로로 교체하세요.
python -m foscar_camera_train.train --data out/검수완료결과/dataset/data.yaml --check-only
python -m foscar_camera_train.train --data out/검수완료결과/dataset/data.yaml --model yolo26n.pt --batch 4
# 같은 데이터로 비교할 때 --model yolo26s.pt
```

GPU 환경은 새 PC의 드라이버/PyTorch 호환을 확인합니다. 수집과 GPU 학습은 별도로 실행합니다.
학습 코드의 fliplr=0, hsv_h=0을 유지합니다. 메모리가 부족하면 batch부터 줄입니다.
학습 출력 가중치는 Ultralytics 실행 로그의 저장 경로에서 확인합니다.
최종 평가는 학습에 쓰지 않은 별도 주행으로 색별 누락·오인식·거리별 성능·처리 지연을 확인합니다.
수집 카메라 9208 모델을 기존 주행 인식에 연결하는 작업과 실주행 평가는 아직 별도 작업입니다.

## 9. 코드 지도

| 파일 | 역할 |
|---|---|
| tools/survey_points.sh | 통합 위치 기록 실행 |
| src/foscar_camera_train/foscar_camera_train/survey.py | 자동 경로 주행 연결·정차·지점 기록 |
| tools/survey_view.py | RGB 영상 창 (토픽 구독만, UDP 중복 바인딩 없음) |
| tools/위치기록_사용법.md | 현장 조작 설명 |
| tools/collect_label.sh | 연속 수집 후 자동 라벨 생성 |
| src/foscar_camera_train/foscar_camera_train/extract.py | RGB/Semantic → YOLO 라벨·중복 제거·분할 |
| src/foscar_camera_train/foscar_camera_train/train.py | 데이터 점검·YOLO26 학습 |
| src/foscar_camera_train/foscar_camera_train/evaluate.py | 거리·크기별 평가 |

이 문서는 현재 인수인계 기준입니다. 패키지의 개발명세서/개발구조에 남아 있는 이전 카메라 자세,
과거 데이터 존재 여부, 옛 정지선 표 사용 절차는 이 문서의 현재 상태와 구분하세요.
