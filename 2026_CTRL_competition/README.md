# 2026foscar_morai — MORAI K-City 자율주행 베이스 코드

<!-- RECORDS:BEGIN -->
## 최고기록 (MORAI 시뮬, 자동 갱신)

| 구간 | 최고기록 | 달성 시각 | 코드 |
|---|---:|---|---|
| 풀랩 | 3:04.28 | 2026-09-29T08:45:39 |  |
| 신호 1 | 2.45 s | 2026-09-29T08:30:17 |  |
| 정적 장애물(박스) | 19.60 s | 2026-09-29T10:15:16 |  |
| 보행자 | 12.50 s | 2026-09-29T22:05:18 | competition@5c48308+dirty |
| 신호 4 (좌회전) | 3.70 s | 2026-09-29T08:32:47 |  |
| 회전교차로 | 8.64 s | 2026-09-29T10:01:40 |  |
| 고속도로 | 31.29 s | 2026-09-29T09:48:20 |  |
| 터널 | 8.35 s | 2026-09-29T10:59:14 |  |

미션 기록은 그 구간의 시작선부터 끝선까지를 통과했을 때만 인정한다 (중간 출발 시나리오 제외). 신호 대기 시간이 포함될 수 있다.

### 최근 풀랩 (출발~END 주행 시간, 최근 10회)

| 시각 | 랩타임 | 충돌 | 코드 |
|---|---:|---:|---|
| 2026-09-29T22:04:02 | 3:11.40 | 0 | competition@5c48308+dirty |
| 2026-09-29T21:21:04 | 3:09.60 | 0 | competition@a42f919 |
| 2026-09-29T10:26:08 | 3:33.90 | 0 |  |
| 2026-09-29T10:22:30 | 3:32.00 | 0 |  |
| 2026-09-29T10:17:44 | 3:23.90 | 0 |  |
| 2026-09-29T10:14:15 | 3:22.90 | 0 |  |
| 2026-09-29T10:10:45 | 3:24.30 | 0 |  |
| 2026-09-29T10:03:00 | 3:24.10 | 0 |  |
| 2026-09-29T09:59:30 | 3:23.80 | 0 |  |
| 2026-09-29T09:55:59 | 3:24.10 | 0 |  |
<!-- RECORDS:END -->

고속도로 차로 변경은 기본 control 모드로 통합 실행됩니다. [구현·실행 안내](src/foscar_planning/HIGHWAY.md)를 먼저 확인하세요.
**신호등 YOLO·다른 PC에서 이어하기:** [인수인계 및 실행 순서](YOLO_다른PC_인수인계.md)
**개발 상황 공유 Discord:** https://discord.gg/Gy8yP2xFX

```bash
# 1) 센서·인지·판단만, 차는 안 움직임 (지금 내가 한 것)
bash run_competition.sh drive:=false bag:=false

# 2) 1) + BEV 화면 — 상자가 주황 박스로, 시킬 경로가 초록 선으로 보이는지 확인
bash run_competition.sh drive:=false bag:=false rviz:=true

# 3) 실제 주행 + BEV (대회 모드에 rviz만 추가)
bash run_competition.sh rviz:=true

# 4) 대회 그대로 (화면 없음, bag 기록됨)
bash run_competition.sh

# BEV만 따로 (다른 터미널에서 스택이 이미 떠 있을 때)
source install/setup.bash
ros2 run foscar_planning bev_node &
rviz2 -d install/foscar_planning/share/foscar_planning/rviz/bev.rviz

# 직접런치후실행
cd /home/foscar/2026Morai/2026foscar
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
bash run_competition.sh speed_kmh:=30 rviz:=true
```


ROS 2 Humble 기반. 시뮬레이터(MORAI)와 UDP 로만 통신해 GPS+IMU 로 위치를 잡고 전역경로를 곡률 기반 감속으로
10~60 km/h 추종합니다. 미션(신호·장애물·보행자·터널)은 `/behavior/target` 하나로 이 주행을 통제하고,
그 메시지는 **`foscar_planning/behavior_node`(FSM) 만** 냅니다. 인지 노드는 `/events` 로 "봤다"만 말합니다.

**대회 실행은 §3-1 `run_competition.sh`.** 지금 통합된 것: 주행 + 라이다 장애물 회피 + 보행자·NPC 정지(YIELD) + 터널 슬라럼 + 구간 속도
+ **카메라 7번 YOLO 신호 정지(기본 켜짐, §9)**. 베이스 내부 구조는 §8.

**터널 장애물 (2026-09-29, `competition` 머지):** 보이는 장애물 전부를 한 번에 넘는 슬라럼으로 계획한다
(`foscar_planning/tunnel_slalom.py`). 장애물 종류는 가정하지 않고 라이다 군집 범위를 그대로 쓰며(터널 pad 0),
새 곡선은 박스 흔들림 실측값(0.16 m)만큼 여유를 두고 긋는다. 이미 타고 있는 곡선은 박스와 닿지 않는 한 유지하고,
버리기 전에 최신 스캔으로 박스를 다시 확인한다. 늦게 보인 곡선은 그 곡선 속도로 즉시 제동한다.
터널 바닥의 아스팔트(반사율 10~12)와 차선 도색(45 이상, 0.4 m 미만)은 장애물에서 뺀다.
`newtunnelavoid` 시나리오 5회 중 4회 통과(나머지 1회는 옛 FAULT, 지금은 복구 동작).

최신 미션 정책은 [planning 안내](src/foscar_planning/README.md), 회전교차로 동작과 검증 범위는 [회전교차로 개발 상태](src/foscar_planning/ROUNDABOUT.md)를 참조하세요. 고속도로는 CP10–CP13이며 전용 차로 변경 모듈은 기본 활성화됩니다. CP12 통과 후 원래 경로로 복귀합니다.

## 1. 설치 (최초 1회)

```bash
# ROS 2 Humble 설치돼 있어야 함. 빌드 도구 + 파이썬 의존성:
sudo apt install python3-colcon-common-extensions python3-numpy python3-pyproj ros-humble-tf2-ros

git clone https://github.com/26foscar-notebook/2026foscar_morai.git ~/2026Morai/2026foscar
cd ~/2026Morai/2026foscar
python3 -m pip install --target .camera-deps -r src/foscar_camera/requirements-onnx.txt   # 카메라 YOLO (onnxruntime)
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

코드를 고친 뒤에는 `colcon build` (파이썬만 바꿨으면 `colcon build --packages-select foscar_control`) 다시.

## 2. 시뮬레이터 준비

1. 맵 `R_KR_PG_K-City_2025`, 차량 Ioniq5 → START
2. (시나리오 쓸 때) `Edit > Scenario > Load Scenario` → Load → **즉시 `P`** (내장 자율주행 정지)
3. **`F4 > Load`** 로 저장된 센서/네트워크 설정 재로드 — 시나리오 로드가 네트워크 설정을 전부 끊음
4. Network Settings 확인 (목적지 = 이 PC IP):

| 채널 | 포트 | 방향 |
|---|---|---|
| Ctrl Cmd | 9093 | PC → 시뮬 |
| Competition Vehicle Status | 9099 | 시뮬 → PC |
| GPS | 9102 | 시뮬 → PC |
| IMU | 9104 | 시뮬 → PC |

시뮬 데스크톱 IP 는 `src/foscar_bringup/config/low_speed.yaml` 의 `sim_ip` 한 곳에만 있습니다 (지금 `192.168.1.2`,
2026-09-22 부터, 노트북 유선 192.168.1.3). **대회장에서 주는 주소로 이 한 줄만 바꾸고 `colcon build --packages-select foscar_bringup`**
(install 쪽 복사본이 따로라 빌드해야 반영). 수신 쪽은 `0.0.0.0` 바인드라 손댈 것 없음.
바꾼 뒤 확인: 스택을 띄웠을 때 `HANDSHAKE` 가 5 s 안에 `CRUISE` 로 넘어가야 합니다. 안 넘어가면 시뮬 Network 탭의
**Ego Ctrl Cmd 수신 Host IP/Port(9093) 가 새 주소로 Connect 돼 있는지** 확인 (2026-09-19 에 이걸로 한 번 막혔음).

GPS/IMU 송신 주기: 규정 상한 30/50 Hz. 상한에 붙이면 시뮬 FPS 드리프트로 넘칠 수 있고 (Q&A), 코드는 20/40 이면 충분
(`gps_timeout` 0.3 s, `sensor_timeout` 1.5 s 에 여유). 실측 2026-09-19: 설정 30/50 에서 실제 27.7/44 Hz, 최대 간격 66/47 ms.

## 3. 실행

### 3-1. 대회용 통합 실행 (이걸로 돈다)

```bash
cd ~/2026Morai/2026foscar
bash run_competition.sh                    # 주행 + 라이다 인지 + FSM(회피·터널·속도구간). 뜨는 순간 출발
bash run_competition.sh rviz:=true         # + BEV 화면 (foscar_planning/README.md "BEV 로 보기")
bash run_competition.sh drive:=false       # 차는 안 움직이고 인지·판단만 (직접 몰면서 볼 때)
```

| 인자 | 기본 | 뜻 |
|---|---|---|
| `drive` | true | 제어 송신 |
| `states` | `cruise,avoid,yield,tunnel` | FSM 이 돌릴 상태. **신호등(`stop_line`)은 목록에 없으므로 신호 때문에 서지 않는다** |
| `speed_kmh` | 60 | tracker 안전망 상한. 실제 상한은 `foscar_planning/config/behavior.yaml` 의 `cruise.*` (도심 50, 보행자·회전교차로 구간 20, 예외구간 60) |
| `lidar` / `lidar_sim` | true / false | 라이다 브리지+인지 / 가짜 포인트클라우드로 대체 |
| `camera` | true | 카메라 7번 브리지 + YOLO 신호 인식 → 신호 정지 (§9). `false` 면 신호에서 서지 않는다 |
| `rviz` / `view` | false / false | BEV RViz / 라이다 matplotlib 뷰어 |
| `bag` | true | `~/foscar_logs/bags/run_*` 에 판단·제어 토픽 기록 (포인트클라우드 제외) |

무엇이 뜨나: `sensor_bridge` `localization` `path_follower(require_target=true)` `lidar_bridge` `obstacle_node` `behavior_node`
(+ `bev_node` `rviz2` `ros2 bag`). 필수 노드 하나가 죽으면 전체가 내려가고 브리지가 브레이크를 건다.
`require_target=true` 라 `behavior_node` 가 0.5 s 조용하면 차가 선다 — FSM 없이 돌리려면 3-2.

화면에서 볼 것:
```
[behavior_node]  CRUISE → AVOID|CRUISE  (s=287.3 v=8 km/h)     ← 상태 전환 (걸린 상태들, priority 순)
[path_follower]  CRUISE | AVOID | ... s=316.7m error=2.48m        ← error 는 경로 기준 (오프셋 적용 중이면 커진다. FAULT 기준은 오프셋 제외)
```

시뮬 없이 이 런치가 도는지: `python3 src/foscar_planning/test/integration_competition.py` (루프백 가짜 차량, ~40 s).

`foscar_lidar/drive.launch.py` 는 **개발용** (베이스를 따로 include, 카메라 7번 포함). 대회는 위 하나만 쓴다.

### 3-2. 베이스만 (FSM 없이)

```bash
bash run_low_speed.sh drive:=true continuous:=true speed_kmh:=30
```

| 인자 | 기본 | 뜻 |
|---|---|---|
| `drive:=true` | false | 제어 송신. 없으면 센서 수신·상태 표시만 (드라이런) |
| `continuous:=true` | false | 경로 끝까지. 없으면 `distance_m` 만 가고 정지 |
| `distance_m:=20` | 20 | 짧은 확인 주행 거리 (m) |
| `speed_kmh:=30` | 5 | 직선 순항 상한. 코너는 자동 감속. 규정 60, 예외구간만 초과 허용 (60 초과 미검증) |
| `route:=auto` | auto | 차 위치로 경로 자동 선택. `global`(공식 START 부터) / `spawn`(리스폰 위치 연결경로) 강제 가능 |
| `rviz:=true` | false | 경로·차량·목표점 시각화 |

- 종료: **Ctrl+C** → 브리지가 브레이크 인가 후 종료
- 자주 쓰는 조합
  - 한 바퀴: `drive:=true continuous:=true speed_kmh:=30`
  - 20 m 만: `drive:=true`
  - 센서만 확인: (인자 없음)

## 4. 화면 읽는 법

```
[sensor_bridge]  RX={'gps': 180, 'imu': 146, 'status': 146} ...   ← status 가 0 이면 출발 못 함 (§2 3·4번)
[path_follower]  WAITING → HANDSHAKE → CRUISE | tracking | ... v=29.8km/h s=1500.8m error=0.09m
[path_follower]  CRUISE | RECOVER cross track 1.8m: steering back at <= 23 km/h | ...   ← 이탈 → 감속하며 복귀
```

**주행 중 영구 정지(FAULT)는 없습니다 (2026-09-29 부터).** 대회에서 한 번 서면 되살릴 방법이 없으므로,
예전에 FAULT 로 끝나던 상황은 전부 스스로 복구합니다.

| 상황 | 예전 | 지금 |
|---|---|---|
| 경로 이탈 (밴드 밖 1.5 m) | FAULT | `RECOVER cross track`: 순수추종으로 경로로 되돌아가며, 복귀 곡선이 경로 횡가속 한계(`a_lat_mps2`) 안에 들도록 감속 |
| 방향이 경로와 60° 이상 | FAULT | `RECOVER heading`: 방향 오차만큼 크게 감속하며 복귀 (회피 중에도 적용) |
| 구간 목표속도 +5 km/h 초과 | FAULT | 제동만 |
| 위치 튐 (`position jump`) | FAULT | 1 s 동안 직전 명령 유지, 계속 튀면 새 위치를 받아들임 |
| 위치·속도 NaN | FAULT | `HOLD` (핸들 유지·약한 브레이크), 값이 돌아오면 재개 |
| Auto/D 피드백 끊김 | FAULT | 제동 후 Auto/D 재요청(`HANDSHAKE`), 받으면 다시 출발 |
| Auto/D 5 s 무응답 | FAULT | 경고 후 계속 요청 |
| IMU·status yaw 10° 불일치 | FAULT | 경고 후 위치추정 yaw 로 계속 주행 |
| 경로 밖 출발 (`car is N m from the route at launch …`) | FAULT | `WAITING` (출발 안 함). 차를 리스폰/START 로 |

**센서가 끊겨도 FAULT 가 아닙니다 (2026-09-19 부터).** 시뮬이 0.5 s 멈춘 것 하나로 주행이 끝나던 걸 고쳤습니다.

| 상태 | 뜻 |
|---|---|
| `CRUISE \| DR` | GPS 가 0.3 s 이상 없음 → **추측항법** (IMU yaw + status 속도 적분). 터널(s=1881~1998)이 이것. 추측항법 전용 속도 제한은 기본 해제(`dr_speed_kmh=0`). GPS 복귀 시 오차를 2 s 에 걸쳐 없앰 (점프 없음) |
| `HOLD \| sensor timeout …` | odom/status 가 1.5 s 넘게 안 옴 (시뮬 프리징·네트워크). 핸들 유지 + 약한 브레이크(0.15), **돌아오면 CRUISE 로 자동 복귀** |
| `HOLD \| control loop stalled` | 우리 루프가 0.25 s 넘게 밀림 (카메라 기동 등). 그 틱만 건너뜀 |

`[localization] GPS 0.3s 무수신 → 추측항법` / `GPS 복귀: 추측항법 114 m, 오차 0.08 m → 2.0s 에 걸쳐 수렴` 로그로 확인.

**터널 실주행 (2026-09-19, 40 km/h 출발, 터널 앞 s=1820 스폰)**: DR 114 m (s 1883~1997), 복귀 오차 **0.08 m**,
터널 안 횡오차 max 0.12 m, 복귀 수렴 중 0.08 m, FAULT/HOLD 0, 경로 끝까지 FINISH.
- 3회 반복: 출구 오차 0.08 / 0.23 / 0.22 m. 라이다 실선과 대조해 터널 직선부 횡드리프트 없음 (`tools/lidar_lane/README.md`).
- 시뮬의 GPS 음영은 매번 같지 않다: 한 번은 s≈1970 에서 GPS 가 돌아왔다가 1986~1996 에서 10 m 다시 끊겼다. 코드는 몇 번을 오가든 상관없다.
- **IMU yaw 는 진북 기준, map(x, y) 는 UTM52 격자북 기준** → K-City 에서 1.35° 어긋난다 (자오선 수차). `localization` 이
  pyproj 로 자동 보정한다. 보정 전엔 DR 86 m 에 횡오차 2.55 m (20개 로그 전부 -1.40°±0.05 로 일정했던 그 값), 보정 후 0.08 m.
  GPS 모드에서도 이게 "직선 0.2 m 오프셋" 의 원인이었다 — 보정 후 터널 밖 횡오차 max 0.26 → 0.14 m.
## 5. 결과 분석

```bash
# 로그: ~/foscar_logs/follow_<시각>.csv (20 Hz: 상태, 위치, 속도, s, 횡오차, 명령)
python3 ~/2026Morai/측정테스트파일/analyze_follow.py ~/foscar_logs/follow_*.csv     # 주행별 요약 표
python3 ~/2026Morai/측정테스트파일/analyze_follow.py <파일> --zones --width 200     # 200 m 구간별
```

## 6. 미션 시험 (인지 없이)

```bash
source install/setup.bash
# FSM 을 거쳐서 (run_competition.sh 가 떠 있을 때) — 가짜 "봤다" 이벤트
python3 tools/fake_event.py obstacle 329            # s=329 에 2×3 m 상자 → AVOID 가 비켜야 함
python3 tools/fake_event.py obstacle 329 d 0.8      # 왼쪽으로 치우친 상자 → 오른쪽으로
python3 tools/fake_event.py obstacle 329 size 1 7   # 도로를 다 막는 벽 → 앞에 서야 함 (stop_s)
python3 tools/fake_event.py light 315 RED           # stop_line 상태가 있을 때만 의미 있음
# FSM 없이 tracker 만 (run_low_speed.sh 일 때) — 가짜 "이렇게 가라"
python3 tools/fake_target.py stop 208        # 앞바퀴 s=208 에 정지
python3 tools/fake_target.py vmax 5          # 18 km/h 캡
python3 tools/fake_target.py offset 1.0      # 1 m 왼쪽으로
```

## 7. 테스트 (PR 전 필수, 시뮬 불필요)

```bash
export PYTHONPATH=$PWD/.camera-deps:/usr/lib/python3/dist-packages:$PYTHONPATH   # 시스템 numpy 2.2 가 scipy 를 깨뜨린다
python3 -m pytest -q src/foscar_control/test src/foscar_planning/test src/foscar_lidar/test src/foscar_camera; echo rc=$?   # 파이프로 거르지 말 것
source install/setup.bash && python3 src/foscar_planning/test/integration_competition.py   # 대회 런치 통째로
```

## 8. 구조

```
src/foscar_interfaces/msg/   DrivingTarget, Event, VehicleStatus, ControlCommand   ← 변경은 팀 3 승인
src/foscar_control/foscar_control/
    bridge.py        UDP ↔ ROS (조향 rad→-1~1 변환은 여기 한 곳)
    localization.py  GPS+IMU+속도 → /localization/odometry (GPS 없으면 추측항법), /localization/s, /localization/mode
                     IMU yaw(진북) 를 UTM 격자북으로 보정 (K-City -1.35°, pyproj 자동)
    core.py          ROS 없는 계산: 패킷, Route(곡률·속도 프로파일), PoseEstimator(GPS↔추측항법), Target, Controller
    follower.py      path_follower 노드: 센서 → Controller.step() → /control/request. /behavior/target 구독
src/foscar_bringup/   launch/competition.launch.py (대회), low_speed.launch.py (베이스), config/low_speed.yaml, routes/, rviz
src/foscar_planning/  behavior_node (FSM 코어) + states/{cruise,avoid,tunnel}.py + avoid_planner + bev_node. 자세한 건 그 README
src/foscar_lidar/     lidar_bridge, obstacle_node (→ /events OBSTACLE/PEDESTRIAN/VEHICLE), 뷰어. 팀 2
src/foscar_camera/    camera_bridge (카메라 7번) + yolo_signal_node (→ /perception/traffic_signal) + camera_view. §9
tools/lidar_lane/    라이다 반사도 차선 측정 도구 (GUI·분석). 결과와 결론은 그 폴더 README
tools/fake_event.py  인지 대역 (FSM 에 가짜 이벤트)
tools/fake_target.py FSM 대역 (tracker 에 가짜 목표)
```

토픽·좌표·단위 규약과 팀 분업은 `개발계획서.md`(레포 밖, `~/2026Morai/`) §1~2.

## 9. 카메라 신호 정지 (카메라 7번 YOLO)

카메라는 **7번 하나**(포트 9208, 1280×720)만 쓴다. `run_competition.sh` 기본값이 `camera:=true` 다.

```
camera_bridge (UDP 9208) → /sensors/camera/traffic/compressed
  → yolo_signal_node (models/best.onnx, 정지선 촬영 구간에서만 추론) → /perception/traffic_signal
  → behavior_node (traffic.enabled) : config/yolo_stoplines.json 의 정지 위치로 정지 목표를 다른 목표와 합침
```

- 빨강·노랑이면 정지선 앞에 서고, 초록(좌회전 교차로는 좌회전 신호)이면 통과한다. 상세: `src/foscar_camera/YOLO_STOPLINE_2026-09-28.md`.
- onnxruntime 은 `.camera-deps` 에 설치한다 (`python3 -m pip install --target .camera-deps -r src/foscar_camera/requirements-onnx.txt`).
  `run_competition.sh` 가 이 폴더를 PYTHONPATH 앞에 둔다.
- 카메라 없이 달리려면 `camera:=false` (신호에서 서지 않는다).
- 확인 (2026-09-29): 풀랩 PASS 189.6 s, YOLO 106 프레임 판정 (red 67 / green 13 / yellow 7).
