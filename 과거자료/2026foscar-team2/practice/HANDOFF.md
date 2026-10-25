# 시뮬 반복 시험·개선 인계 문서 (practice/ 전용)

마지막 갱신: 2026-09-29. 이 문서만 읽고 바로 이어서 시험·개선할 수 있게 쓴다.
`practice/` 폴더는 **제출 전 통째로 삭제**한다 (§1.4). 이 문서도 같이 지워진다.

---

## 0. 한눈에

- **새 세션은 이 문서를 처음부터 끝까지 읽고 시작한다.** 그다음 `practice/TuningPoint.md` 맨 위에서 사용자의 새 요청을 확인한다(§7).
- 개발 루프: **코드 수정 → 단위 테스트(rc 확인) → 시뮬에서 `loop_runner.py` 반복 주행 → bag 분석 → 브랜치 커밋 → 사용자 동의 후 main 머지**.
- **작업 브랜치: `competition`** (worktree `~/Desktop/2026Morai/2026foscar-competition`). 2026-09-29 main 에 머지됨. main = 검증된 완성본.
- 패키지 이름은 `foscar_*` (2026-09-29 변경), 로그 폴더는 `~/foscar_logs`.
- 시뮬은 별도 PC, 노트북에서 SSH·UDP 로 원격 조작한다(§2).
- 기록: 최고기록과 최근 풀랩은 GitHub main README 표에 자동으로 올라간다(§3.2). 풀랩을 돌리면 랩타임은 항상 거기에 남는다.
- 사용자와는 **한국어**로 대화한다. 결과 보고 방식과 선호는 §7.

## 1. 대회 규정: 하면 안 되는 것

근거 우선순위: 규정집 v1.1 (2026-08-28) > 중간점검 가이드 > 공식 Q&A > 배포 데이터.
상세 원문 요약은 `../Readme, 각종정보파일/PROJECT_MASTER.md` 에 있다 (저장소 밖, 한 단계 위 폴더).

### 1.1 주행 코드가 쓸 수 있는 채널 (이것만)

| 채널 | 포트 (우리 설정) | 비고 |
|---|---|---|
| Ego Ctrl Cmd (PC → 시뮬) | 9093 | **cmd type 1 (accel/brake), ctrl mode 2 만** 허용 |
| Competition Vehicle Status | 9099 | 위치 없음 (vel_x, 자세, 제어 입력만) |
| GPS / IMU | 9102 / 9104 | 송신 주기 상한 30 / 50 Hz |
| LiDAR / Camera | 9302 / 92xx | |

- CollisionData 는 규정집상 허용 채널로 적혀 있다. 하지만 팀 방침상 **주행 코드에서는 쓰지 않는다**.
  연습 러너만 쓴다. `check_submission.sh` 도 9091/9092 를 연습 전용으로 잡는다.
- 시뮬과는 **순수 UDP 만** 쓴다. MORAI ROS 연결(rosbridge)은 금지다.
- Ubuntu 에서는 **포트 4자리**를 쓴다. 3자리 포트는 동작하지 않아서 일부러 바꿔 둔 것이다.

### 1.2 금지 채널 (수신·사용 시 실격). **practice/ 도구만** 쓸 수 있다

| 채널 | 연습 설정 포트 | 연습에서 쓰는 곳 |
|---|---|---|
| Object Info (주변 객체 정답) | 시뮬 7605 → 노트북 7505 | 러너가 충돌 상대 확인, 장애물 위치 기록 |
| Ego Vehicle Status 전체판 (위치·타이어) | 시뮬 **9106** → 노트북 9111 | 정답 위치·속도로 추측항법 오차, 횡가속도 기록 |
| Scenario Load | 시뮬 9095 (응답 9096) | 시나리오 재시작 |
| Collision (연습 방침) | 9091 / 9092 | 충돌 판정 |
| TrafficLight Status, MultiEgoSetting, Ghost 등 | – | 쓰지 않는다 |

- **절대로 이 데이터를 인지·판단·제어에 넣지 않는다.** 러너가 판정용으로만 읽는다.
- Ego Vehicle Status 의 시뮬 송신 포트는 **9106** 이다. 9101 은 GPS 가 이미 쓰고 있어서 충돌해 안 나왔다.

### 1.3 채점·페널티 (판단 기준)

- **속도 제한**: 예외구간 **s 1118.7 ~ 1741.7 = CP10 ~ CP13** (링크 A2256W000411 시작 ~ A2256W000153 끝, 지도로 확인) 밖에서는 **60 km/h**.
  - `PROJECT_MASTER.md` 의 "1119~1878" 은 틀렸다. A2256W000153 의 끝점이 정확히 CP13 이다 (2026-09-27 확인). **CP13 을 지날 때 이미 60 이하**여야 한다.
  - 넘으면 즉시 15 s, 3 s 지속마다 +15 s. 주최측은 Ego Vehicle Status 로 판정한다.
  - 가속·곡선 속도를 올릴 때마다 `analysis/speed_limit_check.py` 로 확인한다.
  - 205.9 s 랩은 CP13 을 62 km/h 로 지났다 → 제어기가 예외구간 끝 15 m 전부터 57 km/h 를 목표로 하도록 고쳤다.
- 페널티 크기 순서: **차선 접촉(3 s 당 5 s) < 충돌(15 s) < 경로 이탈(15 s + 재주행 ≈ 24 s)**.
- 신호 위반 각 15 s (앞바퀴가 정지선을 넘는 순간 판정). 신호는 카메라로만 인식한다.
- 출발: 60 s 안에 5% (109 m) 를 통과해야 한다. P→D 전환과 AutoMode=2 가 필수다.
- 체크포인트: 반경 3 m 안으로 통과해야 한다.
- 제한시간 15 분. 완주팀이 항상 미완주팀보다 우선한다. 2회 중 짧은 기록을 쓴다.
- GPS 음영(터널 s 1881~1998) 은 차로 준수 채점이 면제된다. 음영구역 안 임의 위치에 장애물 또는 끼어들기가 나온다.
- NPC 차량은 회전교차로·고속도로에만, 보행자는 보행자 미션에만 나온다 (사용자 확인).
- **대회 당일 코드 수정 불가** (파라미터·주석 포함). IP/포트는 설정 파일로 분리돼 있다.
- 코드 유사성 검토가 있다.

### 1.4 제출 전 (필수)

```bash
bash practice/check_submission.sh   # practice/ 밖에 연습 채널 흔적이 없는지 검사 (exit 0 이어야 함)
git rm -r practice CLAUDE.md && git commit -m "chore: remove practice tools and agent notes for submission"
```

- 주행 코드(`src/`, `run_*.sh`, `status.sh`)는 `practice/` 를 import·실행·참조하지 않는다.
- 저장소 최상위 `CLAUDE.md` 는 에이전트 안내용이라 제출 때 같이 지운다.
- `practice/COLCON_IGNORE` 때문에 빌드에서도 빠진다.
- 새 도구는 반드시 `practice/` 안에 만들고, 만든 뒤 `check_submission.sh` 를 돌린다.

---

## 2. 환경

### 2.1 시뮬 PC (2026-09-27 리눅스 PC 로 교체)

- 시뮬 리눅스 PC `foscar-All-Series` (RTX 2070 SUPER).
  - **MORAI UDP 는 `192.168.0.11`** (시뮬 송신 hostIP). 주행 코드 `src/foscar_bringup/config/low_speed.yaml` 의 `sim_ip`, 연습 러너 `practice/sim/loop_runner.py` 의 `SIM_IP`.
  - **SSH 는 `ssh morai`** (`~/.ssh/config`: 192.168.0.11 포트 22, 계정 `foscar`, 키 `~/.ssh/morai_sim`). 예전 주소 192.168.0.8:9992 는 2026-09-29 기준 응답 없음.
  - 시뮬은 센서·제어를 전부 **`192.168.0.10`** 으로 보낸다(시뮬 쪽 Sensor/Network 설정). 2026-09-29 현재 노트북은 **유선 `enp4s0` 에 192.168.0.10** 이 잡혀 있다
    (와이파이만 쓸 때는 `nmcli con mod <연결> +ipv4.addresses 192.168.0.10/24 && nmcli dev reapply <장치>` 로 추가 주소를 붙인다).
  - 시뮬 PC 방화벽(ufw)은 꺼져 있다. 켜면 `sudo ufw allow from 192.168.0.10 to any port 9992` 와 UDP 포트를 연다.
- 실행 중인 런처 (2026-09-29 확인): `~/바탕화면/MoraiLauncher/MoraiLauncher_Lin/` (시뮬 `Simulator_v.S4.251001.MolitComp03_Linux`). 홈에 다른 설치본이 여럿 있으니(`~/MoraiLauncher_Lin`, `~/MoraiLauncher_FOSCAR` 등) 헷갈리지 말 것.
- 저장 폴더 `.../MoraiLauncher_Lin_Data/SaveFile/`: 센서셋 `Sensor/25.S4.MolitComp03/`, 네트워크 `Network/25.S4.MolitComp03/NetworkInfo_2023_Hyundai_Ioniq5.json`.
- 시나리오 폴더: `~/바탕화면/MoraiLauncher/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/`
  - `Practice3_START`, `Practice3_CP01~15`, 원본 `CompetitionPractice3` 을 2026-09-27 에 복사해 두었다.
  - 시나리오별 네트워크 설정: 같은 폴더 `EgoNetwork/<시나리오이름>_MN.json`.
  - 템플릿은 `scenarios/EgoNetwork_template_MN.json` 이다(시뮬 0.11 → 노트북 0.10). 새 시나리오를 만들면 이걸 `<이름>_MN.json` 으로 복사한다.
  - 복사: `scp <파일> "morai:바탕화면/MoraiLauncher/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/"`
- 이전 환경(윈도 시뮬 `192.168.1.2`, 계정 `pdyin`, 유선 직결 `192.168.1.3`)은 2026-09-27 에 끝났다.

### 2.2 시뮬 상태 확인

- **시뮬은 주행 맵 화면에 있어야 한다.** 런처나 메뉴 화면이면 시나리오 로드도 안 먹고 센서도 안 나온다.
  - 이때 러너 결과는 `NO_START:WAITING` 이고, 로그에 `RX={'gps': 0, 'imu': 0, 'status': 0}` 이 찍힌다.
  - 확인법: 노트북에서 GPS 포트를 2초 들어 본다. 주행 화면이면 약 60~80 개가 들어온다. 0 이면 사람에게 맵 진입을 부탁한다.
    `python3 -c "import socket,time;s=socket.socket(2,2);s.bind(('0.0.0.0',9102));s.settimeout(.5);n=0;t=time.time()` 식으로 2초간 recv 개수를 센다.
  - 프로세스 메모리(`tasklist`)로는 판단할 수 없다. 주행 화면에서도 약 200 MB 로 보였다.
- 시나리오를 로드하면 시뮬이 네트워크 연결을 끊는다. 러너는 `--network` 로 시나리오의 `_MN.json` 을 같이 로드해 복구한다.
  - 사람이 수동으로 할 때는 로드 직후 `P` (내장 자율주행 정지) 를 누르고, `F4 > Load` 로 네트워크를 다시 불러온다.
- **센서셋을 새로 만들거나 바꾼 뒤**(2026-09-29): 러너가 시나리오+네트워크를 불러와도 **Competition Vehicle Status(9099) 만 0건**이라 `NO_START:WAITING` 이 났다
  (`RX={'gps': 1600, 'imu': 1187, 'status': 0}`). 설정 파일은 맞았고, 시뮬 화면 Network Settings 에서 그 채널을 Connect 하자 바로 됐다. 이 로그가 나오면 사람에게 그 채널 연결을 부탁한다.
- **센서셋 작업 중엔 시뮬이 멈춘다.** 그 동안 도는 회차는 버린다: 정답 위치는 멈췄는데 상태 패킷이 같은 속도를 계속 보내 추측항법이 차를 159 m 밀어냈다.

### 2.3 빌드·환경

```bash
cd ~/Desktop/2026Morai/2026foscar-competition
python3 -m pip install --target .camera-deps -r src/foscar_camera/requirements-onnx.txt   # 최초 1회 (YOLO onnxruntime, numpy 1.24)
colcon build --symlink-install     # 반드시 symlink. 안 그러면 테스트가 src 가 아닌 install 복사본을 import 한다
source /opt/ros/humble/setup.bash && source install/setup.bash
export PYTHONPATH=$PWD/.camera-deps:/usr/lib/python3/dist-packages:$PYTHONPATH   # 러너·테스트 전에 (아래)
```

- **`~/.local` 의 numpy 2.2 가 먼저 잡히면 scipy·sklearn 이 import 에서 죽는다** → behavior_node·obstacle_node 가 뜨자마자 죽고 러너는 `STACK_EXITED`.
  러너·테스트 전에 위 `export` 를 하고, ROS setup 을 다시 source 하지 않는다. `run_competition.sh`·`run_camera.sh` 는 `.camera-deps` 를 스스로 앞에 둔다.

- config yaml 도 symlink 라서 파라미터 수정은 재빌드 없이 다음 실행부터 반영된다.
- 새 파일을 추가했을 때만 다시 빌드한다.

### 2.4 PC 이전 체크리스트 (2026-09-27: 시뮬 PC → 리눅스, 실행 노트북도 교체 예정)

**새 실행 노트북**
1. 저장소 받기: `git clone` 뒤 작업 브랜치 `exp/corner-speed` 체크아웃 (main 은 v1.0 기준 검증본).
2. ROS 2 Humble 과 의존성 설치 (README §1), `colcon build --symlink-install`.
3. SSH 키: 예전 노트북의 `~/.ssh/morai_sim`, `morai_sim.pub` 를 옮기거나, 새로 만들어 시뮬 PC `~/.ssh/authorized_keys` 에 등록한다.
4. 주행 코드의 시뮬 IP: `src/foscar_bringup/config/low_speed.yaml` 의 `sim_ip`.
5. 연습 도구의 시뮬 IP: `practice/sim/loop_runner.py` 의 `SIM_IP`.
6. 지도 데이터는 저장소에 들어 있다(`src/foscar_bringup/maps/link_set.json`, 경로는 수정본 `routes/global_path.txt`). 따로 옮길 필요 없다.
   주행 코드는 `package://foscar_bringup/maps/link_set.json` 로 찾고, 테스트·도구는 저장소 기준 상대경로로 찾는다. 저장소 폴더 위치는 상관없다.

**새 시뮬 PC (리눅스)**
1. SSH: `sudo apt install openssh-server` 후 노트북 공개키를 `~/.ssh/authorized_keys` 에 추가한다.
   - ufw 를 쓰면 `sudo ufw allow from <노트북 IP> to any port 22`.
2. 시나리오: `practice/scenarios/Practice3_START.json`, `Practice3_CP01~15.json` 을
   `<MORAI 설치>/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/` 로 복사한다.
3. 시나리오별 네트워크 파일: `scenarios/EgoNetwork_template_MN.json` 을 같은 폴더 `EgoNetwork/<이름>_MN.json` 으로 복사한다.
   - **템플릿에 IP 가 박혀 있다**: `destinationIP` 192.168.1.3 (노트북), `hostIP` 192.168.1.2 (시뮬). 새 IP 로 바꾼 뒤 복사한다.
4. 시뮬 네트워크 설정: 연습용 EgoVehicleStatus 송신 포트는 **9106** (9101 은 GPS 와 겹침).
5. 시뮬 상태 확인(§2.2)은 GPS 패킷 수로 한다.

---

## 3. 도구 사용법

### 3.1 단위 테스트 (커밋 전 필수, 종료코드로 판정)

```bash
PYTHONNOUSERSITE=1 python3 -m pytest -q src/foscar_control/test src/foscar_planning/test > /tmp/pt.log 2>&1; rc=$?
grep -E "FAILED|passed|failed" /tmp/pt.log | tail; echo rc=$rc
python3 -m pytest -q src/foscar_lidar/test src/foscar_camera src/foscar_camera_train; echo rc=$?   # lidar 는 user-site sklearn 필요 (NOUSERSITE 없이)
```

- **`| tail` 로 파이프하면 실패가 숨는다.** 실제로 실패 상태로 커밋한 적이 있다. 반드시 `rc` 를 확인한다.
- 2026-09-29: control 79 · lidar 66 · camera 138 통과. planning 503 통과 **5 실패**
  (`test_empty_entry` 1, `test_entry_gate` 3, `test_following_departure` 1 — 회전교차로·출발, 이 날 작업 전부터 실패하던 것).

### 3.2 반복 주행 러너 `sim/loop_runner.py`

한 번 실행할 때마다 다음을 순서대로 한다. 시나리오 로드(+네트워크) → `run_competition.sh` 로 실제 주행 → ROS 토픽·충돌·ObjectInfo·정답 상태 감시 → 판정 → 스택 종료.

```bash
# 전체 랩 (출발점부터). 카메라 기본 켜짐이라 신호 대기가 있다 → stop-s 40
python3 practice/sim/loop_runner.py Practice3_START --runs 1 --end-s 2165 --watch-from 0 --stop-s 40 --timeout 500
# 터널만 (newtunnelavoid 시나리오)
python3 practice/sim/loop_runner.py newtunnelavoid_R1790 --runs 1 --end-s 2010 --watch-from 1790 --stop-s 8 --timeout 120
# 고속도로만 (CP08 = 회전교차로 직전부터 시작)
python3 practice/sim/loop_runner.py Practice3_CP08 --runs 3 --end-s 1745 --watch-from 1110 --stop-s 25 --timeout 300
```

- 시나리오: `Practice3_START`, `Practice3_CP01` ~ `CP15`. 각 체크포인트에 자차를 둔다. 생성기는 `scenarios/make_checkpoint_scenarios.py`.
- 결과 파일: `practice/sim/runs/<stamp>/results.jsonl` (한 줄 = 한 회). 런치 로그 `launch_<n>.log`, bag 은 `~/foscar_logs/bags/run_<stamp>/`.
- `result` 값:

| 값 | 의미 |
|---|---|
| `PASS` | `end-s` 도달 |
| `COLLISION:id<N>` | 충돌 |
| `COLLISION?:speed_drop` | 브레이크 없이 급감속 |
| `STOPPED:<state>` | `stop-s` 이상 정지 |
| `FAULT:<사유>` | (2026-09-29 부터 주행 중에는 안 나온다: 컨트롤러가 영구 정지 대신 `RECOVER`/`HOLD`/재핸드셰이크로 복구. 로그에서 `RECOVER` 를 찾는다) |
| `NO_START:<state>` | 출발 못 함 |
| `TIMEOUT` | 시간 초과 |

- 그 밖의 필드:
  - `splits`: 구간 경계 도달 시간.
  - `full_stops`: 출발 후 완전 정지 위치 (보행자 양보 s≈451 은 정상).
  - `truth_peaks`, `truth_peaks_at`: 최대 횡가속도 등과 그 위치(s)·속도.
  - `ay_top`: 10 m 단위 횡가속도 상위 10 곳 `[s, |ay|, km/h]`.
  - `far_collision_packets`: 멀어서 충돌로 치지 않은 패킷.
  - `pose_error`: GPS·추측항법 위치 오차.
- 충돌 판정 규칙 (오판 수정 이력):
  - 출발 전 패킷은 무시한다 (이전 회차 충돌이 남아 들어온다).
  - 상대가 **자차 중심 8 m 안**일 때만 충돌로 본다. 240 m 떨어진 차의 패킷이 실제로 들어온 적이 있다.
  - 속도 판정은 3 샘플 중앙값을 쓰고, 정답 속도로 교차 확인한다.
  - 정답 속도는 **km/h 단위**다. 러너 안에서 /3.6 해서 m/s 로 쓴다.
- RViz 는 기본으로 안 띄운다 (`--rviz` 로 켬). 사람이 보고 싶으면 `status.sh` 로 따로 띄운다.
- **기록 (매 회 자동)**: `race_hud_node` 가 `~/foscar_logs/records.json` 에 최고기록을 쓰고, 러너가 회차 뒤 `records_readme.py` 로 GitHub main README 표를 갱신한다
  (worktree `../2026foscar-records`, README 한 파일만 main 에 직접 푸시하는 유일한 예외).
  - 결과가 PASS 가 아닌 회차에서 생긴 기록은 러너가 되돌린다 (멈춘 시뮬에서 박스 7.2 s·보행자 12.35 s 가짜 기록이 올라간 적이 있다).
  - 출발점부터 END 까지 통과한 풀랩은 최고기록이 아니어도 `~/foscar_logs/laps.json` 에 쌓이고, README "최근 풀랩" 표(최근 10회)에 올라간다.
- 러너를 백그라운드로 돌리고 끝날 때까지 기다릴 때, `pgrep -f "loop_runner.py ..."` 로 폴링하지 않는다.
  - 대기 셸 자신의 명령줄에도 그 문자열이 있어서 영원히 안 끝난다.
  - 러너 명령을 직접 백그라운드 작업으로 실행하고 종료 알림을 받는다.

### 3.3 bag 분석 `sim/analysis/`

| 스크립트 | 용도 |
|---|---|
| `speed_why.py <bag> <dt>` | s별 속도·목표속도·v_max·상태·제한 원인(limit_source)·페달·고속도로 상한 이유 |
| `highway_follow.py <bag> <N> <step>` | 앞차 간격·앞차 속도·접근속도·comfort/hard cap |
| `highway_admission.py <bag> <t0> <t1>` | 차선변경 허가 순간의 blocked_by·후방 간격·admission_profile |
| `speed_limit_check.py <bag>` | 예외구간 밖 60 km/h 초과 여부 |
| `read_bag.py <bag> <topics> <t0> <t1>` | 임의 토픽 덤프 |

- 주요 토픽:
  - `/behavior/highway`: JSON. state, reason, speed_cap_mps, speed_reason, blocked_by, following_objects, gap_strategy, admission_profile 등.
  - `/behavior/target`: v_max, state.
  - `/control/telemetry`: speed_mps, target_speed_mps, limit_source, accel, brake.
  - `/localization/s`, `/vehicle/status`.
- 같은 bag 에서도 s 가 1000 미만이면 `s= 580.9` 처럼 공백이 들어간다. awk 로 거를 때는 `sed 's/s= */s=/'` 를 먼저 한다.

### 3.4 체크포인트 ↔ 경로 거리 (전역 경로 s)

| CP | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 끝 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| s | 73 | 186 | 237 | 347 | 471 | 581 | 635 | 859 | 931 | 1119 | ~~1286~~ 삭제 | 1594 | 1742 | 1860 | 1997 | 2185 |

- **CP11 은 2026-09-30 규정에서 빠졌다.** `checkpoints.txt` 에서 [11] 을 지웠고 나머지 번호는 그대로다. 고속도로는 5→4 뒤 4차로에서 기다리지 않고 바로 4→3 을 판단한다 (`highway.yaml` `hold_checkpoint: null`).

- 미션 구간 (`src/foscar_planning/config/missions.yaml`):

| 미션 | 구간 |
|---|---|
| 정적 장애물 | 275~390 |
| 보행자 | 405~495 (20 km/h) |
| 회전교차로 | 860~920 (20 km/h) |
| 고속도로 | CP10~CP13 (고정 상한 없음, 곡률만) |
| 터널 | 1875~2000 |

---

## 4. Git 워크플로

- **main = 검증된 완성본** (v1.0 태그 = 첫 완성본). 작업은 브랜치에서 하고, 성공하면 **사용자 동의를 받아** main 에 머지한다. main 에 직접 커밋하지 않는다.
- 지금 작업 브랜치는 `competition`. 다른 worktree(`-camera`, `-drive`, `-camdrive`, `-dataset-publish`)는 이름 변경(`foscar_*`) 전 브랜치라 합칠 때 정리가 필요하다.
- main 은 다른 에이전트가 쓰는 `~/Desktop/2026Morai/2026foscar` 에 체크아웃돼 있지 않다. 머지는 임시 worktree(`git worktree add --detach <tmp> origin/main`)에서 하고 `git push origin HEAD:main`.
- **커밋하지 않는 것**:
  - 카메라(`src/foscar_camera*`)는 사용자가 지시한 변경만 커밋한다 (2026-09-29: 이름 변경, 카메라 7번 외 삭제).
  - `src/foscar_bringup/routes/backup/` (경로 편집기 백업).
- `git add` 는 항상 파일을 지정해서 한다 (`-A` 금지). 사용자가 작업 폴더에서 직접 고친 파일(예: 경로)은 확인받고 커밋한다.
- 커밋 메시지에는 원인(측정값·위치·날짜)을 남기고, 끝에 `Co-Authored-By: ...` 줄을 붙인다.

---

## 5. 현재 상태

### 5.0 2026-09-29 (`competition`, main 머지)

- **터널 슬라럼** (`foscar_planning/tunnel_slalom.py`, `states/avoid.py`): 장애물 종류를 가정하지 않고 라이다 군집 범위를 그대로 쓴다(터널 pad 0).
  새 곡선은 박스 흔들림 실측(0.16 m, p95)만큼 여유, 타고 있는 곡선은 닿지 않는 한 유지(지나온 그룹 건너뜀), 버리기 전에 최신 스캔으로 재확인,
  늦게 들어온 곡선은 그 곡선 속도로 즉시 제동. `newtunnelavoid` 5회 중 4회 통과(나머지 1회는 옛 FAULT).
- **라이다**: 터널 바닥의 아스팔트(반사율 10~12)와 차선 도색(≥45, 0.4 m 미만)을 뺀다.
- **컨트롤러**: 주행 중 영구 정지(FAULT) 없음. 경로 이탈·방향 이탈은 `RECOVER`(감속하며 복귀), 과속은 제동, 위치 튐은 1 s 유지 후 수용,
  NaN 은 `HOLD`, Auto/D 끊김은 재핸드셰이크. README §4 표.
- **이름**: 패키지 `foscar_*`, 로그 `~/foscar_logs`.
- **카메라**: 7번(9208) YOLO 만 남기고 나머지 카메라·HSV 노드 삭제. 대회 런치 기본 `camera:=true`.
- **풀랩**: PASS 189.6 s (카메라 켬, 충돌 0). 정지: s311.5, 451.9(보행자), 885.7(회전교차로).
- 남은 것: 회피 플래너가 막힘으로 서면 앞으로 빠질 곡선이 없어 사실상 끝난다(후진 또는 모서리 대비 필요). 센서 값이 얼어도 상태 패킷만 오면 멈춤을 모른다.

### (이전) 2026-09-27 기준, 브랜치 `exp/corner-speed`

### 5.1 main 에 들어간 것 (51b70eb, 문서 정리 a5c05c6)

- 터널 벽 인식·회피 안정화, 고속도로 앞차 간격(급제동 4 m/s² 기준), 차선변경 뒤차 조건(`REAR_CLOSING`), 가속 개선(스로틀 0.8), 고속도로 미리보기 거리.
- 문서 정리: 주제별 문서 하나씩만 남김(README, `src/foscar_planning/{README,HIGHWAY,ROUNDABOUT}.md`, lidar README, practice 문서).

### 5.2 `exp/corner-speed` 에만 있는 것 (주요 커밋, 오래된 순)

**곡선·속도**
- `3948162`: 곡률 계산 창 ±1.5 → ±4 m. 일반 도로 코너 진입 감속 1.5 → 2.5 m/s².
- `f93ff58`: 고속도로 구간은 곡률 창 ±10 m. 예외구간 끝(CP13) 15 m 전부터 57 km/h 목표.
- `dc8b8e6`: 구간별 횡가속도 한도 `a_lat_zones` 추가.
- `871c3cd`: **전역 횡가속도 한도 2.5 → 4.5 m/s²**. 스윕 3.0 / 3.5 / 4.0 / 4.5 에서 회피 구간 밖 최대 횡오차 0.51 m 이하였다.
- `ca0ba67`: lookahead 최대 14 m. 10 m 는 고속에서 흔들렸고, 20 m 는 차로 사이를 흘렀다.
- `bcac1bc`: END 를 속도 그대로 통과한다. 경로 끝에 60 m 직선을 붙이고, 경로 끝 정지만 4 m/s² 로 한다.
- `a6b1275`: 사용자가 `tools/path_editor.py` 로 회전교차로 경로(s876~960)를 폈다. 최소 반경 11.0 → 12.4 m.

**고속도로 차선변경**
- `ac70b34`: 차선변경이 속도를 고정하지 않는다. 경로는 도달할 속도 기준으로 만든다.
- `b4650a5`: 그 설계가 맞지 않으면 현재 속도로 다시 설계한다.
- `1ae13e3`, `b92571f`: CP11 통과 기록을 위치 초기화에도 유지한다. 변경 경로를 감속 거리만큼 연장하고, 정렬 대기(SETTLE)는 경로를 지나치면 완료로 처리한다.
- `b676dce`, `ca0ba67`: 고속도로 미션이 끝나면 로컬 경로를 반납한다(`reference_release`). 노드 초기화 순서도 반영했다.
- `92c8dc8`, `8b4dcbb`: **시작 순간 나란함 금지(`ABREAST`)**. 목표 차로 차가 확실히 앞이거나 뒤여야 시작한다.
- `c328068`:
  - **진입 차로 먼저 앞지르기(`PASS_FIRST`)**
  - 마감을 차폭 기준으로 바꿈(5차로 s1228 → s1247)
  - 변경 시간 2.5 s
  - 앞차 간격 `follow_headway_s` 0.5, `headway_s` 1.0
- `b4650a5`: 변경 중 떠나는 차로의 앞차는 따라잡을 때만 본다. `cut_in_margin_m` 1.0.
- `10402bc`: 진입 차로에서 목표 차로 차의 속도가 미확정이면 확인될 때까지 변경을 보류한다.
- `38cc710`: CP11 합류(테이퍼)에서는 `ABREAST` 를 빼서 교착을 막는다. **`prepare_gap` 계산 0.7 → 0.2 s** (판단 노드가 멈춰 급제동하던 문제).

**인식·기타**
- `871c3cd`: 고속도로 급제동 방지.
  - 같은 ID 는 마지막으로 잰 속도를 1 s 유지한다.
  - 3스캔 미만의 미확정 물체는 비상 거리 밖이면 무시한다.
  - 스캔 공백 0.6 s 까지는 속도를 유지한다.
- `1ab2bfe`, `40165ae`, `4f9dcb9`, `a2895bf`: 회전교차로.
  - 20 km/h 상한 두 개를 없앴다(미션, 진입 판단).
  - 예측 모델 속도 25 km/h.
  - 예측이 늦으면(1 s 이내) 속도를 유지한다.
  - 관측·미션 시작을 s830 으로 앞당겼다.
- `1ae13e3`: 회피 중 가속 금지. 터널 두 번째 장애물 충돌 대응.
- `35dff1f`: 정적 장애물 구간에서 `DYNAMIC_UNKNOWN` 도 상자로 회피한다. **되돌릴 후보(§5.4)**.

**최근 검증**
- 175.3 s / 182.3 s / 179.8 s / 175.7 s: 모두 PASS.
- 184.0 s PASS, 그리고 **1회 충돌(s1244)**. 원인을 `38cc710` 으로 고쳤지만 수정 후 실주행 검증은 아직이다.

### 5.3 다음 할 일

1. **`38cc710` 이후 전체 랩 검증** (새 PC 로 옮긴 뒤). CP10~11 에서 확인할 것: 앞지르기(`PASS_FIRST`), 속도 확인 보류(`TARGET_MOTION_PENDING`), 합류.
2. `TuningPoint.md` 항목의 상태를 🔄 에서 ✅ 로 갱신한다. 실주행 결과를 근거로 적는다.
3. 판단 노드 부하: `prepare_gap` 이 아직 약 0.2 s 걸린다. 고속도로에서 판단 주기가 밀리는지 bag 으로 확인한다(`/behavior/target` 간격).
4. 앞차 간격 0.5 s 로 줄인 뒤 추돌이 나면 0.6 s 로 되돌린다.
5. 검증되면 사용자 동의를 받아 main 머지.

### 5.4 되돌릴 후보 (사용자 지시)

- **35dff1f** (정적 장애물 구간에서 `DYNAMIC_UNKNOWN` 도 상자로 회피): 시뮬 지연(라이다 8 Hz) 탓일 가능성이 있는 추돌에 대응한 수정이다. `DYNAMIC_UNKNOWN` 관련 문제가 생기면 이 커밋을 되돌린다.

---

## 6. 과거 함정 (같은 실수 반복 금지)

- (2026-09-27 추가)
  - **위치 연속성 초기화가 CP11 통과 기록을 지웠다** (GPS 보정 2.9 m).
  - **SETTLE 에서 경로 끝을 지나 영원히 대기했다.**
  - **미션 전환 때 노드가 `reset()` 을 `apply()` 보다 먼저 불러** 경로 반납이 안 됐다. 테스트는 실제 호출 순서로 짠다.
  - **`prepare_gap` 을 무겁게 하면 판단 노드가 0.6 s 멈춰 제어기가 경로를 잃는다.** 후보 수를 늘리면 반드시 시간을 잰다.
  - **나란함(`ABREAST`) 같은 "지금 위치" 조건을 강제 합류(테이퍼)에 걸면 교착한다.**
  - `pgrep -f "<명령>"` 으로 대기하면 자기 자신과 일치해 끝나지 않는다.
  - 스윕 스크립트가 `low_speed.yaml` 을 바꾸는 동안 같은 파일을 편집하지 말 것.
  - 시뮬 GPU 가 90% 근처면 라이다가 8 Hz 로 떨어지고 인식이 흔들린다. 시뮬 PC 의 Discord·Epic·Steam·NVIDIA 오버레이를 끄게 한다.
  - `PROJECT_MASTER.md` 의 예외구간 끝 "1878" 은 틀렸다. 실제는 CP13 = s1741.7 (§1.3).

- `colcon build` 를 `--symlink-install` 없이 하면 테스트가 옛 install 코드를 돌린다.
- pytest 를 파이프로 거르면 실패가 숨는다. `rc` 로 확인한다.
- `git revert -q` 가 조용히 실패한 적이 있다. 결과를 `git log` 로 확인한다.
- 벽 검출:
  - 가장 바깥 면을 쓰면 먼 반사가 붙는다 → 중앙값을 쓴다.
  - 좌우는 y 부호가 아니라 경로 d 부호로 정한다.
  - 도로 위 점(바리케이드)은 벽 면에서 제외한다.
- 러너 오판:
  - 텔레메트리 한 샘플이 0 이 되는 글리치가 있다.
  - 출발 전과 먼 거리의 충돌 패킷이 들어온다.
  - 정답 속도 단위는 km/h 다.
- **가속 가정 진입 판단** (`ACCELERATING`): 앞차 때문에 속도 상한이 막혀 있으면 실제로는 가속할 수 없다. 이 때문에 빠른 뒤차 앞으로 끼어들어 추돌당했다. 지금은 `REAR_CLOSING` 이 막는다.
- **전역 경로 기준 속도 상한을 로컬 경로에 씌우지 말 것.**
  - 전역 경로는 CP12(s1594) 에 11° 링크 꺾임이 있어 30 km/h 까지 떨어졌다.
  - 고속도로는 차선 중심선 기반의 자체 경로를 쓴다.
- 속도 상한 판단은 앞쪽 경로 길이가 **제동거리보다 길어야** 한다. 100 m 창으로는 92 km/h 에서 부족했다.
- 전역 경로 곡률은 ±1.5 m 창에서 흔들림이 급커브처럼 보인다. 곡선 속도가 이상하게 낮으면 창 크기부터 의심한다.

---

## 7. 사용자 선호·작업 방식 (새 에이전트가 반드시 따를 것)

- **언어**: 한국어로 대화한다.
- **요청 창구**: 사용자는 `practice/TuningPoint.md` **맨 위**에 요청을 쓴다.
  - 작업 전에 새 항목이나 미완료 항목을 확인한다.
  - 처리하면 그 아래 "결과 보고"에 상태 표시(✅ / 🔄 / ⏳ / ❓)를 달고, 원인(bag 근거) · 수정(커밋) · 검증 · 남은 것을 채팅 결과보고처럼 쓴다. 요청 원문은 고치지 않는다.
- **보고 방식**: 무엇이 문제였는지 수치와 위치(s, km/h, 시각)로 말하고, 무엇을 바꿨는지 커밋과 함께 말한다. 검증 안 된 것은 안 됐다고 분명히 말한다.
- **속도 철학**:
  - 강제 속도 상한을 싫어한다. 차선변경 중 고정 상한, 회전교차로 20 km/h, 터널 고정 상한 모두 거부했다.
  - 물리(곡률·횡가속도·제동거리)나 앞차 거리로 속도가 정해지게 한다. "예상 속도"는 괜찮지만 "제한"은 안 된다.
- **고속도로 전략**:
  - CP10~11 에서는 첫 차 뒤로 양보해 묶이지 말고, 남은 거리가 되면 **가속해서 제치고** 들어간다.
  - 나란한 상태로 끼어들지 않는다. 끼어들 때는 뒷범퍼 기준 여유를 충분히 둔다.
  - 앞차 간격은 너무 멀지 않게 한다(1.6 s 는 과했다).
  - 차선변경 중 불필요한 브레이크를 싫어한다.
- **규정 우선**: 60 km/h 초과 금지(예외구간 CP10~CP13 밖), END 는 3 m 통과만 필요(서지 않아도 됨), 금지 채널은 practice 에서만 쓴다.
- **브랜치**: 실험은 브랜치에서 하고, main 머지는 사용자 동의 후에 한다.
- **시뮬 조작**: 사용자가 시뮬 화면을 보며 관찰한 것을 알려 준다(예: "차선 밟음", "브레이크 이유 없음"). 그 순간을 bag 에서 찾아 원인을 증명한다.

## 8. 새 세션 시작 절차

1. 이 문서를 읽는다. §2.1 로 환경(IP·SSH·시나리오)이 맞는지 확인한다.
2. worktree `2026foscar-competition` 에서 `git pull`. 빌드는 `colcon build --symlink-install`, 그 전에 §2.3 의 `export`.
3. 테스트(§3.1): planning 의 옛 실패 5개 말고는 전부 통과해야 한다.
4. 시뮬 상태를 확인한다(§2.2, GPS 패킷 수).
5. `practice/TuningPoint.md` 맨 위의 요청을 확인하고 §5.3 을 이어서 진행한다.

