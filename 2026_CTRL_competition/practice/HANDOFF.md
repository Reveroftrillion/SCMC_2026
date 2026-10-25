# 시뮬 PC 연결·원격 조작 인계 문서 (practice/ 전용)

마지막 갱신: 2026-10-08. 노트북에서 **SSH 로 시뮬 PC 에 접속**하는 방법과, **UDP(및 연습용 gRPC)로 MORAI 시뮬레이터를 조작**하는 방법을 정리한 문서다.
요약은 저장소 최상위 `CLAUDE.md` 에 있다. `practice/` 폴더는 제출 전에 통째로 지운다(§6). 이 문서도 같이 지워진다.

---

## 1. 네트워크 구성

| 장비 | 주소 | 비고 |
|---|---|---|
| 노트북 (주행 코드 실행) | `192.168.1.3` | 유선 `enp4s0` 에 고정. 시뮬은 센서·상태를 전부 이 주소로 보낸다 |
| 시뮬 PC | `192.168.1.2` | 리눅스, 계정 `foscar`, SSH 22 (2026-10-08 새 PC) |

- 와이파이만 쓸 때는 노트북에 주소를 추가로 붙인다.
  `nmcli con mod <연결> +ipv4.addresses 192.168.1.3/24 && nmcli dev reapply <장치>`
- 2026-10-08 같은 공간의 다른 장비들과 주소가 겹쳐 혼선이 나서 `192.168.0.10/0.11` → `192.168.1.3/1.2` 로 옮겼다.
  예전 주소들(`192.168.0.10/0.11`, `192.168.0.8:9992`, `192.168.10.1`)은 더 이상 쓰지 않는다.
  `측정테스트파일/go.py` 의 기본 `--ip 192.168.10.1` 도 예전 값이라 쓸 때 `--ip 192.168.1.2` 를 준다.
- 시뮬 PC 방화벽(ufw)을 켜야 하면 `sudo ufw allow from 192.168.1.3 to any port 22` 와 아래 UDP 포트들을 연다.

### 1.1 주소를 바꾸는 곳 (한 군데)

- **시뮬 주소: `src/foscar_bringup/config/low_speed.yaml` 의 `sim_ip` 한 줄.**
  주행 코드(`sensor_bridge` 의 제어 송신 대상·수신 출발지 검사)와 `practice/` 도구(`sim_scenario.sim_ip()` 를 통해 `loop_runner.py`, `signal_director.py`, `weather_matrix.sh`)가 모두 여기서 읽는다.
  `tools/check_sim.py` 도 같은 값을 읽는다. config 는 symlink 빌드라 재빌드가 필요 없다.
- **노트북 주소는 코드 어디에도 없다**(수신 소켓은 `0.0.0.0`). 바꿀 곳은 노트북 OS(nmcli)와 시뮬 쪽 Network 설정의 destination 뿐이다.
- 시뮬 PC 자체 주소는 시뮬 PC OS 에서 바꾼다(아래). 시뮬 쪽 Network 설정·`EgoNetwork/*_MN.json` 의 `hostIP` 는 시뮬 PC 주소, `destinationIP` 는 노트북 주소다.

```bash
# 노트북 (유선 enp4s0)
nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.3/24 && nmcli dev reapply enp4s0
# 시뮬 PC (화면 앞에서, 연결 이름은 nmcli con show 로 확인)
nmcli con mod "<유선 연결 이름>" ipv4.method manual ipv4.addresses 192.168.1.2/24 && nmcli dev reapply <장치>
```

### 1.2 대회장 절차 (SSH·연습 도구 없이)

대회장에서는 SSH 도, `practice/` 도구도 못 쓴다(제출본에서 지워짐). 시뮬 PC 의 주소와 Network 설정은 주최측 것을 쓴다.

1. 주최측이 정한 노트북 주소로 유선을 맞춘다: `nmcli con mod "Wired connection 1" ipv4.addresses <노트북IP>/24 && nmcli dev reapply enp4s0`
2. `src/foscar_bringup/config/low_speed.yaml` 의 `sim_ip` 를 시뮬 PC 주소로 바꾼다. **다른 파일은 손대지 않는다.**
3. `python3 tools/check_sim.py` 로 ping 과 상태 9099·GPS 9102·IMU 9104 수신을 확인한다.
   ping 은 되는데 수신이 0 이면 시뮬 Network 설정의 목적지가 노트북 주소가 아니거나 해당 채널이 Connect 안 된 것이다.
4. `bash run_competition.sh`

## 2. SSH 연결

### 2.1 접속

노트북 `~/.ssh/config`:

```
Host morai
    HostName 192.168.1.2
    Port 22
    User foscar
    IdentityFile ~/.ssh/morai_sim
    IdentitiesOnly yes
    ConnectTimeout 8
    ServerAliveInterval 30
```

```bash
ssh morai                         # 접속
ssh morai 'pgrep -fa Simulator'   # 시뮬 실행 여부만 확인
```

- `No route to host` / 타임아웃이면 시뮬 PC 전원, 랜선, 노트북 주소(`ip -4 addr show enp4s0`)부터 확인한다.

### 2.2 새 노트북에서 처음 설정할 때

1. 예전 노트북의 `~/.ssh/morai_sim`, `morai_sim.pub` 를 옮긴다(`chmod 600 ~/.ssh/morai_sim`).
   새로 만들려면 `ssh-keygen -t ed25519 -f ~/.ssh/morai_sim` 한 뒤 공개키를 시뮬 PC `~/.ssh/authorized_keys` 에 추가한다.
2. 위 `Host morai` 블록을 `~/.ssh/config` 에 넣는다.
3. 시뮬 IP 는 `src/foscar_bringup/config/low_speed.yaml` 의 `sim_ip` 한 곳에만 맞춘다(§1.1).

### 2.3 새 시뮬 PC 를 준비할 때

1. `sudo apt install openssh-server` 후 노트북 공개키를 `~/.ssh/authorized_keys` 에 추가한다.
2. 시나리오·네트워크 파일을 복사한다(§2.4).
3. 시뮬 네트워크 설정에서 연습용 Ego Vehicle Status 송신 포트를 **9106** 으로 둔다(9101 은 GPS 와 겹친다).

### 2.4 시뮬 PC 의 파일 위치와 복사

- 쓰는 런처: `~/Desktop/2026_국토부/MoraiLauncher_Lin/` (시뮬 `Simulator_v.S4.251001.MolitComp03_Linux`).
  다른 설치본(`~/Desktop/MoraiLauncher_Lin`, `~/MoraiLauncher_Lin`)도 있으니 헷갈리지 않는다.
- 시뮬 PC 랜은 두 개다: USB 랜 `enx588694f73d11` = **192.168.1.2** (노트북 직결), 내장 `enp4s0` = 192.168.0.8 (인터넷선).
  노트북 1.3 으로 가는 패킷은 같은 대역인 1.2 쪽으로 나가므로 출발지가 `sim_ip` 와 맞는다.
- 저장 폴더 `…/MoraiLauncher_Lin_Data/SaveFile/`
  - 센서셋: `Sensor/25.S4.MolitComp03/`
  - 네트워크: `Network/25.S4.MolitComp03/NetworkInfo_2023_Hyundai_Ioniq5.json`
  - 시나리오: `Scenario/R_KR_PR_K-city_2025/` (`Practice3_START`, `Practice3_CP01~15`, 원본 `CompetitionPractice3` 등)
  - 시나리오별 네트워크: 같은 폴더 `EgoNetwork/<시나리오이름>_MN.json`

```bash
SC="morai:Desktop/2026_국토부/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
ssh morai 'ls Desktop/2026_국토부/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/'   # 덮어쓰기 전 확인
scp practice/scenarios/<이름>.json "$SC/"
scp practice/scenarios/EgoNetwork_template_MN.json "$SC/EgoNetwork/<이름>_MN.json"
```

- 새 시나리오를 만들면 `practice/scenarios/EgoNetwork_template_MN.json` 을 `<이름>_MN.json` 으로 복사해 둔다.
  이 파일이 있어야 UDP 시나리오 로드 뒤 네트워크 연결이 복구된다(§4.1).
- 템플릿을 다른 환경에 쓸 때는 안에 든 `hostIP`(시뮬)·`destinationIP`(노트북)가 현재 주소와 맞는지 먼저 확인한다.

## 3. UDP 채널

### 3.1 주행 코드가 쓰는 채널 (대회 허용)

| 채널 | 방향 | 노트북 포트 / 시뮬 포트 | 비고 |
|---|---|---|---|
| Ego Ctrl Cmd | 노트북 → 시뮬 | 시뮬 **9093** | cmd type 1(accel/brake), ctrl mode 2 만 허용 |
| Competition Vehicle Status | 시뮬 → 노트북 | **9099** | 181 B, 위치 없음(속도·자세·기어·모드·조향) |
| GPS / IMU | 시뮬 → 노트북 | **9102 / 9104** | 송신 주기 상한 30 / 50 Hz |
| LiDAR / Camera | 시뮬 → 노트북 | 9302 / 92xx | |

- 시뮬과는 **순수 UDP 만** 쓴다. MORAI ROS 연결(rosbridge)은 금지다.
- 리눅스에서는 **포트 4자리**를 쓴다. 3자리 포트는 동작하지 않았다.

### 3.2 연습 도구만 쓰는 채널 (주행 코드에서 쓰면 실격)

| 채널 | 포트 | 쓰는 도구 |
|---|---|---|
| Scenario Load | 시뮬 **9095** (응답 9096) | `sim_scenario.py`, `loop_runner.py` |
| Collision | 시뮬 9091 → 노트북 **9092** | `loop_runner.py`, `record_collision.py` |
| Object Info (주변 물체 정답) | 시뮬 7605 → 노트북 **7505** | `loop_runner.py` |
| Ego Vehicle Status 전체판 (정답 위치) | 시뮬 **9106** → 노트북 **9111** | `loop_runner.py` |
| MORAI gRPC API | 시뮬 **7789** (TCP) | `morai_rpc.py`, `signal_director.py` |

- 이 데이터는 판정·기록에만 쓴다. 인지·판단·제어에는 절대 넣지 않는다.

### 3.3 제어 패킷 형식 (9093)

```
'#MoraiCtrlCmd$' + <iiii>(23, 0, 0, 0) + payload 23 B + '\r\n'   = 55 B
payload = <bbbfffff> ctrl_mode, gear, cmd_type, velocity, acceleration, accel, brake, steer
```

- 주행: `ctrl_mode=2`(AutoMode), `gear=4`(D), `cmd_type=1`, `velocity=0`, `acceleration=0`, `accel`·`brake` 0~1, `steer` -1~1(왼쪽 +, 1.0 = 실효 34.9°).
- **반드시 23 B.** 공식 예제(24.R2)의 27 B(rear_steer 포함)는 이 대회 빌드에서 에러 없이 무시되고 차가 안 움직인다.
- 50 Hz 정도로 계속 보낸다. 구현: `src/foscar_control/foscar_control/core.py` 의 `ctrl_packet`.
- 끝낼 때(브리지 `handoff_packet`): 브레이크로 멈춤 → `gear=1`(P) 확인 → `ctrl_mode=1`(Manual)로 브레이크 해제. Manual 상태에서는 기어 변경이 무시되므로 P 를 먼저 확인한다.

### 3.4 상태 패킷 (9099)

- `'#MoraiInfo$'` 로 시작, 길이 181 B, payload 길이 필드 152. 파서: `core.py` 의 `parse_status`.
- 원격 조작 결과 확인용으로 본다: `mode`(1 Manual / 2 Auto), `gear`(1 P / 4 D), 속도(km/h), 조향각, 현재 링크.

## 4. 원격으로 시뮬 조작하기

### 4.1 시나리오 로드 (UDP 9095)

```bash
python3 practice/sim/sim_scenario.py <이름>                    # 패킷만 출력 (dry run)
python3 practice/sim/sim_scenario.py <이름> --send --network   # 자차·NPC·보행자·물체 리셋 + 저장된 네트워크 연결 복구
#   --keep-npc      NPC 를 다시 불러오지 않음
#   --keep-objects  정적 물체를 다시 불러오지 않음
#   --pause         로드 후 일시정지 상태로 둠
```

- 패킷: `'#ScenarioLoad$' + <i3i>(len, 0, 0, 0) + 이름(30 B, 공백 채움) + 플래그 7 B(delete_all, network, ego, npc, pedestrian, object, pause) + '\r\n'` = 69 B. 이름은 ASCII 30 B 이하.
- **로드하면 시뮬이 네트워크 연결(9099·ObjectInfo·충돌)을 끊는다.** GPS·IMU·LiDAR 는 계속 나온다. `--network` 를 붙이면 `EgoNetwork/<이름>_MN.json` 으로 복구된다.
- 시뮬 화면에서 사람이 직접 로드했다면: 로드 직후 `P`(내장 자율주행 정지)를 누르고 `F4 > Load` 로 네트워크 설정을 다시 불러온다.
- 로드하면 화면의 현재 배치가 바뀐다. 남겨야 할 배치가 있으면 시뮬에서 먼저 저장한다.

### 4.2 차 직접 움직이기 (UDP 9093)

```bash
python3 측정테스트파일/go.py --ip 192.168.1.2                         # accel 0.3 직진
python3 측정테스트파일/go.py --ip 192.168.1.2 --accel 0.5 --steer 0.3 # 왼쪽으로 돌며 전진
python3 측정테스트파일/go.py --ip 192.168.1.2 --brake 1.0             # 정지
```

- `go.py` 는 저장소 밖(`~/Desktop/2026Morai/측정테스트파일/`)에 있다. Ctrl+C 로 끝내면 브레이크 패킷을 1초 보내고 멈춘다.
- 주행 스택 전체를 돌릴 때는 `bash run_competition.sh` 가 같은 9093 으로 보낸다. **두 개를 동시에 켜지 않는다.**

### 4.3 시나리오 로드 + 주행 + 판정 한 번에

```bash
python3 practice/sim/loop_runner.py <이름> --runs 1 [--end-s 2165 --timeout 400] [--no-npc] [--rviz]
```

- 순서: 시나리오 로드(9095, network=True) → `--settle` 초 대기 → `run_competition.sh` 실행 → 충돌(9092)·ObjectInfo(7505)·정답 위치(9111) 수신 → 종료 조건에서 스택 정지.
- 결과는 `practice/sim/runs/<시각>/` 에 남는다.

### 4.4 신호등·시간·날씨 (gRPC 7789, 연습 전용)

- `practice/sim/morai_rpc.py` 의 `Morai(sim_ip()+':7789')` 로 시나리오 로드, 자차 정답 상태, 신호등 읽기·쓰기, 시간·날씨 변경을 할 수 있다.
- `practice/sim/signal_director.py` 는 이 API 로 카메라 신호를 빨간불로 고정했다가 차가 정지선에 서면 풀어 준다.

## 5. 동작 확인과 문제 해결

### 5.1 시뮬이 주행 맵 화면인지 확인

시뮬이 런처·메뉴 화면이면 시나리오 로드도 안 먹고 센서도 안 나온다. GPS 포트를 2초간 들어 본다.

```bash
python3 -c "import socket,time;s=socket.socket(2,2);s.bind(('0.0.0.0',9102));s.settimeout(.5);n=0;t=time.time()
while time.time()-t<2:
  try:s.recv(65535);n+=1
  except socket.timeout:pass
print(n)"
```

- 주행 화면이면 약 60~80 개가 들어온다. 0 이면 시뮬 PC 앞의 사람에게 맵 진입을 부탁한다.
- 다른 프로그램(주행 스택 등)이 이미 9102 를 쓰고 있으면 bind 에러가 난다. 그때는 그 프로그램의 로그를 본다.

### 5.2 증상별 정리

| 증상 | 원인·조치 |
|---|---|
| `ssh: … No route to host` | 시뮬 PC 꺼짐, 랜선, 노트북 `192.168.1.3` 미설정 |
| 러너 `NO_START:WAITING`, 로그 `RX={'gps': 0, 'imu': 0, 'status': 0}` | 시뮬이 맵 화면이 아님 |
| `RX={'gps': 1600, 'imu': 1187, 'status': 0}` | 9099 연결만 끊김. 시나리오 로드 때 `--network` 누락, 또는 센서셋을 바꾼 뒤라면 시뮬 Network Settings 에서 해당 채널을 Connect |
| 제어를 보내도 차가 안 움직임 | 패킷이 23 B 인지, 대상이 `sim_ip:9093` 인지, 시뮬 Network Settings 에서 Ctrl Cmd 가 연결돼 있는지 |
| 로드 직후 차가 혼자 출발 | 내장 자율주행. 시뮬에서 `P` |
| 포트 사용 중(bind 에러) | 이미 떠 있는 러너·주행·수집 프로그램을 끈다 |
| 센서셋 작업 중 돌린 회차 | 시뮬이 멈춰 있어 결과가 틀린다. 버린다 |

## 6. 제출 전 (필수)

```bash
bash practice/check_submission.sh   # practice/ 밖에 연습 채널 흔적이 없는지 검사 (exit 0 이어야 함)
git rm -r practice CLAUDE.md && git commit -m "chore: remove practice tools and agent notes for submission"
```

- 주행 코드(`src/`, `run_*.sh`, `status.sh`)는 `practice/` 를 import·실행·참조하지 않는다.
- 새 연습 도구는 반드시 `practice/` 안에 만들고, 만든 뒤 `check_submission.sh` 를 돌린다.
