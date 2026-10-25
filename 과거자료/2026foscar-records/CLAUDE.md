# 에이전트·새 팀원 안내: 시뮬 PC 연결과 원격 조작 (제출 전 삭제: practice/HANDOFF.md §6)

마지막 갱신: 2026-10-08. 이 파일은 **노트북에서 SSH 로 시뮬 PC 에 붙고, UDP 로 MORAI 시뮬레이터를 조작하는 방법**만 다룬다.
명령어·패킷 형식·문제 해결 등 자세한 내용은 `practice/HANDOFF.md` 에 있다.

---

## 1. 구성

```
노트북 (192.168.1.3, 유선 enp4s0)  ──SSH 22──▶  시뮬 PC (192.168.1.2, 계정 foscar)
        │  ◀──UDP 센서·상태 (GPS 9102, IMU 9104, 상태 9099 …)──        │  MORAI Simulator_v.S4.251001.MolitComp03_Linux
        └──UDP 제어 9093 / 시나리오 로드 9095 ──▶                      │
```

- **SSH** 는 파일 복사(시나리오·네트워크 설정)와 시뮬 PC 상태 확인에만 쓴다. 차를 움직이는 건 **UDP** 다.
- **시뮬 주소는 `src/foscar_bringup/config/low_speed.yaml` 의 `sim_ip` 한 줄에만 있다.** 주행 코드와 `practice/` 도구(러너·시나리오 로드·gRPC)가 전부 여기서 읽는다.
- 노트북 주소는 코드에 없다(수신은 `0.0.0.0`). 노트북 OS 설정과 시뮬 쪽 Network 설정의 목적지(destination)에만 들어간다.

## 1.1 대회장 (SSH 없음)

1. 노트북 유선 주소를 대회장에서 정해 준 값으로 바꾼다: `nmcli con mod "Wired connection 1" ipv4.addresses <노트북IP>/24 && nmcli dev reapply enp4s0`
2. `low_speed.yaml` 의 `sim_ip` 를 시뮬 PC 주소로 바꾼다(symlink 빌드라 재빌드 필요 없음).
3. `python3 tools/check_sim.py` 로 ping 과 9099·9102·9104 수신을 확인한다(시뮬 쪽 목적지가 노트북 주소인지 여기서 드러난다).
4. `bash run_competition.sh`

## 2. 바로 쓰는 명령

```bash
ssh morai                                   # 시뮬 PC 접속 (~/.ssh/config 별칭, 키 ~/.ssh/morai_sim)
ping -c 2 192.168.1.2                       # 접속이 안 되면 먼저 확인. "No route to host" = 시뮬 PC 꺼짐 또는 네트워크 문제

# 시나리오 파일을 시뮬 PC 로 복사
scp practice/scenarios/<이름>.json "morai:Desktop/2026_국토부/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/"

# 시나리오 로드(UDP 9095). --send 없이 실행하면 패킷만 출력한다
python3 practice/sim/sim_scenario.py <이름> --send --network

# 시나리오 로드 + 주행 스택 실행 + 판정까지 한 번에
python3 practice/sim/loop_runner.py <이름> --runs 1
```

## 3. 꼭 지킬 것

1. **시뮬 화면이 주행 맵에 들어가 있어야 한다.** 런처나 메뉴 화면에서는 UDP 로 아무것도 안 된다. 맵 진입은 시뮬 PC 앞에서 사람이 한다.
2. **시나리오를 로드하면 시뮬이 네트워크 연결을 끊는다.** UDP 로드할 때는 항상 `--network` 를 붙인다(러너는 자동으로 붙인다).
3. **제어 패킷은 payload 23 B** 다. 공식 예제의 27 B 는 에러 없이 무시된다.
4. **주행 코드(`src/`)는 허용 채널만 쓴다.** 제어 9093, 상태 9099, GPS 9102, IMU 9104, LiDAR 9302, 카메라 92xx.
   Object Info, 전체 Ego Status, 시나리오 로드, gRPC, 충돌 채널은 `practice/` 도구에서만 쓴다(실격 사유).
5. 시뮬 PC 의 저장 파일을 덮어쓰기 전에는 `ssh morai ls …` 로 무엇이 있는지 먼저 확인한다.
