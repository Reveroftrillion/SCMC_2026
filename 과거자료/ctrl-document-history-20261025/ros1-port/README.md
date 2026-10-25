# ROS1 / MORAI 실행 안내

2026-10-24 명명 정리: 패키지·자체 모듈·실행 파일·launch는 `ctrl_*`를 사용한다.
최초 재빌드와 이전 경로 대응은 [CTRL 이름 안내](docs/CTRL_NAMES_2026-10-24.md)를 따른다.

2026-10-23 최신 수정 후 **터널 왼쪽·중앙·오른쪽 단일 상자 장면에서 각1회 감독 완주**했다.
세 주행 모두 제공 CP 반경15/15 통과·원본 충돌 보고0·정지/Manual/P 인계,
생산 소스349개 변화0을 확인했다. 이번 묶음 전체는5회 중4회 완료이며,
수정 전 왼쪽 반복1회는 실제 충돌로 중단해 원본을 보존했다.

최종 계산 회귀는979개 중978 PASS/기존 ROUNDABOUT 기대값1 FAIL/skip0, 도구28 PASS다.
합성 조합1,167개 중 원거리 연속 장애물4조건은 실패 상태다.
다음 실제 시험은 두 번째 장애물의 진행 중 재계획·제동, 터널 보행자 재진입·차단 해제와
관측 공백 재현이다. 각 배치1회 성공은 반복 안정성이나 공식 점수 확인을 뜻하지 않는다.
Windows·새 catkin/native 설치 빌드는 이번 단계에서 실행하지 않았다.

[터널 수정·실험 기록](docs/TUNNEL_CONTINGENCY_2026-10-23.md) ·
[최종 검증 요약](docs/tunnel-contingency-20261023/batch-01/final-state-right-complete.json) ·
[현재 작업 계획](docs/DEVELOPMENT_WORKFLOW.md) · [전체 상태](../README.md)

앞선 [사용자 완주2회](docs/USER_FULLCOURSE_2026-10-23.md),
[로터리 수정](docs/ROUNDABOUT_WAIT_2026-10-23.md),
[정적 수정](docs/STATIC_STOP_2026-10-23.md),
[터널 재질 수정](docs/TUNNEL_MATERIAL_EDGE_2026-10-23.md),
[10/22 반복 검증](docs/STABILITY_AUDIT_2026-10-22.md)은 당시 결과로 보존한다.

## 환경 구축

현재 실검증 환경은 Ubuntu의 `rmt-noetic-py310:local` Docker 이미지다.
[Docker 환경 구성](docker/README.md), [Noetic 환경](docs/NOETIC_ENVIRONMENT.md),
[기존 native 검증](docs/NOETIC_DOCKER_VALIDATION.md)을 참고한다.

```bash
cd /home/foscar/RMT-ros1
bash ros1-port/docker/build.sh
bash ros1-port/docker/run.sh bash docker/build_workspace.sh
```

첫 명령은 Docker 이미지·GPU/underlay 준비 조건이 필요하며 네트워크 다운로드를
포함할 수 있다. 기존 이미지/빌드가 있으면 매 주행마다 다시 만들지 않는다.
코드는 bind mount되며 새 모듈/빌드 변경 시 워크스페이스를 재빌드한다.
ROS1 메시지·TF·토픽·포트·단위와 원본 데이터는 유지한다.

## MORAI 준비

같은 `2026_molit_comp_sample_scene.json`을 로드하고 원래 Ego 시작점,
Pause 해제·Manual/P로 준비한다. Reset/장면 로드 후 채널이 끊길 수 있으므로
화면에서 다시 Connect하고 실제 수신으로 확인한다.

| 채널 | 현재 호스트 설정 |
| --- | --- |
| Competition Vehicle Status | 127.0.0.1:9088 → 127.0.0.1:909 |
| Ego Ctrl Cmd | MORAI 수신9093 |
| Collision Data | 출력 목적지9092 |
| GPS / IMU | 수신1111 / 1112 |
| VLP16 / Camera-7 | 수신2368 / 9297 |

Ego Status9111을 Competition Status909 대신 사용하지 않는다.
기존 ROS/master/기록기를 중복 기동하지 않는다. MORAI의9088/9093 점유는 정상이다.

## 대회용 launch 한 번으로 실행

**이 PC에는 호스트 명령 등록이 끝났다.** 위 MORAI 준비를 마친 뒤 새 Bash 터미널에서:

```bash
roslaunch ctrl_start.launch
```

어느 폴더에서든 실행할 수 있으며 내부 Docker 진입은 자동이다.
**기본값은 `drive:=true bag:=true raw:=true`다. 기동 직후 출발할 수 있다.**
전체 센서·위치 추정·인지·미션 판단·추종·UDP 송수신과 기본/원본 bag 기록을 실행한다.
카메라는 기존대로5초 뒤 시작한다. 매 실행마다 이미지나 워크스페이스를 다시 빌드하지 않는다.

연결·센서만 먼저 확인할 때:

```bash
roslaunch ctrl_start.launch drive:=false
```

수신 점검을 Ctrl+C로 종료하고 주행 명령을 별도로 실행한다. `drive:=false`는
bridge의 제어 UDP 송신을 끈다. 엄밀한 송신0 검사는 [verify_live.py](docker/verify_live.py)의
passive capture를 사용하며 센서 수신만으로 판정하지 않는다.

설정 등록 전에 열어 둔 터미널이거나 `is not a launch file name` 오류가 나면 한 번 실행한다:

```bash
source ~/RMT-ros1/ros1-port/docker/host_setup.bash
```

이후 같은 터미널에서 `roslaunch ctrl_start.launch`를 다시 사용할 수 있다.
새 Bash 터미널은 위 source가 필요 없다. 연결은 정확히 첫 인자가 `ctrl_start.launch`인
명령에 적용되며, 다른 패키지의 roslaunch는 기존 호스트 ROS로 전달한다.

| 인자 | 기본값 | 용도 |
| --- | --- | --- |
| `drive` | `true` | `false`면 제어 UDP 송신을 끄고 수신·판단 확인 |
| `bag` | `true` | `false`면 기본 bag과 raw bag 기록기를 모두 끔 |
| `raw` | `true` | `false`면 추가 원본 센서 bag만 끔. 기본 bag은 bag 설정을 따름 |
| `run_dir` | 실행마다 새 디렉터리 | 직접 지정할 때는 `/workspace/ros1-port/docker/artifacts/` 아래 새 경로 사용 |

종료는 실행 터미널에서 Ctrl+C 한 번 후 실제 속도0·Manual/P 인계를 확인한다.
이 일반 주행에는 충돌·장기 무진행 자동 종료 감독이 없다. 감독 시험은
[audit_course.py](docker/audit_course.py)와 [감독 절차](docs/STABILITY_AUDIT_2026-10-22.md#기록재현복구)를 따른다.

### 다른 PC의 한 번 설치

호스트 ROS1 Noetic·Docker·GPU와 이 저장소의 이미지/워크스페이스 빌드를 먼저 준비한다.
현재 PC의 명령 연결만 설치할 때는 저장소 루트에서:

```bash
cd ~/RMT-ros1
python3 ros1-port/tools/install_host_roslaunch.py
source ros1-port/docker/host_setup.bash
```

설치 도구는 ~/.bashrc를 백업하고 연결 블록을 등록한다.
호스트에 Docker의 Python3.10 catkin devel/setup.bash를 직접 source하지 않는다.
[설치 범위·해제](docs/HOST_ROSLAUNCH_2026-10-23.md)를 참고한다.

### 기존 Docker·bash 실행 방법

호스트 명령 연결을 쓰지 않을 때도 기존 명령은 사용할 수 있다. 저장소 루트에서:

```bash
bash ros1-port/docker/run.sh --receive-root roslaunch ctrl_start.launch
```

Docker 터미널에 직접 들어가려면:

```bash
bash ros1-port/docker/run.sh --receive-root
# 열린 Docker 터미널의 /workspace/ros1-port에서 실행
roslaunch ctrl_start.launch
```

화면 반복 출력 축소·원문 console.log·실행별 ROS 로그도 필요하면 기존 래퍼를 사용한다:

```bash
bash ros1-port/docker/run_course.sh drive:=true bag:=true
```

run_course.sh의 **무인자 기본값은 drive=false**다. 직접 launch의 기본 drive=true와 구분한다.
래퍼도 같은 ctrl_start.launch를 사용하며 실행·제어 정책은 같다.

## 시작점부터 실행

위 `roslaunch ctrl_start.launch`는 START 반경3m 안의 첫 랩 전용이다.
정확히 같은 좌표 하나를 요구하지 않지만 반경 밖에서 s0을 강제로 부여하지 않는다.
반경 밖·중간 위치에서는 FAULT로 거부한다. 다음 주행 전 원래 시작점으로 Reset하고
Manual/P·채널 연결을 확인한다. 터널·로터리 중간에서 그대로 새 주행을 시작하지 않는다.

중간 위치의 수신 점검은 첫 랩 옵션이 없는 기존 local.launch로 별도 수행한다.
저장소 루트에서:

```bash
bash ros1-port/docker/run.sh --receive-root \
  roslaunch docker/local.launch drive:=false bag:=true
```

## 종료·기록

정상 종료는 실행한 터미널에서 Ctrl+C 한 번. 실제 속도0과
`shutdown handoff confirmed: Manual/P, brake released`를 확인한다.
인계 실패 로그가 있으면 성공으로 취급하지 않는다. 비상 시 MORAI Pause를
사용하되 Pause 상태 자체는 제동·인계 증거가 아니다.

`roslaunch ctrl_start.launch`의 기록은 호스트
`ros1-port/docker/artifacts/ctrl_start_*` 아래에 실행별로 생성된다.

- `bags/`: 기본 상태·인지·계획·제어 bag
- `raw.bag`: 원본 GPS·IMU·LiDAR·raw/debug 카메라·TF bag
- `follower/`: 경로 추종 CSV

기본 bag과 추가 raw bag은 bag=true일 때 기록하며, raw=false면 추가 raw 기록만 끈다.
`run_dir`를 직접 지정하면 컨테이너의 /workspace bind mount 안에 실행별 새 경로를 사용한다.
직접 launch의 ROS 로그는 컨테이너 기본 경로를 따르므로 영구 console/ROS 로그가 필요하면
위 run_course.sh 래퍼를 사용한다.

래퍼는 `docker/artifacts/manual-*`에 위 주행 자료와 원문 console.log·실행별 ROS 로그를 남긴다.
화면에는 반복 상태/통계를5초, 같은 경고를10초 간격으로 표시하며
상태 전환·ERROR/FATAL·인계는 즉시 표시한다. 필터 전 문구는 console.log에 보존한다.
YOLO JSONL의 영구 보존은 이 래퍼만으로 보장하지 않는다.
기록 파일명 출력의 개행은 수정했으며 과거 개행 포함 파일은 그대로 보존했다.
터미널 프롬프트의 `$`, 이어쓰기 프롬프트 `>`는 명령에 포함하지 않는다.

## 문제 구분

| 현상 | 먼저 볼 근거 |
| --- | --- |
| WAITING / pose 없음 | Status RX0인지, IMU·GPS만 수신되는지 |
| START radius FAULT | 현재 위치가 시작점 반경 안인지. 중간 재시작에 첫 랩 옵션을 썼는지 |
| AVOID v_max0 | 장애물·지도·곡선·BEV 관측 제약. 단순히 속도 제한을 높이지 않음 |
| NO_TARGET | planner 노드·입력 freshness·계산 공백 |
| FINISH | 경로/runout 종료. 이후 실제 속도·Manual/P 확인은 별도 |

## 검증·개발

[현재 작업 계획](docs/DEVELOPMENT_WORKFLOW.md),
[최신 터널 실험·검사 요약](docs/tunnel-contingency-20261023/batch-01/final-state-right-complete.json),
[이전10/22 알고리즘·native 검사](docs/verification-20261022/checks-summary.json),
[파일·미션 안내](docs/FILE_MISSIONS_2026-10-22.md),
[원래 이식 변경](docs/CHANGES.md)를 참고한다.

기존 README의 상세 실험·구현 설명은
[이전 안내 원문](docs/history/ROS1_README_before_2026-10-22.txt)에 보존했다.
해당 원문의 링크/명령은 당시 ros1-port/README.md 위치 문맥이며 현재 실행 지침과 구분한다.
원본 출처·작성자·라이선스는 각 소스/패키지의 선언을 유지한다.

<a id="2026-10-13-morai-실행과-다음-작업-인계"></a>
<a id="빌드와-실행"></a>

과거 문서의 실행/빌드 앵커는 호환성을 위해 유지한다. 현재 명령은 위 환경
구축·시작점 실행 절을 따르고, 10/13 당시 상세 기록은 이전 안내 원문을 참고한다.
