# CTRL 팀 ROS1 실행 안내

이 안내는 현재 `ctrl_*` 패키지의 설치·수신 점검·주행·기록을 설명한다. 새 제어 모듈은 오프라인 개발 중이며 기본 launch의 실행 제어기는 기존 ROS1 이식·수정본이다. 전체 상태는 [팀 README](../README.md)와 [개발 계획](docs/DEVELOPMENT_WORKFLOW.md)을 따른다.

## 환경 구축

Linux ROS1 Noetic 환경과 저장소의 Python 3.10 Docker 구성을 사용한다. Windows에서 수행한 새 모듈 검사는 Linux catkin 빌드 성공의 근거가 아니다. 저장소 루트에서:

```bash
bash ros1-port/docker/build.sh
bash ros1-port/docker/run.sh bash docker/build_workspace.sh
```

첫 명령은 이미지 준비가 필요할 때 실행한다. 이름 변경으로 `ctrl_interfaces`의 메시지 패키지도 바뀌었으므로 예전 devel/install 결과와 섞지 않는다. 의존성·실행 권한은 [Docker 안내](docker/README.md), 최초 이식 당시의 검증은 [기존 native 기록](docs/NOETIC_DOCKER_VALIDATION.md)을 참고한다.

## MORAI 준비

기존 검증 장면은 `2026_molit_comp_sample_scene.json`이다. 장면과 시작 위치, Manual/P, Pause와 센서 연결을 확인한다. 새로운 장면·날씨·차량 배치의 통과는 별도 검증한다.

| 채널 | 현재 local.launch 기준 |
| --- | --- |
| Competition Vehicle Status | 수신 UDP 909 |
| Ego Ctrl Cmd | MORAI 수신 UDP 9093 |
| GPS / IMU | 수신 UDP 1111 / 1112 |
| VLP16 / Camera-7 | 수신 UDP 2368 / 9297 |

포트의 기준은 [docker/local.launch](docker/local.launch)다. 장면을 다시 로드한 뒤에는 설정 화면뿐 아니라 실제 수신도 확인한다. 다른 ROS master/bridge와 수신 포트를 중복 점유하지 않는다.

## 대회용 launch 한 번으로 실행

호스트 명령 연결이 설치된 Linux 터미널에서 수신 점검:

```bash
roslaunch ctrl_start.launch drive:=false
```

점검을 종료하고 주행 준비를 마친 뒤:

```bash
roslaunch ctrl_start.launch
```

기본값은 `drive:=true bag:=true raw:=true`다. 기본 실행은 제어를 송신할 수 있으며, 일반 launch 자체에 충돌·장기 무진행 자동 종료 감독이 있는 것은 아니다. 시작점 초기화는 원래 START 반경 3m 조건을 사용한다. 중간 위치에서 임의로 첫 랩을 시작하지 않는다.

### 다른 PC의 한 번 설치

환경·빌드를 준비한 뒤 저장소 루트에서 호스트 명령 연결을 등록한다.

```bash
python3 ros1-port/tools/install_host_roslaunch.py
source ros1-port/docker/host_setup.bash
```

이 도구는 호스트 설정 파일을 변경한다. 구체적인 설치·해제 범위는 [호스트 연결 기록](docs/HOST_ROSLAUNCH_2026-10-23.md)에 있다. 이미 등록된 환경의 새 터미널에는 반복 설치하지 않는다.

### 기존 Docker·bash 실행 방법

호스트 명령 연결을 쓰지 않는 수신 점검:

```bash
bash ros1-port/docker/run.sh --receive-root roslaunch ctrl_start.launch drive:=false
```

중간 위치에서 첫 랩 초기화 없이 수신만 확인할 때:

```bash
bash ros1-port/docker/run.sh --receive-root roslaunch docker/local.launch drive:=false
```

`run_course.sh`의 무인자 기본값은 drive=false이며 직접 launch의 기본값과 다르다. 명령을 혼용하기 전에 실제 인자를 확인한다.

## 종료·기록

실행한 터미널에서 Ctrl+C 후 실제 속도 0과 Manual/P 인계를 확인한다. 정상 종료 확인 문자열은 `shutdown handoff confirmed: Manual/P, brake released`다. Pause 상태 자체를 인계 완료로 판정하지 않는다.

기본 실행의 기록은 `ros1-port/docker/artifacts/ctrl_start_*` 아래에 생성된다. bag·raw 센서·follower CSV를 실행별 새 폴더로 보존하며 Git에는 올리지 않는다. `drive:=false`의 실제 송신 0 확인이 필요한 경우 [verify_live.py](docker/verify_live.py)의 수신전용 점검을 별도 수행한다.

## 새 제어 개발 확인

Windows PowerShell에서 저장소 루트 기준:

```powershell
& .\ros1-port\.python310\python.exe -X utf8 -B .\ros1-port\tools\validate_path_selection.py
```

이 검사는 새 계산 계층의 오프라인 검사다. production launch를 실행하거나 제어 패킷을 보내지 않는다. 새 모듈의 명세·수치·미검증 범위는 [현재 결과](docs/PATH_SELECTION_RESULT_2026-10-25.md)에 있다.

## 이전 문서 원문

[교체 전 원문과 당시 기록](../%EA%B3%BC%EA%B1%B0%EC%9E%90%EB%A3%8C/ctrl-document-history-20261025/ros1-port/README.md). 원래 작성자·날짜·출처와 실험 수치를 보존했다.

이전 문서의 절 링크는 보관 원문에서 확인한다.

<a id="2026-10-13-morai-실행과-다음-작업-인계"></a>
