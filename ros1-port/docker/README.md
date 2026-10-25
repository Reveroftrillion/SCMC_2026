# CTRL 팀 Docker 실행 환경

현재 실행 절차는 [ROS1 안내](../README.md)를 따른다. 이 디렉터리는 Noetic/Python 3.10 빌드 환경, MORAI 수신·기록·검증 도구를 관리한다. 외부 참고 코드의 ROS2 개발 환경과 CTRL ROS1 실행 환경을 구분한다.

## 구성

| 파일 | 역할 |
| --- | --- |
| [Dockerfile](Dockerfile) | Ubuntu 22.04 CUDA 기반 Python 3.10/Noetic 환경 구성 |
| [build.sh](build.sh) | `rmt-noetic-py310:local` 이미지 빌드 |
| [run.sh](run.sh) | 저장소를 `/workspace`에 연결해 명령 실행 |
| [build_workspace.sh](build_workspace.sh) | catkin 빌드와 환경 검사 |
| [local.launch](local.launch) | 호스트별 네트워크 인자와 대회 launch 연결 |
| [verify_workspace.sh](verify_workspace.sh) | 빌드/메시지/native 검증 진입점 |

버전·원본 dependency hash는 Dockerfile과 lock 파일을 기준으로 확인한다. 예전 문서의 개인 PC 경로나 특정 센서 각도를 새 환경의 보장값으로 사용하지 않는다.

## 준비와 빌드

저장소 루트의 Linux 터미널에서:

```bash
bash ros1-port/docker/build.sh
bash ros1-port/docker/run.sh bash docker/build_workspace.sh
bash ros1-port/docker/run.sh bash docker/verify_workspace.sh
```

이 문서를 다시 쓴 작업에서 위 빌드나 GPU 검사를 실행하지 않았다. 새 CTRL 패키지 이름 변경 후의 실제 빌드 결과는 따로 확보해야 한다.

## 수신 전용 실행

```bash
bash ros1-port/docker/run.sh --receive-root roslaunch ctrl_start.launch drive:=false
```

`--receive-root`는 현재 낮은 수신 포트 909를 사용하기 위한 실행 옵션이다. run.sh는 이 경우 필요한 capability와 읽기 전용 source mount를 설정한다. `--capture-control`은 수신전용 검증 도구에서 제어 UDP 관측 권한이 필요할 때 사용한다. 옵션은 [run.sh](run.sh)의 실제 구현을 따른다.

## 개발 기록

기록은 `artifacts/` 아래 새 실행 폴더에 저장한다. 센서 입력·코드 버전·실행 인자·종료 상태를 함께 남긴다. 원본 bag/영상, 빌드 결과와 로컬 환경은 커밋 대상에서 제외한다.

과거 환경·native 결과는 [Noetic 기록](../docs/NOETIC_DOCKER_VALIDATION.md), 과거 주행 조건은 [실험 기록 목차](../docs/MARKDOWN_INDEX.md)에서 찾는다. 현재 재구현 결과와 이전 실행 결과를 합산하지 않는다.

## 이전 문서 원문

[교체 전 원문과 당시 기록](../../%EA%B3%BC%EA%B1%B0%EC%9E%90%EB%A3%8C/ctrl-document-history-20261025/ros1-port/docker/README.md). 원래 작성자·날짜·출처와 실험 수치를 보존했다.
