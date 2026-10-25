# 이 Ubuntu MORAI PC의 격리 실행 환경

현재 실행은 [ROS1 README](../README.md), 실제 성공·미해결 범위는
[2026-10-22 상태](../docs/CURRENT_STATUS_2026-10-22.md)를 먼저 확인한다.
시작점에서는 `bash ros1-port/docker/run_course.sh drive:=true bag:=true`를
저장소 루트에서 실행한다. 기동 전 수신 점검은 drive:=false로 한다.
이 래퍼는 현재 코드를 사용하며 새 폴더에 원문 로그·원본 센서를 기록한다.
정확한 START 한 점 대신 반경3m 안을 허용한다. 자동 종료 감독은 없다.
아래 10/13 문제/명령과 환경 준비 수치는 과거 기록이다.

2026-10-13의 실제 연결·직접 실행과 두 번째 교차로 신호 정지 문제를 이어서 볼 때는
[ROS1 README의 다음 작업 인계](../README.md#2026-10-13-morai-실행과-다음-작업-인계)를 먼저 읽는다.
그 문서에 Competition 재연결 방법, 현재 노드·신호·정지 계산, 실제 bag 근거와 원본 영상/YOLO 로그를
호스트에 보존하는 명령을 정리했다. 아래 Docker 환경 준비 이력과 전체 미션 검증을 구분한다.
일일 마감과 업로드 제외 범위는 [2026-10-13 작업 기록](../docs/WORK_LOG_2026-10-13.md)에 있다.

호스트 Python 3.8, 사용자 pip 패키지, `/opt/ros/noetic`은 마운트하거나 변경하지 않는다.
컨테이너는 Ubuntu 22.04의 CPython 3.10 ABI로 필요한 Noetic underlay와 `tf2_py`를 소스 빌드한다.
CUDA 12.6.3/cuDNN 9 runtime 이미지의 amd64 digest, Noetic release 아카이브 SHA256,
Python 패키지 버전을 고정한다. 원본 `numpy==1.24.4`, `onnxruntime-gpu==1.23.2`는 유지한다.
APT 저장소는 Ubuntu Jammy를 사용하며 실제 설치 버전은 이미지 `/opt/rmt/dpkg-manifest.txt`에 기록한다.
장기적으로 APT 저장소에서 사라진 버전까지 재현하려면 검증 이미지를 함께 보관해야 한다.

Noetic 소스 목록은 `rosinstall_generator ros_comm common_msgs tf2_ros tf2_geometry_msgs rviz
--rosdistro noetic --deps --tar`로 만든 `noetic.rosinstall`이다. 각 archive는 `noetic.sha256.json`으로
검증한다. 구형 wstool의 PyYAML 6 비호환을 피해 `fetch_noetic.py`로 다운로드한다.
Noetic rosconsole 1.14.4에는 Jammy log4cxx 0.12의 포인터 API를 지원하는 Debian 공식 호환 패치만
적용한다. 패치는 `rosconsole-log4cxx.patch`로 보존하고 Dockerfile에서 SHA256을 검사한다.
로깅 구현과 해당 테스트의 포인터 타입 변환이며 RMT 소스나 의존성 버전을 변경하지 않는다.
Jammy log4cxx 헤더의 `std::shared_mutex` 때문에 C++ 표준은 빌드 옵션으로 17을 지정한다.
출처: https://sources.debian.org/src/ros-rosconsole/1.14.4-2/debian/patches/0002-be-compatible-with-log4cxx-0.11-and-0.12-0.13.patch
`install_rosdeps.py`는 rosdep의 apt 해석 결과를 한 번에 설치한다. skip하는 Python 키는
이미 `python310.lock.txt`로 설치한 모듈이며, 환경 검사를 생략하는 용도가 아니다.

## 준비와 빌드

저장소 루트에서 다음을 실행한다. 첫 명령은 호스트의 NVIDIA Container Toolkit 설치가 필요할 때만
로컬 터미널에서 실행한다. sudo 인증이 필요하며 Docker 설정을 백업하고 daemon을 재시작한다.
실행 중인 Docker 컨테이너가 있으면 중단한다. NVIDIA 공식 절차:
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

```bash
bash ros1-port/docker/setup_nvidia_toolkit.sh
bash ros1-port/docker/build.sh
bash ros1-port/docker/run.sh bash docker/build_workspace.sh
# 추가로 devel/install 환경과 실제 메시지 직렬화·native tf2·기존 adapter/packaging 테스트:
bash ros1-port/docker/run.sh bash docker/verify_workspace.sh
```

`build_workspace.sh`는 catkin devel 빌드, source, 기존 `check_noetic_environment.py --json` 검사를 실행한다.
호스트에서 만든 Python 3.8 devel/build 공간과 섞지 않는다. 이 저장소의 빌드 경로는 컨테이너 안에서
`/workspace/ros1-port`이므로 실행도 같은 컨테이너 마운트 경로를 사용한다.

## 이 컴퓨터의 네트워크

Linux `--network host`를 사용한다. 컨테이너의 `127.0.0.1`이 MORAI와 같은 네트워크 namespace다.
ROS master/advertise IP도 `127.0.0.1`이며 외부 ROS 호스트와 연결하는 구성은 아니다.

| 채널 | MORAI 송신 Host Port | RMT 수신 Destination Port |
| --- | ---: | ---: |
| Competition Vehicle Status | 9088 | 909 |
| GPS | 1110 | 1111 |
| IMU | 1113 | 1112 |
| VLP16 | 2369 | 2368 |
| Camera-7 RGB | 9090 | 9297 |

모두 Host/Destination IP `127.0.0.1`. Competition Host Port는 사용자가 `908`에서 `9088`로 변경했고
이후 실제 181바이트 패킷을 원본 파서로 검증했다. Ego Status `9100 → 9111`은 229바이트로 파서가
거부하므로 대체하지 않는다. 제어의 MORAI 수신 포트는 **9093**이다. 화면의 Destination 9082는
제어 수신 포트가 아니다. 원본 bridge는 송신 소켓을 특정 source port에 bind하지 않는다.

원본 YAML 대신 `local.launch`에서 ROS private parameter로 네트워크만 override한다.
낮은 수신 포트 909 때문에 현재 호스트에서는 `--receive-root`가 필요하다. 이 옵션은 root 컨테이너에
`NET_BIND_SERVICE`만 주고 원본 및 이식본 src를 읽기 전용으로 마운트한다. 일반 빌드는 호스트 UID로
실행한다. artifacts는 그룹 쓰기와 setgid를 사용하므로 root 검사 결과도 호스트 사용자가 관리할 수 있다.

## 제어 송신을 끈 검증

MORAI bridge와 동시에 probe를 실행하면 수신 포트가 충돌한다. probe는 전용으로 실행하고 종료한다.
probe와 검증 도구는 제어 패킷을 보내지 않는다.

```bash
bash ros1-port/docker/run.sh --receive-root python3 docker/probe_morai.py \
  --seconds 20 --output docker/artifacts/udp.json --save-images docker/artifacts/cameras
bash ros1-port/docker/run.sh python3 docker/check_gpu_inference.py \
  --image docker/artifacts/cameras/camera-port-9297.jpg --output docker/artifacts/gpu.json
bash ros1-port/docker/run.sh --receive-root --capture-control python3 docker/verify_live.py \
  --seconds 40 --output docker/artifacts/live.json
```

`verify_live.py`는 별도 ROS master가 이미 있으면 중단한다. `drive:=false`로만 launch하고 실제 토픽과
TF를 구독한 뒤 SIGINT로 종료한다. `NET_RAW`는 loopback의 UDP destination 9093 송신 여부를
수동 캡처하기 위한 추가 권한이다. 센서·판단 메시지나 `/clock`을 주입하지 않는다.

현재 Camera-7은 **640×480, pitch 340°, 위치 1.90/0/1.20m, FOV 90°, 30Hz**이다.
원본 주행 README/보정 파일은 **Camera 7 RGB, 1280×720, pitch 335°**를 요구한다.
원본 주행 당시 위치·FOV는 확인되지 않았다. 수집용 pitch 0° 설정을 섞지 않는다.
현 조건의 JPEG 수신과 GPU 추론은 진단이며 원본 ROI/신호 판정 입력의 동등성 증거가 아니다.
Camera-1 또는 측면 카메라를 대신 사용하지 않고 알고리즘·ROI도 변경하지 않는다.

실제 MORAI VLP16 위치는 `1.19/0.01/1.25m`, GPS는 `0.52/0/1.21m`, IMU는
`0.75/0.04/1.23m`이며 세 센서의 화면 회전은 모두 0°다. 원본 LiDAR TF는
`2.0/0/2.11m`, GPS offset과 IMU yaw offset은 0이다. MORAI 차량 원점과 원본
base_link/후륜축 사이 변환 및 좌표축 규약이 확인되지 않아 화면 값을 TF/offset에 대입하지 않는다.
실제 장착 좌표의 동등성은 TF 메시지의 수신·lookup 통과와 별개로 미검증이다.

## 기존 테스트와 보존 검사

```bash
bash ros1-port/docker/run.sh bash docker/verify_comparison.sh
```

검증은 일회용 복사본에서 한다. Linux 원본 checkout 497개 파일이 Windows manifest의 CRLF와 달라
checkout 자체의 기존 보존 검사는 실패한다. `prepare_comparison.py`는 원본 작업 파일을 수정하지 않고
일회용 사본에서만 CRLF를 복원한다. 기존 manifest의 크기/SHA256과 정확히 일치할 때만 허용하며
그 외 차이는 중단한다. 원본의 원시 checkout 실패와 복원 사본의 통과를 별개로 보고한다.
기존 검사/manifest는 수정하지 않는다. 원본/이식본 파일의 캐시 부작용도 사본 안에 한정한다.

`verify_comparison.sh`는 모든 검사 exit code를 기록하고 하나라도 실패하면 실패한다.
모델 README의 기존 SHA256 불일치는 preflight 실패로 남는다. 실제 모델은 원본/이식본 동일하며
별도의 GPU 검사에서 초기화·shape/class guard와 실제 추론을 확인한다.

## 수동 진단과 이후 주행 명령

```bash
# 제어 UDP 송신 없음. 종료는 Ctrl+C.
bash ros1-port/docker/run.sh --receive-root roslaunch docker/local.launch drive:=false

# 아래 명령은 이후 주행용이며 이번 작업에서는 실행하지 않는다.
# 원본 센서 조건 및 남은 검증을 확인하고 MORAI에서 주행을 준비한 후 사용한다.
bash ros1-port/docker/run.sh --receive-root roslaunch docker/local.launch drive:=true bag:=true
```

카메라 원본 입력 조건, 실제 ROS2/ROS1 transport·clock·동일 순서 replay와 전체 미션 주행은
이 환경 준비만으로 완료되지 않는다. 검증 결과는 `../docs/NOETIC_DOCKER_VALIDATION.md`를 참조한다.
현재 SIGINT 종료 시 카메라 두 노드의 ExternalShutdownException traceback도 남는다.
이 현상과 센서 입력/좌표 조건을 해결하기 전에는 주행 명령을 검증 완료 명령으로 판단하지 않는다.

## 허용 후 수행한 짧은 실제 주행

사용자가 현 센서 차이를 유지한 첫 짧은 시험을 허용해 2026-10-13에 한 번 실행했다.
실제 결과와 제한은 [첫 주행 기록](../docs/MORAI_SHORT_DRIVE_2026-10-13.md)에 있다.
짧은 시험 도구는 다음 명령으로 원본 launch를 drive=true/bag=true로 시작하고, 실제 속도·s·상태를
수동 관측하다 SIGINT로 종료한다. 이 명령은 실제 MORAI 제어 송신을 시작한다.

```bash
bash ros1-port/docker/run.sh --receive-root --capture-control \
  python3 docker/drive_short_trial.py --output-dir docker/artifacts/drive-next
```

출발 후 3초/진행 12m/속도 8m/s와 오차·freshness에 따른 외부 시험 종료 조건이며 제어기 튜닝을
변경하지 않는다. 실제 정지까지는 감속 거리와 시간이 더 필요하다. MORAI Pause 해제 시점과
비상 Pause 방법을 먼저 확인한다. 현재 local.launch의 bag_dir 기본값은 호스트 artifacts의 절대 경로다.
시험 도구도 상대 출력 경로를 절대 경로로 바꾸며, ROS 로그/HOME CSV를 같은 폴더에 보존한다.
최초 시험의 bag은 호스트에 남지 않았고 후속 정차 bag 저장 확인과 구분한다.
