# Ubuntu MORAI PC Docker 검증 기록

이 문서는 앞선 환경 준비와 drive=false 단계의 기록이다. 이후 명시 허용한 drive=true 첫 시험은
[짧은 실제 주행 기록](MORAI_SHORT_DRIVE_2026-10-13.md)에 별도로 정리한다.

2026-10-13 KST. 저장소 `/home/foscar/RMT-ros1`, 브랜치 `feat/ros1-port`, 시작 HEAD
`395c10276ecbfdbcb6eb81e0c742f4f1ad3333b7`. 적용할 AGENTS.md는 저장소 및 상위 경로에서 찾지 못했다.
시작 작업 트리는 clean이었다. 원격 push와 `drive:=true` 실행은 하지 않았다.

재현 명령과 Docker 설정은 [docker/README.md](../docker/README.md)에 있다.
검증 데이터는 [noetic-validation-2026-10-13](noetic-validation-2026-10-13/)에 보존한다.
기계 판독용 전체 상태는 [summary.json](noetic-validation-2026-10-13/summary.json)에 있다.

## 환경

호스트 Ubuntu 20.04.6, Python 3.8.10, ROS Noetic `/opt/ros/noetic`과 사용자 pip 설치는 변경하지 않는다.
초기 샌드박스의 Docker/GPU/socket 조회 차단은 승인된 호스트 조회로 구분했다. 호스트 Docker daemon
접근은 정상이다. NVIDIA Container Toolkit은 처음에 없었고 사용자가 제공한 스크립트를 로컬 sudo로
실행해 1.20.1-1을 설치했다. Docker NVIDIA runtime 등록과 실제 GPU 컨테이너의 `nvidia-smi`를 확인했다.

컨테이너는 amd64 CUDA 12.6.3/cuDNN runtime Ubuntu 22.04 이미지의 digest를 고정한다.
CPython 3.10.12, NumPy 1.24.4, ONNX Runtime GPU 1.23.2를 사용한다. 실제 공유 라이브러리는
CUDA cudart 12.6.77, cuBLAS 12.6.4.1, cuDNN 9.5.1.17이며 ONNX CUDA provider의 `ldd`에서
필요 라이브러리 누락은 없었다. 호스트 드라이버는 560.35.05, RTX 3070 Ti 8GB다.
자세한 라이브러리 해석 결과는 [cuda-libraries.txt](noetic-validation-2026-10-13/cuda-libraries.txt)에 있다.

Noetic release 소스 84개와 아카이브 SHA256을 고정하고 같은 Python 3.10으로 빌드한다.
Jammy의 log4cxx 0.12 API에는 rosconsole 1.14.4용 Debian 공식 호환 패치를 적용한다.
패치 SHA256은 `cfdd9c6c1ca22d0ddd80a3075cd8a1cde2cc6edf45b30e4042fb99f23e21a681`이다.
추가로 `std::shared_mutex`를 지원하도록 C++17 빌드 옵션을 지정한다. RMT 코드·계산식·상태 전환·모델·
지도·경로·튜닝값과 고정 실행 의존성을 바꾸는 변경은 아니다. 호스트 Python 3.8 ROS 경로를
Python 3.10에 붙이지 않는다.

최종 image ID는 `sha256:cb0e0d2d3937335e463bb4529ff8f195d866dcec2f1d40b743156dd10302c044`다.
rosconsole의 기존 단위 테스트는 **250개 통과, 오류/실패/skip 0개**였다. 실제 설치 manifest는
[pip-freeze.txt](noetic-validation-2026-10-13/pip-freeze.txt),
[dpkg-manifest.txt](noetic-validation-2026-10-13/dpkg-manifest.txt)에 저장했다.

## catkin·실제 메시지·native 모듈

첫 실제 catkin 빌드에서 최상위 `src/CMakeLists.txt`의 `catkin_workspace` 명령을 찾지 못했다.
공식 catkin top-level 초기화와 같이 `CATKIN_TOPLEVEL_FIND_PACKAGE`를 find_package 앞에서 설정하고
뒤에서 해제하는 두 줄만 추가했다. 이 수정은 패키징이며 RMT production Python과 설정은 변경하지 않았다.

| 검사 | 결과 |
| --- | --- |
| catkin devel 빌드/source | PASS |
| catkin install 빌드/source | PASS |
| 기존 check_noetic_environment.py | 두 공간에서 각각 10 PASS, 모델 README는 KNOWN_SOURCE_ISSUE |
| 실제 generated interface 4종 | 필드/타입/상수 및 모든 필드의 genpy 직렬화 왕복 PASS |
| 실제 PointCloud2 | little/big endian, organized cloud, NaN/intensity/padding wire 보존 PASS |
| 실제 Marker | quaternion w=1 기본값과 native 직렬화 PASS |
| native tf2 | 같은 Python 3.10 import 및 static transform lookup PASS |
| 기존 ROS1 어댑터 unittest | 27/27 PASS, skip 0; Windows native skip도 실제 실행 |
| 기존 패키징 보조 unittest | 8/8 PASS |
| roslaunch --nodes | devel의 local.launch, install의 competition.launch 파싱 PASS |

두 공간에서 실행 래퍼 interpreter 15개 및 helper 경로, 리소스 22개 SHA256도 기존 검사로 확인했다.
증거는 [devel-environment.json](noetic-validation-2026-10-13/devel-environment.json),
[install-environment.json](noetic-validation-2026-10-13/install-environment.json),
[native-serialization.json](noetic-validation-2026-10-13/native-serialization.json),
[exit-codes.json](noetic-validation-2026-10-13/exit-codes.json)에 있다.

## 실제 MORAI UDP

Linux host network로 로컬 루프백을 공유한다. 모든 통신 IP는 `127.0.0.1`이다.
원본 YAML은 보존하고 [local.launch](../docker/local.launch)의 private parameter로 네트워크만 override한다.

| 채널 | MORAI Host → Destination Port | 실제 확인 |
| --- | --- | --- |
| Competition Vehicle Status | 9088 → 909 | 10초간 181바이트 288개, 원본 parse_status 모두 통과 |
| Ego Vehicle Status | 9100 → 9111 | 229바이트, 원본 Competition 파서가 거부; 사용하지 않음 |
| GPS | 1110 → 1111 | GGA/RMC NMEA checksum/좌표 원본 파서 통과 |
| IMU | 1113 → 1112 | 115바이트 framing/88바이트 payload/quaternion 원본 파서 통과 |
| VLP16 | 2369 → 2368 | 1206바이트와 12개 block marker 확인 |
| Camera-7 | 9090 → 9297 | MOR 조각 재조립, 실제 JPEG decode 및 자연색 RGB 영상 확인 |
| Ego Ctrl Cmd | 임시 송신 포트 → 9093 | 화면의 Destination 9082를 제어 수신 포트로 사용하지 않음 |

초기 Competition Host Port 908에는 MORAI 소켓이 없고 Destination 909에도 패킷이 없었다.
일반 사용자의 1024 미만 bind가 거부되는 호스트다. 사용자가 Host Port만 9088로 바꾸어 재연결한 뒤
181바이트 패킷이 들어왔다. 수신 Destination 909는 root 컨테이너의 NET_BIND_SERVICE로 bind했다.
초기 실패와 변경 후 결과는 각각 [udp-initial.json](noetic-validation-2026-10-13/udp-initial.json),
[udp-status-fixed.json](noetic-validation-2026-10-13/udp-status-fixed.json)에 있다.

MORAI가 사용자 화면에서 Pause인 상태에서도 실제 센서 UDP와 timestamp가 계속 들어왔다.
Pause 해제나 simulator control packet은 이 수신 검사에 필요하지 않았다. `/clock` 정지나 역행 검증을
수행했다는 의미는 아니다. Camera-7 포트에는 JPEG 이외 MOR 헤더가 아닌 패킷도 함께 들어왔고
원본 bridge와 같은 방식으로 제외했다. JPEG decode 성공과 별개로 이 비정상 패킷 수는 기록했다.

## 실제 drive=false 실행

실제 MORAI 입력으로 40초 동안 실행했다. 센서나 `/clock`을 주입하지 않았다. 실제 읽은
`/sensor_bridge/drive`는 false이며 지정한 루프백/포트 override가 적용됐다. loopback AF_PACKET
수동 캡처로 시작부터 SIGINT 종료까지 **UDP destination 9093 패킷 0개**를 확인했다.
양방향 차량 제어/Auto-D handshake 성공을 검사한 것은 아니다.

| 실제 구독 토픽 | 관측 개수 | 관측 빈도 |
| --- | ---: | ---: |
| /sensors/gps | 398 | 약 10.0Hz; 5Hz GGA/RMC 두 종류 |
| /sensors/imu | 1042 | 약 26.1Hz |
| /vehicle/status | 1147 | 약 28.7Hz |
| /sensors/lidar/points | 339 | 약 8.5Hz |
| /sensors/camera/traffic/compressed | 912 | 약 26.3Hz; 카메라는 5초 후 시작 |
| /localization/odometry | 1989 | 약 50.0Hz |
| /behavior/target | 786 | 약 20.0Hz |
| /control/request | 799 | 약 20.0Hz |

각 빈도는 관측 도구가 구독한 구간의 도착 빈도다. 센서 화면의 설정 빈도 달성이나 무손실을 증명하지 않는다.
bridge 로그의 GPS/IMU/Competition parse error는 0이었다. `/map/route`와 `/tf_static`은 늦게 구독해도
각 1개를 받았다. map←base_link 및 base_link←lidar lookup이 성공했고 후자는 원본 값
`2.0/0/2.11m`, identity quaternion을 유지했다. 실제 장착값과의 동등성은 아래 제한을 따른다.

관측 상태는 behavior **CRUISE**, v_max=16.6667m/s, 제어 요청 **HANDSHAKE**, accel=0, brake=1,
steering=0이었다. 차량은 Park 상태이며 송신이 꺼져 Auto/D 응답 대기 경고가 반복되는 것은 예상된 결과다.
현재 위치 s≈0.87m에서는 `/events`와 `/perception/traffic_signal` 출력이 0개였다. 장애물 이벤트·
신호 색 confirmation은 검증하지 않았다. YOLO 노드의 실제 CUDA provider 초기화는 로그로 확인했다.
자세한 결과는 [live.json](noetic-validation-2026-10-13/live.json),
[live-launch.txt](noetic-validation-2026-10-13/live-launch.txt)에 있다. 보고서에서 원래 NaN인 stop_s는
strict JSON을 위해 문자열 "NaN"으로 기록하며 런타임 값은 변경하지 않는다.

첫 관측 시도에서는 같은 프로세스의 TF listener와 AnyMsg 구독의 공유로 관측 callback에 AttributeError가
발생했다. 관측 도구를 실제 메시지 타입 구독으로 고친 뒤 전체 40초를 재실행한 결과만 위 표에 사용했다.
초기 관측 결과와 로그는 docker/artifacts에 보존했고 검증 완료 근거로 사용하지 않았다.

필수 `/sensor_bridge`를 `rosnode kill`로 종료하는 검사도 카메라 5초 지연 중과 카메라 시작 후 각각
수행했다. 두 경우 모두 launch 전체가 추가 SIGINT 없이 종료되고 제어 UDP는 0개였다. 처음 관측 도구가
닫힌 master를 조회하던 문제는 child의 자연 종료를 기다리도록 수정해 재검사했다. 다른 필수 노드
각각의 종료, 모든 launch 옵션/자식 프로세스 오류 조합을 검사했다는 의미는 아니다.

**깨끗한 종료는 미완료다.** 정상 SIGINT 종료 시 camera_bridge와 yolo_signal_node가
`ExternalShutdownException` traceback을 남겼다. 이들 main이 KeyboardInterrupt만 처리하는 경계 문제를
발견했고 원본 main이나 어댑터를 임의로 바꾸어 숨기지 않았다. root launch exit code는 0이며 프로세스는
종료됐지만 이 카메라 노드들의 종료 예외 처리는 추가 검토가 필요하다. 수신/판단 PASS와 구분한다.

## 기존 테스트와 원본 보존

원본 Windows manifest는 530개 파일을 고정한다. 현재 Linux checkout은 그중 497개가 LF/CRLF만 다르다.
나머지 33개는 그대로 일치한다. 원본 checkout의 기존 보존 검사는 실제로 실패했다:
[checkout-preservation.txt](noetic-validation-2026-10-13/checkout-preservation.txt).
원시 checkout의 preflight도 원본 경로 calibration 및 메시지 바이트 검사와 모델 README 해시에서 실패한다:
[checkout-preflight.txt](noetic-validation-2026-10-13/checkout-preflight.txt).

원본 작업 파일은 수정하지 않았다. 일회용 비교 사본에서만 LF를 기록된 CRLF로 복원하고 **기존 manifest의
파일 크기와 SHA256이 정확히 일치할 때만** 사용했다. 모든 530개를 기존 해시로 재구성할 수 있었다.
manifest 갱신이나 비교 허용 오차 확대는 하지 않았다. 복원 사본의 기존 보존 검사는 원본 파일 530개,
production Python 78개 바이트 동일/26개 허용된 경계 변환, 기존 테스트 파일 99개로 통과했다.

동일 CPython 3.10 환경과 공통 테스트 대역으로 원본/이식본의 기존 테스트를 각각 처음부터 실행했다.
**원본 909/909, 이식본 909/909 통과**, 각각 경고 10개가 있었다. 실패 case의 재실행은 필요하지 않았다.
trace 35,608개, 수치 3,657,429개, 최대 절대 차이 0, case outcome과 trace SHA256이 같다.
부동소수 허용 오차는 기존 `atol=rtol=1e-12`, 상태·정수·bool·timestamp는 정확 비교를 유지했다.
이 차등 검사는 실제 ROS2/ROS1 전송·스케줄링 비교가 아니다.
[differential.json](noetic-validation-2026-10-13/differential.json),
[preservation-restored-snapshot.json](noetic-validation-2026-10-13/preservation-restored-snapshot.json).

## 실제 GPU 추론과 모델 문서 문제

원본/이식본의 변경하지 않은 Detector로 GPU session 초기화와 입력 shape/class guard를 통과했다.
합성 영상 3개와 실제 현 Camera-7 RGB JPEG 1개에서 raw tensor, decode와 choose_signal 결과가 모두
정확히 같다. 최대 절대 차이는 0이다. 별도 ONNX profile에서 CUDAExecutionProvider node 실행 1,644건을
확인했고 CPU provider로의 fallback은 허용하지 않았다. 실제 이미지 tensor의 추론 5회 중앙값은
23.66ms였다. 현재 부하에서 측정한 작은 표본이며 전처리·ROS 지연을 포함한 실시간 보장은 아니다.
[gpu-inference.json](noetic-validation-2026-10-13/gpu-inference.json).

실제 모델 SHA256은 원본/이식본 모두 `d3c3939f51cc8cfa19d0eefc815c3bb9eaae6533e795f8ff87f347e9ed048b98`다.
원본 README의 `04ab171d8a46b5576891a23034000fa34cc63bf63336c8edb67b9a3fee551b8b`와 다르다.
이를 수정하거나 검사에서 삭제하지 않았다. 복원 사본에서도 기존 preflight는 이 문제로 실패한다:
[preflight.txt](noetic-validation-2026-10-13/preflight.txt).
Detector는 이 README 해시를 런타임 guard로 사용하지 않아 현재 실제 모델의 session/추론에는 영향을
주지 않았다. 모델 출처와 문서 불일치의 원인은 여전히 미해결이다.

## 실제 센서 조건의 차이와 보정 기준

| 센서 | 실제 MORAI Sensor Transform 화면 | 원본 코드/주행 기록 | 판정 |
| --- | --- | --- | --- |
| Camera-7 | 위치 1.90/0/1.20m, 회전 0/340/0°, 640×480, FOV 90°, 30Hz | Camera 7 RGB, 1280×720, pitch 335°; 주행 당시 위치/FOV 미상 | 입력 조건 불일치, 현 조건은 진단용 |
| VLP16 Lidar3D-6 | 위치 1.19/0.01/1.25m, 회전 0°, 10Hz, Intensity | base_link→lidar 위치 2.0/0/2.11m, 회전 0° | 기준점 변환 미확인 |
| GPS-4 | 위치 0.52/0/1.21m, 회전 0°, 5Hz | 후륜축 기준 gps_offset_x/y=0 | 기준점 및 GPS 출력 위치 의미 미확인 |
| IMU-5 | 위치 0.75/0.04/1.23m, 회전 0°, 50Hz | imu_yaw_offset_deg=0, UTM grid convergence 별도 계산 | 장착 yaw 화면은 같음; 좌표축 규약 미확인 |

MORAI 차량 원점에서 코드의 base_link/후륜축으로 변환하는 `R`, `t`를 확인해야
`p_base = R * p_morai + t`로 장착 위치를 비교할 수 있다. 화면 숫자를 그대로 TF/GPS offset에
대입하지 않았다. GPS 출력이 실제 안테나 위치를 나타내는지도 확인해야 한다. IMU quaternion의 축·
회전 방향을 원본 파서/UTM yaw 계산과 대조해야 하며 grid convergence를 중복 보정하면 안 된다.
이 단계 전에는 TF 그래프 통신 성공을 실제 extrinsic 동등성으로 판단하지 않는다.

Camera-7의 확정 주행 조건은 해상도 1280×720, pitch 335°, RGB(None Ground Truth)이며
네트워크 포트는 현 호스트용 9297을 사용할 수 있다. 원본 위치/FOV 프리셋은 아직 없다.
수집용 인수인계의 pitch 0°/위치 0.40/0/1.80m/FOV 70°를 주행용 설정과 섞지 않았다.
원본 ROI·모델·신호 confirmation 로직을 현 카메라에 맞추어 변경하지 않는다.

## 남은 범위

실제 원본 ROS2 Humble 설치 manifest, 같은 관측 순서·판단 시각의 ROS2/ROS1 replay,
TCPROS와 DDS의 손실·부하·재연결 차이, `/clock` 정지/전진/역행, 모든 launch 옵션 및 미션 전체
주행과 신호 정지선 판단은 이 테스트 결과로 통과했다고 판단하지 않는다.
실제 주행 시작 명령은 docker README에 준비하며 `drive:=true`는 이번 작업에서 실행하지 않는다.
