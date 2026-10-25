# 대상 ROS1 환경에서 남은 검증

아래는 통합 검증 체크리스트다. Ubuntu MORAI PC에서 실제 수행한 범위와 미검증 범위는
[Linux Docker 검증 기록](NOETIC_DOCKER_VALIDATION.md)에 별도로 기록한다. Windows 비교 통과와
별개로 각 항목의 결과가 필요하다. 기준은 원본 `2026_CTRL_competition`이며 다른 폴더나 SCMC를 사용하지 않는다.

## 환경과 패키지

Python 3.10으로 일관되게 빌드한 Noetic에서 `tools/check_noetic_environment.py`를 실행한다. Python, ROS 배포, NumPy 1.24.4, ONNX Runtime GPU 1.23.2, 모든 native extension 및 CUDA/cuDNN provider를 확인하고 실제 패키지 버전을 기록한다. 원본 ROS2 Humble의 전체 설치 manifest도 확보한다.

devel과 install 공간 각각 `catkin_make`, `roslaunch --nodes`로 `competition.launch`와 포함 launch를 검사한다. 원본의 모든 인자·기본값을 기준으로 camera/lidar/가짜 cloud/RViz/view/bag/points 조합을 확인한다. calibration 경로 해시, 모델 shape/class guard와 누락 리소스 오류가 실제로 실패를 보고하는지도 확인한다. 원본 오류를 포트에서 자동 보정하지 않는다.

## 메시지·TF·종료

실제 생성된 메시지로 adapter unittest를 실행한다. PointCloud2 little/big endian, field count/offset/order, NaN, intensity, organized cloud와 ROS1 byte serialization을 확인한다. Marker 기본 quaternion과 native roundtrip의 Windows skip도 여기서 실행한다.

TF update와 cloud callback 순서를 trace로 기록한다. `base_link`, `lidar`, `map` 좌표와 후륜축 기준을 비교한다. `/map/route`, `/localization/mode`의 늦은 구독자 latch, 재연결 및 큐 깊이를 확인한다.

필수 base node 각각 종료 시 전체 launch가 종료되는지, 5초 camera 지연 도중/이후 종료 시 자식 launch도 종료되는지 확인한다. Bridge의 watchdog, 50Hz UDP 송신, 3초 handoff, 상태 무수신·중복 timestamp·SIGINT 중복·초기 bind 오류를 원본과 같은 입력으로 확인한다. 차를 제어하는 실제 주행 전에는 `drive:=false`로 메시지 경로와 종료를 검증한다.

## 동일 입력·순서·판단 시각 replay

각 node의 수신 직후 timestamp와 순서, executor callback의 시작·끝·group, tick의 ROS time 및 독립 monotonic time을 기록한다. 단순한 bag 파일의 재생 순서만으로 이 조건을 만족했다고 가정하지 않는다. 양쪽에서 동일 trace를 적용하고 다음을 비교한다:

| 영역 | 비교 내용 |
| --- | --- |
| 미션 합성 | 활성 상태 전체, 우선순위, 동시 제약, reset·timeout 경계 |
| 신호 | 교차로별 색 기억, UNKNOWN·낡은 관측·전환, 통행/정지, stop_s |
| 장애물·보행자 | 진입/복귀·회피 chain, 속도 상한, YIELD 진입/해제·소유권 |
| 회전교차로 | 관측·ring gap·정지/입장·following·reference 전환 |
| 고속도로 | mission 경계·lane slot·reference 좌표/ID·속도 상한·복귀 |
| 터널 | GPS loss, corridor/slalom/dynamic 제약, 정지·재출발·GPS 복귀 |
| 위치·제어 | GPS/DR/blend, steering·accel·brake·속도 profile, watchdog·FINISH |

상태·ID·배열 길이·순서는 정확히 일치해야 한다. 같은 interpreter/수치 라이브러리에서의 계산 비교는 `atol=rtol=1e-12`, timestamp는 정수 nanoseconds 정확 일치를 기준으로 시작한다. reference hash를 수치 허용 오차로 무시하지 않는다. 다른 하드웨어나 GPU에서 허용 오차를 확대해야 한다면 차이의 원인·최댓값·판단 경계 영향과 승인 근거를 먼저 기록한다.

## ROS time과 TCPROS 부하

`/clock` 정지·전진·역행, 입력 없이 tick만 진행, 관측 timestamp만 오래된 경우, monotonic watchdog이 계속 진행하는 경우를 분리한다. clock을 바꾸어 정지 조건을 억지로 통과시키지 않는다.

동일 녹화 입력으로 cloud/image burst, callback 처리 지연, reconnect, 느린 구독자, 큐 overflow와 센서 패킷 손실을 재현한다. 실제 callback trace·최대 관측 나이·누락 메시지·타겟 timeout·정지 결과를 원본 DDS와 ROS1 TCPROS에서 비교한다. 차이가 판단을 바꾸면 TCPROS 후보를 동등하다고 채택하지 말고 구체적인 trace와 대안을 사용자에게 제시한다.

마지막으로 실제 카메라 녹화 입력과 원본 provider 선택으로 GPU 추론·ROI·색 confirmation을 비교하고, 같은 미션 시나리오의 전체 주행 결과를 기록한다. 현재 합성 영상 CPU 검사나 catkin 빌드는 이 검증을 대신하지 않는다.
