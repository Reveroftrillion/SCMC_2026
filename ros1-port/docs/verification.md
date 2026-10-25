# 동작 보존 검증

이 문서는 Windows에서 수행한 정적 검사와 Python 차등 검증을 설명한다. 실제 ROS1/ROS2 discovery, TCPROS/DDS 전달, ROS master, rosbag, catkin 빌드, 시뮬레이터 주행을 실행한 결과는 아니다. ROS 환경의 필수 검증은 아래에 별도로 남긴다.

## 비교 기준과 원본 보호

`verification/source-manifest.json`은 작업 시작 시 원본 `2026_CTRL_competition`의 파일 530개 전체에 대해 크기와 SHA256을 기록한다. 원본의 Git 로컬 변경 및 삭제가 없음을 확인한 후 생성했다. 모델 파일도 실제 저장된 바이트를 기록하며 문서에 기재된 다른 모델 해시로 교체하지 않는다.

`verification/check_preservation.py`는 현재 원본을 이 manifest와 대조하고, 이식한 Python 파일 104개의 내용을 검사한다. 78개는 원본과 바이트가 같고 26개는 `tools/port_boundary.py`에 명시한 ROS API/메시지/리소스 경계 변환만 적용된다. 기존 테스트와 fixture 파일 99개도 원본과 바이트가 같다. 이는 알고리즘과 상수 변경을 발견하기 위한 검사이며 ROS 실행 동작까지 증명하지는 않는다.

차등 테스트는 `verification/.work/original` 및 `.work/port`에 복사한 disposable 소스에서 실행한다. 원본에서 Python 모듈을 실행하거나 원본 아래에 pytest/bytecode/cache를 생성하지 않는다. 특히 원본 `HighwayPolicy`의 곡률 cache 갱신과 지도 seam cache 생성도 복사본에서만 발생한다. `PYTHONDONTWRITEBYTECODE=1`과 `-p no:cacheprovider`도 적용한다.

## 기존 테스트 재사용과 차등 비교

`verification/run_differential.py`는 원본의 `foscar_control`, `foscar_planning`, `foscar_lidar`, `foscar_camera` 테스트 전체를 수정 없이 양쪽에서 실행한다. source/port 프로세스는 독립적이며 동일한 테스트 순서, hash seed, fixture 입력을 사용한다. 원본 테스트에서 쓰는 decision time/observation stamp와 순수 Python 클래스의 입력을 기록한다. ROS2 형태의 기존 fixture timestamp는 테스트 객체의 `sec/nanosec`와 `secs/nsecs` alias를 통해 같은 값으로 표현한다.

`verification/ros_stubs.py`는 테스트용 메시지 및 Node 객체만 제공한다. publisher는 결과를 저장하고 timer/subscription은 생성만 하며 자동 실행하지 않는다. 원본 callback 몸체와 계산 코드의 결과를 비교하기 위한 공통 경계다. 실제 `foscar_ros1` executor/queue/timer는 이 차등 suite에서 사용하지 않고 별도 adapter suite에서 검사한다. 따라서 이 결과만으로 실제 callback 도착 순서나 지연이 같다고 주장할 수 없다.

| 대상 | 비교하거나 확인하는 내용 |
| --- | --- |
| `Behavior.tick` | 활성 상태 집합, 우선순위 합성, state, 속도 상한, 정지점, offset, chain, reference |
| `SignalStopPolicy.observe/apply` | 교차로 key, 색 기억, accepted stamp, 통행/정지 및 reset |
| `Avoid.compute_target`, `Yield.compute_target` | 장애물 경로/chain, 정지점과 상한, 보행자 해제 시점 |
| `CircularGap.evaluate` 및 기존 회전교차로 tests | gap, 위험 경계, YIELD 소유권, 진입/추종/복귀 판단 |
| `HighwayRuntime.apply` 및 고속도로 tests | shadow/control 모드, reference의 global station/xy, scan freshness, rejoin/mission handoff |
| 터널 tests | 지도/회랑/슬라럼/보행자/동적 장애물, 경로 chain과 속도 제한 |
| `Controller.step` | state/reason, accel, brake, steering, target speed/xy, dropout/종료 |
| `PoseEstimator.update` | GPS/DR 모드, xy, GPS 복귀/보정, blackout 거리 |
| Marker 생성 함수 | 메시지 필드, geometry, 기본 quaternion 값. 원본 w=1/ROS1 native w=0을 구분하며 실제 Marker adapter를 로드 |
| 기존 LiDAR/camera tests | ROI/ground/track/footprint/visibility, 신호색 선택과 decode |

실수 비교 허용 오차는 `abs(a-b) <= max(1e-12, 1e-12*max(abs(a),abs(b)))`이다. 정수, bool, state 문자열, 배열 길이/순서, `NaN/+Inf/-Inf` 구분은 정확히 비교한다. 각 테스트 자체의 기존 assertion과 허용 오차는 그대로 유지한다. 기록한 입력과 출력의 호출 순서도 비교하며, 한쪽 실패를 삭제하거나 xfail 처리하여 성공으로 바꾸지 않는다.

검증 결과와 실행한 dependency 버전은 `verification/differential-results.json`, case별 성공/실패 목록은 `verification/case-outcomes.json`, 원본 보존 검사는 `verification/preservation-results.json`에 저장한다. 상세 pytest log와 전체 입력/출력 trace는 재실행 시 `.work/`에 저장된다. 사용한 Python 3.10 및 Windows wheel은 Ubuntu에 설치된 rospy/catkin/드라이버 호환성 증거가 아니다.

실행 결과는 원본/이식본 각각 909개 통과다. 순서가 같은 입력/출력 trace 35,608개에서 수치 3,657,429개를 비교했고 최대 절대 오차는 0이었다. canonical trace SHA256도 양쪽 모두 `1c88a25cdec77b28a5a99ac8ce585660009a34b57d8b1fa91f57e15699e4c36a`로 같다. 구성은 Python 3.10.11, NumPy 1.24.4, SciPy 1.10.1, pytest 8.4.2, PyYAML 6.0.3, pyproj 3.6.1, scikit-learn 1.3.2, matplotlib 3.7.5, OpenCV 4.8.1.78이다.

최초 전체 실행은 양쪽 908개 통과와 1개 실패였다. 실패 원인은 테스트 double의 표준 `Marker.TRIANGLE_LIST=11` 상수 누락이었다. 이 검증 도구만 보완하고 해당 시각화 case를 양쪽에서 다시 실행해 통과했다. 기존 908개 결과와 이 재실행을 합산한 결과이며, 최초 traceback와 선택 재실행 provenance도 JSON에 보존한다. 원본/이식 production 코드를 수정하여 이 실패를 해결하지 않았다. 차등 runner는 비교 불일치이면 exit 1, 양쪽 동일한 기존 테스트 실패가 남아도 exit 2를 반환한다.

camera train 보조 도구의 ROS2 launch 확장 테스트와 POSIX `pty`/signal/subprocess 테스트는 이 competition runtime suite에 포함하지 않는다. 원본 테스트 파일은 보존한다. 원본 launch graph의 인자/default/노드/파라미터와 ROS1 XML의 정적 일치는 `tools/check_packaging.py`가 별도로 확인한다.

## 독립적인 ROS1 경계 검사

`src/foscar_ros1/test`의 unittest는 실제 production adapter를 불러오고 rospy transport/time만 fake로 제공한다. default callback group의 직렬 실행, 분리된 group의 동시성, entity 예약, 생성 순서와 timer 우선 snapshot, subscription depth와 FIFO/overflow, missed timer/clock rewind, worker 예외 전파/종료, latched publication, parameter YAML/override/type을 검사한다.

PointCloud2 테스트는 고정된 공식 Humble serializer 원본 fixture와 NumPy 변환 결과를 비교하며, `uint8[]`/big-endian/structured dtype/NaN/field 순서의 ROS1 경계를 확인한다. Marker 테스트는 Humble quaternion 기본값 복원과 명시된 pose 보존을 확인한다. 실제 생성된 Noetic message serializer를 요구하는 경우 ROS 미설치 Windows에서는 skip한다. skip은 실제 wire 검증 성공으로 계산하지 않는다.

Python 3.10.11 / NumPy 1.24.4에서 직접 실행한 결과는 27개 중 26개 통과, 1개 skip이다. skip은 `test_full_native_serialization_and_explicit_pose`이며 실제 Noetic 생성 메시지를 요구한다.

`tools/check_packaging.py`도 직접 실행하여 launch 인자 110개, 노드 선언 21개, 타입을 보존한 parameter 106개, 원본 bag topic 24개와 추가 선택 topic 2개, resource/message 파일 21개의 정확한 복사 검사를 통과했다. 보조 runner/resource/의존성 검사에 대한 `tools/test_packaging.py` 8개 테스트도 통과했다. 실제 catkin 빌드/roslaunch 실행 성공을 의미하지 않는다.

## 재실행

검증 dependencies는 production camera pins와 분리한 `verification/requirements.txt`에 있다. 이미 준비된 Windows Python 3.10에서:

```powershell
ros1-port/.python310/python.exe ros1-port/verification/check_preservation.py
ros1-port/.python310/python.exe ros1-port/verification/run_differential.py
ros1-port/.python310/python.exe -m unittest discover -s ros1-port/src/foscar_ros1/test -v
ros1-port/.python310/python.exe ros1-port/tools/check_packaging.py
```

일반 Python 3.10 가상환경에서도 같은 script를 실행할 수 있다. 차등 실행 스크립트는 Windows embeddable Python의 isolated `._pth`를 바꾸지 않고 child 프로세스의 테스트용 import 경로를 명시한다. `--record-baseline`은 manifest를 최초 생성할 때만 사용하고 원본 변화에 맞추어 기존 manifest를 덮어쓰지 않는다.

## ROS 환경에서 남은 검증

1. 사용자 선택 Python 3.10로 Noetic/rospy/생성 메시지/cv_bridge/catkin을 구성하고 source/devel/install space에서 전체 빌드와 실행을 확인한다. 모델/지도/경로 hash 검사를 먼저 통과해야 한다.
2. 같은 입력 packet, stamp, callback admission 순서와 decision clock을 기록해 Humble/Noetic의 실제 노드 그래프에서 위 비교 항목을 다시 검사한다. 대기열 초과, 같은 시각의 timer/subscription 준비, 두 callback group 동시 실행, 시계 pause/rewind/jump도 포함한다.
3. TCPROS와 원본 DDS QoS의 차이 때문에, 지연·손실·재연결 시 각 input freshness/timeout/초기값/reset의 동작이 달라지는지 확인한다. 실제 ROS 환경에서 검증하기 전 두 middleware의 동등성을 결론내리지 않는다.
4. rosbag 기록/재생, camera 5초 지연, RViz, TF `/tf`/`/tf_static`, watchdog, Ctrl-C/required-node 종료와 차량 handoff를 확인한다.
5. 모든 미션 구간과 경계, 신호 조합, 장애물 chain/보행자, 회전교차로, 고속도로 rejoin, 터널 GPS 음영 및 복귀를 시뮬레이터와 실제 주행으로 검증한다.
