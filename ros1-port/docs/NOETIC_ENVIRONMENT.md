# Python 3.10 기반 Noetic 배포 환경

2026-10-22 실제 완주/현재 실패 범위는 [현재 상태](CURRENT_STATUS_2026-10-22.md),
새 수동 실행은 [ROS1 README](../README.md)를 따른다. 격리 build/install은
성공했지만 리소스 SHA/원본 비교 검사 실패는 남아 있다. 아래 배포 절차를
실행했다고 그 검사나 모든 미션이 통과한 것으로 판정하지 않는다.

Ubuntu MORAI PC용 격리 재현 구현은 [Docker 안내](../docker/README.md), 실제 실행 결과는
[Linux Docker 검증 기록](NOETIC_DOCKER_VALIDATION.md)에 있다. 호스트 Python/Noetic을 재사용하지 않는다.

사용자 선택은 **원본 의존성 유지 + CPython 3.10 기반 ROS1 Noetic**이다. 원본의 `numpy==1.24.4`, `onnxruntime-gpu==1.23.2`를 유지한다. 원본 requirements 파일과 모델은 변경하지 않는다. [ONNX Runtime GPU 1.23.2 공식 배포 정보](https://pypi.org/project/onnxruntime-gpu/1.23.2/)는 Python 3.10 이상을 요구한다.

Ubuntu 20.04의 기본 Noetic 바이너리는 보통 Python 3.8용이다. `PYTHON_EXECUTABLE`만 3.10으로 지정하면 이미 설치된 네이티브 확장 모듈의 ABI가 바뀌지 않는다. CPython 3.10에 맞춘 Noetic underlay와 네이티브 의존성을 먼저 준비해야 한다. 특히 `tf2_py`/`tf2_ros`, NumPy, SciPy, scikit-learn, OpenCV, pyproj를 실제 3.10 인터프리터에서 import해야 한다. `cv_bridge`는 현재 competition 코드에서 사용하지 않지만 설치·사용한다면 동일한 인터프리터에 맞춰 빌드해야 한다.

이 저장소는 Noetic 자체나 시스템 Python을 설치·변경하지 않는다. 기존 ROS2 underlay를 동시에 source하지 않는다. 아래 경로는 준비한 3.10 underlay의 실제 경로로 바꾼다.

```bash
source /path/to/noetic-python310/install/setup.bash
cd /path/to/RMT/ros1-port
PYTHON310=/absolute/path/to/python3.10
"$PYTHON310" --version
catkin_make -DPYTHON_EXECUTABLE="$PYTHON310"
source devel/setup.bash
export PYTHONPATH="$PWD/.camera-deps:${PYTHONPATH:-}"
"$PYTHON310" tools/check_noetic_environment.py
```

`.camera-deps`에는 원본 requirements에 맞는 **CPython 3.10용** NumPy/ONNX Runtime이 있어야 한다. 다른 Python 버전에서 만든 디렉터리를 재사용하지 않는다. 필요한 설치는 준비 단계에서 별도로 수행하며, 검사기는 설치하거나 버전을 대체하지 않는다. CUDA 12/cuDNN 9 요구는 원본 requirements 주석을 보존한다. 실제 CUDA 라이브러리 로딩·GPU inference·프레임 처리 시간은 별도 검증 대상이다.

`run_competition.sh`는 원본처럼 `/opt/ros/...`를 source하는 경로를 유지하며 배포본은 `/opt/ros/noetic/setup.bash`를 사용한다. 커스텀 underlay가 다른 위치에 있으면 준비한 환경을 source한 뒤 직접 실행한다.

```bash
source /path/to/noetic-python310/install/setup.bash
source /path/to/RMT/ros1-port/devel/setup.bash
export PYTHONPATH="/path/to/RMT/ros1-port/.camera-deps:${PYTHONPATH:-}"
roslaunch foscar_bringup competition.launch drive:=false
```

원본 기본 `drive:=true`는 그대로다. 위의 `drive:=false`는 ROS 환경 확인용 명시 인자다. 검사기 자체는 `rospy.init_node`, Publisher/Subscriber 생성, 노드 생성자, UDP 소켓, 시뮬레이터 명령을 실행하지 않는다.

검사기는 다음을 실제 배포 인터프리터로 확인한다.

- Linux/CPython 3.10, `ROS_VERSION=1`, `ROS_DISTRO=noetic`, Python 3 설정과 Noetic 도구 경로.
- 정확한 NumPy/ONNX Runtime 버전과 import 경로, GPU 배포의 CUDA provider 등록 여부.
- 수치·영상·좌표 변환 네이티브 라이브러리 import 및 작은 계산.
- rospy, catkin, tf2와 실제 생성된 ROS1 메시지 import.
- competition과 선택 실행 노드 15개 모듈, 실행 래퍼와 선택 Python의 일치, RViz/static TF/bringup helper 경로.
- 원본 리소스·메시지·requirements 22개 파일의 크기 및 SHA256.
- 메시지 필드/타입/상수, 실제 genpy 직렬화 왕복, Marker quaternion 기본값 어댑터.
- 소스 워크스페이스가 있으면 package.xml 파싱과 catkin 위상 순서.

```bash
"$PYTHON310" tools/check_noetic_environment.py --json
python3 tools/check_packaging.py
```

검사 실패는 종료 코드 1로 반환한다. 원본 모델 README와 실제 파일 해시가 다른 문제는 `KNOWN_SOURCE_ISSUE`로 따로 출력한다. 실제 원본/이식본 모델 SHA256은 `d3c3939f51cc8cfa19d0eefc815c3bb9eaae6533e795f8ff87f347e9ed048b98`이며, 보존된 원본 README의 값은 `04ab171d8a46b5576891a23034000fa34cc63bf63336c8edb67b9a3fee551b8b`다. 검사기는 이 차이를 숨기거나 README·모델·기존 해시 검사를 바꾸지 않는다. `tools/resource_hashes.json`은 원본에서 확인한 파일 바이트를 고정한 별도 이식 무결성 기준이다.

이 검사를 통과해도 CUDA session 생성, ONNX 추론의 수치·시간 동등성, ROS master 통신, callback 순서, 시뮬레이션 시간, 실제 주행을 통과한 것은 아니다. 사용자가 선택한 TCPROS 후보는 지연·손실·순서 변동을 포함한 ROS2/ROS1 재생 비교 후에 판단한다. 현재 Windows 검증은 Noetic 배포 검증과 구분한다.

`record.launch.py`와 camera_train 도구는 원본 비교용으로 보존되어 있으며 competition이 실행하지 않는다. offline 수집 launch의 ROS1 XML 변환과 message_filters 경계는 이번 competition 실행 검증 범위에 포함되지 않는다.
