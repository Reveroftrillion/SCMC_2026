# 원본 문제와 이식 검증의 한계

2026-10-13, 기준 `2026_CTRL_competition`, 커밋 `a1ea82efbf8d5f165201b7f51bf8a141439fb8c7`. 아래 항목을 이유로 원본 알고리즘·튜닝·조건을 수정하지 않았다.

## 원본 모델 문서의 해시 불일치

원본 `src/foscar_camera/models/README.md`는 `best.onnx` SHA256을 `04ab171d8a46b5576891a23034000fa34cc63bf63336c8edb67b9a3fee551b8b`로 기록한다. 현재 추적된 실제 파일은 38,669,688 bytes, SHA256 `d3c3939f51cc8cfa19d0eefc815c3bb9eaae6533e795f8ff87f347e9ed048b98`이다. 원본은 Git에서 수정 상태가 아니며 LFS pointer도 아니다. 불일치가 생긴 시점이나 이유는 확인되지 않았다.

이식본은 **현재 원본과 바이트 단위로 동일한 파일**을 사용한다. 이전 문서에 맞추어 모델을 교체하지 않았다. 새 `tools/preflight.py`가 이 불일치를 오류로 보고한다. 원본의 traffic route 보정 해시와 Detector의 입력 shape·클래스 매핑 검사는 유지했으며 현재 파일은 이 검사들을 통과했다.

실제 모델을 원본/이식본 Detector로 실행한 Windows CPU 비교는 `docs/model-comparison.json`에 있다. 고정된 합성 영상 3개에서 raw tensor·decode·신호 선택의 차이는 0이었다. 합성 영상에서는 검출 결과가 비어 있었으므로 실제 신호등 영상에서 색·ROI 판정과 GPU 추론의 검증을 대신하지 않는다.

## 원본 경로 해시와 Git 줄바꿈

이 PC의 `core.autocrlf=true` 상태에서 원본 `global_path.txt`는 CRLF 4,429개를 포함하며 SHA256은 보정 파일의 값 `a7fa4a89ff3840c7cc1be9b3b971572a67eb514f3ebee5a4a00ef535b73c64c3`와 일치한다. 하지만 같은 커밋의 Git blob은 LF이며 SHA256 `b1763844feea95368a21d1ebae1c756c442add2ebeb425fdc6ec85d9b53c1641`이다. 따라서 원본을 Linux에서 평범하게 새 checkout하면 보정 해시 검사가 실패할 수 있다. 현재 Windows 원본은 Git상 clean이다.

이식본은 현재 비교 기준인 원본 작업 파일의 바이트를 그대로 복사했다. `ros1-port/.gitattributes`의 `* -text`는 이식본 파일의 Git 자동 줄바꿈 변환을 막는다. 경로 숫자·보정 해시를 수정하거나 guard를 우회하지 않는다. `source-manifest.json`은 Git의 정규화된 blob이 아닌 현재 보존한 원본 작업 파일의 바이트를 기준으로 한다. 다른 PC에서 원본 비교 기준을 재구성할 때도 이 차이를 확인해야 하며 manifest를 무작정 재기록하지 않는다.

## 원본 의존성과 Noetic 기본 Python의 차이

원본 `requirements-onnx.txt`는 NumPy 1.24.4와 ONNX Runtime GPU 1.23.2를 고정한다. [공식 ORT 배포](https://pypi.org/project/onnxruntime-gpu/1.23.2/)는 Python >=3.10을 요구한다. 기본 Noetic Python 3.8 환경에 맞춘 다운그레이드는 하지 않았다. 사용자는 Python 3.10 기반 Noetic을 선택했다. Linux Python 3.10 underlay와 native extension은 [Ubuntu Docker 검증](NOETIC_DOCKER_VALIDATION.md)에서 호스트와 분리해 소스 빌드·실행했다.

## 전송·버퍼·시각·동시성

사용자는 TCPROS 후보를 유지하고 지연·손실 재현 검증 후 판단하기로 했다. 원본 DDS BEST_EFFORT와 TCPROS는 전달·손실·head-of-line blocking 특성이 다르다. 애플리케이션 큐 깊이와 callback group은 보존했지만 TCP·커널·rospy 수신 버퍼와 publisher 큐까지 같은 시스템은 아니다. 기본 rospy 수신 버퍼와 큰 cloud/image 메시지의 과부하 동작은 실제 환경에서 측정해야 한다.

서로 다른 토픽의 물리적 도착 순서와 worker 스케줄은 양쪽 모두 비결정적이다. 동일 callback trace와 판단 시각의 계산 비교 및 실제 transport/clock 검증을 분리한다. ROS1 전역 `/use_sim_time`과 `/clock` 전환·역행·정지를 실제 Noetic에서 추가 확인해야 한다.

## 메시지 스키마와 기본값

Humble Quaternion.w의 기본값 1을 Noetic의 기본값 0으로 바꾸지 않도록 Marker 생성 어댑터를 넣었다. 원본 출력이 기본 pose를 쓰는 DELETEALL 메시지도 1을 유지한다. Windows에서 skip이던 실제 Noetic 직렬화 roundtrip은 [Ubuntu Docker 검증](NOETIC_DOCKER_VALIDATION.md)에서 실행해 통과했다.

ROS1 Header에는 `seq`가 추가된다. ROS2 Marker의 texture/mesh 관련 부가 필드는 Noetic 정의에 없다. 현재 원본은 이 필드를 사용하지 않는다. 원본이 사용하는 모든 필드의 의미·단위·값과 상수는 유지한다. 프로토콜의 wire 바이트가 ROS1과 ROS2에서 같다고 주장하지 않는다.

## Ubuntu 실제 실행에서 확인한 추가 제한

현재 Linux 원본 checkout은 Windows manifest와 497개 파일의 줄바꿈이 달라 기존 보존/패키징 검사가 실패한다.
원본 파일이나 manifest를 고치지 않고, 일회용 사본에서 기존 해시에 정확히 맞는 CRLF 복원만 적용해
비교 검증했다. 원시 checkout의 실패도 결과에 남겼다.

MORAI Camera-7의 해상도/pitch가 원본 주행 조건과 다르고, 차량 원점에서 base_link/후륜축으로의 변환도
확인되지 않았다. 현재 LiDAR/GPS/IMU 장착 화면값을 원본 TF/offset에 그대로 대입하지 않았다.
센서 수신과 TF lookup 통과는 실제 입력/장착 동등성 통과가 아니다.

`drive:=false`의 실제 실행과 제어 UDP 0건은 확인했지만, 종료 시 camera_bridge와 yolo_signal_node가
`ExternalShutdownException` traceback을 남긴다. 원본 camera main은 KeyboardInterrupt만 처리하며,
이식 어댑터의 shutdown 예외와 경계 처리가 추가 검토 대상이다. launch는 종료되고 남은 실행 중 노드는
없었으나, 이 카메라 노드들의 깨끗한 종료까지 검증 완료로 보고하지 않는다. 알고리즘이나 원본 main을
바꾸어 숨기지 않았다. 상세 로그와 수행 범위는 [Docker 검증 기록](NOETIC_DOCKER_VALIDATION.md)에 있다.

## 원본 테스트 및 캐시의 부작용

`HighwayPolicy._save_kappa`와 LaneMap은 조건에 따라 패키지 내부의 곡률·seam 캐시를 쓴다. 이 원본 동작은 고치지 않았다. 검증은 전체 원본 해시를 기록하고 **일회용 복사본**에서 실행하여 원본 수정과 bytecode/cache 생성을 방지한다. 대상 install 공간이 쓰기 금지인 경우 원본의 캐시 write 실패 처리까지 확인해야 한다.

원본 `foscar_control/test/integration_udp.py`는 다른 `2026foscar/` 경로를 하드코딩한다. 이 파일은 이식본에서도 비교용 원본 그대로 두었으며 실행하지 않았다. 다른 RMT 폴더를 비교 기준으로 사용하는 통합 검증을 하지 않는다. 대회 미사용 `camera_train`의 offline launch 및 일부 외부 서비스 테스트는 별도 범위다. 기존 테스트의 실패/skip/미수집은 검증 결과 문서에 개별 기록하며 원본 코드를 고쳐 통과시키지 않는다.

## 원본 환경 버전 미상

원본이 실행되던 ROS2 Humble 시스템의 `rclpy`, `sensor_msgs_py`, NumPy/SciPy/OpenCV/GPU 드라이버 등 전체 설치 manifest는 없다. PointCloud2 어댑터는 공식 Humble commit에 고정했고 해당 코드와 직접 비교했다. 실제 원본 설치 버전까지 동일하다는 증거는 아니므로 원본 실행 환경의 패키지 manifest와 replay 결과가 추가로 필요하다.
