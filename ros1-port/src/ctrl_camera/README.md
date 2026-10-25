# CTRL 팀 카메라 신호 인지 — ctrl_camera

현재 실행은 Camera-7 영상에서 ONNX 검출 결과를 받아 신호 관측을 발행한다. 정지·출발 판단은 planning 계층이 담당한다. 카메라 패키지가 직접 조향이나 정지 목표를 발행하지 않는다.

| 구성 | 역할 |
| --- | --- |
| `ctrl_camera_bridge` | UDP 영상 조각 수신과 compressed image 발행 |
| `ctrl_yolo_signal_node` | 활성 신호 구간의 추론·관측 전달 |
| `ctrl_yolo_onnx` | 모델 입력·추론·후처리 |
| `ctrl_camera_view`, `ctrl_gps_logger` | 영상 확인과 보정용 기록 |

주요 연결은 `/sensors/camera/traffic/compressed` → `/perception/traffic_signal`이다. 현재 호스트의 카메라 수신 포트는 대회 실행을 감싼 [local.launch](../../docker/local.launch)의 9297이다. 패키지 단독 launch의 기본값과 같은 값이라고 가정하지 않는다.

## 실행과 보정

[ROS1 실행 안내](../../README.md)의 대회 구성으로 실행한다. ROS1 개별 카메라 구성은 [ctrl_yolo_signal.launch](launch/ctrl_yolo_signal.launch)에 있다. 예전 ROS2 `colcon` 명령이나 타인 PC의 주소를 현재 설정으로 사용하지 않는다.

정지선·구간·ROI의 연결은 [신호 인지 안내](YOLO_STOPLINE_2026-09-28.md)를 따른다. 카메라 장착 위치·해상도·각도가 바뀌면 기존 보정의 유효성을 먼저 확인한다.

## 모델과 결과

[모델 원래 메타데이터](models/README.md)는 보존한다. `best.onnx`의 hash와 '사용자 제공' 기록은 CTRL 팀 자체 학습의 증명이 아니다. 학습 데이터·가중치 출처는 [자산 목록](../../docs/independent_assets_2026-10-25.json)의 열린 항목이다.

이 패키지는 아직 새 인지 구현으로 교체하지 않았다. 과거 완주/추론 속도 기록을 현재 PC나 재구현의 성능으로 다시 기재하지 않는다. 개발 순서는 [팀 계획](../../docs/INDEPENDENT_REBUILD_PLAN_2026-10-25.md)을 따른다.

## 이전 문서 원문

[교체 전 원문과 당시 기록](../../../%EA%B3%BC%EA%B1%B0%EC%9E%90%EB%A3%8C/ctrl-document-history-20261025/ros1-port/src/ctrl_camera/README.md). 원래 작성자·날짜·출처와 실험 수치를 보존했다.
