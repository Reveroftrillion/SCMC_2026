# CTRL 팀 신호 관측·정지선 연결 안내

파일명의 날짜는 이전 문서 경로를 유지하기 위한 것이다. 현재 안내는 CTRL ROS1 실행의 책임과 확인 순서를 설명하며, 이전 2026년 실험 원문은 아래 보관 링크에 있다.

## 처리 흐름

1. Camera-7 compressed image를 수신한다.
2. 미션이 카메라 관측 구간을 활성화하면 `ctrl_yolo_signal_node`가 추론한다.
3. 검출 결과를 신호 관측으로 발행한다.
4. planning의 `ctrl_signal_stop`이 다음 정지선과 허용 색을 판단한다.
5. 제어기는 속도 상한과 앞바퀴 기준 정지 station을 적용한다.

현재 planning은 채택한 색을 단순한 시간 만료로 해제하지 않는 기억 정책을 가진다. 신호를 아직 채택하지 못한 경우와 채택한 정지 색이 유지되는 경우를 혼동하지 않는다. 이 정책의 변경은 별도 미션 재구현 작업이다.

## 설정과 검증

[정지선/ROI 설정](config/yolo_stoplines.json), [ROS1 카메라 launch](launch/ctrl_yolo_signal.launch), [신호 판단 코드](../ctrl_planning/ctrl_planning/ctrl_signal_stop.py)를 함께 확인한다. 경로가 바뀌면 보정의 route hash와 station도 확인한다.

점검 순서는 영상 수신 → 보정 구간 활성화 → 검출 색/시각 → planning 신호 상태 → 정지 목표다. 영상 수신 주파수와 추론 완료 주파수는 서로 다르며 과거 측정값을 새 PC의 처리 시간으로 사용하지 않는다.

이 문서 정리에서 모델·ROI·정지선·신호 기억·속도 설정은 바꾸지 않았다. 사용 모델의 출처와 보정 데이터 생성 근거는 [출처 안내](../../docs/SOURCES_AND_HISTORY.md)에 따라 따로 기록한다.

## 이전 문서 원문

[교체 전 원문과 당시 기록](../../../%EA%B3%BC%EA%B1%B0%EC%9E%90%EB%A3%8C/ctrl-document-history-20261025/ros1-port/src/ctrl_camera/YOLO_STOPLINE_2026-09-28.md). 원래 작성자·날짜·출처와 실험 수치를 보존했다.
