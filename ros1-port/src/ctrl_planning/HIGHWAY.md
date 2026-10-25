# CTRL 팀 고속 구간 판단 안내

현재 고속 구간은 기존 ROS1 이식·수정 구현을 사용한다. CTRL 팀의 독자 교체는 제어/인지/일반 회피 기반을 검증한 뒤 진행하는 9단계다.

## 현재 역할

`ctrl_highway_runtime`이 관측·정책·차로 경로·움직임 계획을 연결한다. `ctrl_highway_policy`는 행동 판단, `ctrl_highway_slot_planner`는 진입 가능한 공간/속도 후보, `ctrl_highway_maneuver`는 변경 경로 연결을 담당한다. 실제 인자는 [highway.yaml](config/highway.yaml)과 미션 설정을 기준으로 확인한다.

## 교체 시 보존할 의미

차로 선택, 앞차 추종, 뒤차 접근, 차로 변경 시작/진행/복귀를 구분한다. 위치·속도·전후방 간격의 시간축을 맞추고 실행 중 더 이상 유효하지 않은 계획의 감속/중단 조건을 정한다. 경로의 local 길이와 전역 station을 같은 값으로 가정하지 않는다.

검사는 빈 도로, 느린 앞차, 빠른 뒤차, 변경 중 앞차 감속/관측 상실, 곡선과 복귀를 나눠 수행한다. 예전 최고속도·통과 시간은 원래 장면의 기록이며 새 코드나 새 배치의 성능으로 쓰지 않는다.

현재 새 경로 변환은 [변환 결과](../../docs/PATH_CONVERSION_RESULT_2026-10-25.md), 실제 교체 순서는 [팀 계획](../../docs/INDEPENDENT_REBUILD_PLAN_2026-10-25.md)을 따른다. 이번 문서 정리에서는 속도·차로 정책을 변경하지 않았다.

## 이전 문서 원문

[교체 전 원문과 당시 기록](../../../%EA%B3%BC%EA%B1%B0%EC%9E%90%EB%A3%8C/ctrl-document-history-20261025/ros1-port/src/ctrl_planning/HIGHWAY.md). 원래 작성자·날짜·출처와 실험 수치를 보존했다.
