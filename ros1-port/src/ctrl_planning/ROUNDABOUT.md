# CTRL 팀 로터리 판단 안내

현재 로터리 실행은 외부 참고 구현의 이식·수정 경로다. 이 문서는 CTRL 팀이 확인할 역할과 교체 범위를 정리하며, 이전 작성자의 원래 주행/튜닝 기록은 보관 원문에 있다.

## 판단 흐름

도로 기준 로터리 위치와 물체의 점유/운동을 구성하고, 진입 가능한 간격과 시점을 평가한다. 진입 뒤에는 선행차 추종·감속·정지와 합류 중 계획 유효성을 함께 판단한다. 주요 구현은 `ctrl_roundabout_planner`, `ctrl_roundabout_boxes`, `ctrl_ring_merge`, `ctrl_entry_gate`와 behavior node의 연결부다.

## CTRL 교체 시 검사

앞/뒤 차량의 접근·감속, 선행차 정지, 관측 상실, 불확실한 크기/속도, 진입 중 무효화를 나눈다. 출발 순간 간격만 검사하지 않고 실제 합류 경로 전체를 평가한다. 진입 시간을 줄이는 것은 충돌/도로 조건을 통과한 뒤 비교한다.

현재 설정은 [missions.yaml](config/missions.yaml), [behavior.yaml](config/behavior.yaml)에서 확인한다. 기존 지면 보정과 주행 근거는 [10/23 로터리 기록](../../docs/ROUNDABOUT_WAIT_2026-10-23.md)에 있으며 새 설계의 검증 결과와 구분한다. 새 구현 순서는 [8단계 계획](../../docs/INDEPENDENT_REBUILD_PLAN_2026-10-25.md)에 있다.

## 이전 문서 원문

[교체 전 원문과 당시 기록](../../../%EA%B3%BC%EA%B1%B0%EC%9E%90%EB%A3%8C/ctrl-document-history-20261025/ros1-port/src/ctrl_planning/ROUNDABOUT.md). 원래 작성자·날짜·출처와 실험 수치를 보존했다.
