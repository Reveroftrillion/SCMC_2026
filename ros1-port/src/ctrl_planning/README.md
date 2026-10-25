# CTRL 팀 미션 판단 — ctrl_planning

차량 위치·인지·신호를 받아 경로/속도/정지 목표를 결정한다. 현재 실행은 기존 이식·수정 경로이며, 새 미션별 구현은 팀 교체 계획의 후속 단계다.

## 실행 구조

`ctrl_behavior_node`가 관측과 상태를 모아 `/behavior/target`을 발행한다. `ctrl_mission`은 구간별 소유/속도 정책, `ctrl_state`는 상태와 제약을 결합한다. 실행 구간의 기준은 [missions.yaml](config/missions.yaml), 정책 값은 [behavior.yaml](config/behavior.yaml)이다.

| 기능 | 현재 주요 구현 |
| --- | --- |
| 기본 주행/신호 | `states/ctrl_cruise`, `ctrl_signal_stop` |
| 정적 회피 | `states/ctrl_avoid`, `ctrl_static_course`, `ctrl_avoid_planner` |
| 보행자 양보 | `states/ctrl_yield`, 관측/도로 겹침 판단 |
| 로터리 | `ctrl_roundabout_planner`, `ctrl_roundabout_boxes`, 진입/추종 helper |
| 고속 구간 | `ctrl_highway_runtime`, `ctrl_highway_policy`, slot/maneuver helper |
| 터널 | `states/ctrl_avoid`, `ctrl_tunnel_simple`, BEV/벽/위치 추정 연결 |

터널은 `states/ctrl_tunnel.py` 하나로 동작하는 것이 아니다. 정적 회피와 터널이 공유하는 경로를 고려해 교체한다. 고정 장애물 사전 정보를 쓰는 현재 설정도 실제 관측과 도로 검사까지 포함해 평가한다.

## 연결 계약과 개발

`DrivingTarget`의 속도는 m/s, 정지는 앞바퀴 기준 전역 station, 횡방향 offset은 좌측 양수다. chain/reference가 있는 경우의 의미는 [제어 계약](../../docs/CONTROL_CONTRACT_2026-10-25.md)을 따른다.

새 조향/경로 계산은 [ctrl_control](../ctrl_control/README.md)에서 오프라인 개발 중이다. 현재 planning 노드가 새 계산에 연결돼 있다는 뜻은 아니다. [로터리 안내](ROUNDABOUT.md), [고속 구간 안내](HIGHWAY.md), [팀 개발 계획](../../docs/DEVELOPMENT_WORKFLOW.md)을 참고한다.

## 이전 문서 원문

[교체 전 원문과 당시 기록](../../../%EA%B3%BC%EA%B1%B0%EC%9E%90%EB%A3%8C/ctrl-document-history-20261025/ros1-port/src/ctrl_planning/README.md). 원래 작성자·날짜·출처와 실험 수치를 보존했다.
