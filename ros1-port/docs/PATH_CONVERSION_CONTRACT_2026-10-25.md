# 경로 변환과 전역 station 계약 — 2026-10-25

## 목표와 범위

일반 경로, 명시적인 회피 entry/return·chain, local reference를 새 `Polyline`/진행 관리자/조향 계산에 전달한다. 기존 `Route`, reference controller나 회피 곡선 함수를 import하지 않는다. 이번 영향은 새 `ctrl_path_conversion.py`, 검사·검증 도구와 현재 계획 문서다.

## 거리와 불변 데이터

- `StationedPath`는 불변 `Polyline`과 각 꼭짓점에 대응하는 전역 station을 가진다. 전역 station은 유한하고 엄격히 증가해야 하며 좌표 점 개수와 같아야 한다.
- local 거리는 실제 전달 경로의 첫 점부터 누적한 길이, 전역 station은 원래 미션/경로의 라벨이다. 두 값이 같거나 상수 차이라고 가정하지 않는다.
- 양방향 변환은 인접 꼭짓점 사이 선형 보간이다. 범위를 벗어난 요청을 끝점으로 clamp하거나 외삽하지 않고 거부한다. 경로 길이와 station 라벨 구간의 길이가 다를 수 있다.
- 일반 경로는 호출자가 지정한 전역 원점과 polyline 누적 길이로 구성한다. spawn 연결부의 음수 전역 원점도 명시할 수 있다. 자동 route 선택은 이 모듈의 책임이 아니다.
- local reference는 x/y/global station 배열을 원자적으로 검증·복사한다. 일부 배열이 잘못되면 경로 객체를 만들지 않는다. 동일 ID로 형상을 바꾸는 메시지, 해제/유지 정책과 요청 시각 관리는 후속 메시지 선택 계층에서 다룬다.

## 회피 변환

- `OffsetMove(start_global_s, length_m, target_offset_m)`는 전역 station에서 시작해 정해진 길이 동안 목표 횡오프셋으로 이동한다. 길이는 양수이고 끝 station이 표현 가능해야 한다.
- 움직임은 입력 순서대로 시작 station이 증가하고 서로 겹치지 않아야 한다. 시작/끝은 기준 경로의 전역 범위 안에 있어야 한다. 좌회전 방향 오프셋이 양수다.
- 초기 오프셋은 호출자가 명시한다. 각 움직임은 직전 목표값에서 새 목표값으로 `u³(10-15u+6u²)`로 연결한다. 이는 양 끝의 1·2차 미분이 0이라는 경계조건에서 얻은 선택이다. 움직임 사이와 종료 후에는 마지막 오프셋을 유지한다.
- entry/return은 entry 목표 오프셋과 명시적인 return-to-zero 두 움직임으로 표현한다. chain은 동일 길이의 세 배열을 검증한 후 움직임 목록으로 만든다. entry가 없는 legacy offset은 호출자가 시작·길이를 먼저 확정해야 하며 현재 위치로 자동 재설정하지 않는다.
- 기준 polyline의 꼭짓점 단위 접선을 평균하고 선분 안에서 보간·정규화한 방향의 좌측 법선으로 offset을 적용한다. 방향이 반전돼 법선을 정할 수 없는 꼭짓점은 거부한다. 이는 새 기하 선택이며 기존 곡선 구현을 재현한다는 뜻이 아니다.
- 기준 경로 길이의 일정 간격, 원래 꼭짓점과 전환 경계에서 표본을 만든다. 수치상 1µm 이내 표본은 합친다. 변환 후 실제 길이로 local 거리를 새로 계산하고, 원래 전역 station 라벨은 보존한다.
- 표본 간격은 기준 경로 길이 기준이며 변환 후 XY 간격이나 곡률의 상한을 보장하지 않는다. offset 상한과 표본 개수 한계는 호출자가 제공한다. 새 경로 ID는 기준 경로 ID와 달라야 한다.
- 변환된 선분이 기준 진행 방향의 반대로 향하면 거부한다. 이 검사만으로 차체 충돌·도로 포함·실제 조향 가능성을 보장하지 않는다.

전체 XY 경로의 C2 연속성은 보장하지 않는다. C2 조건은 offset 프로파일의 각 전환에 대한 조건이며 원래 polyline의 모서리까지 매끄럽게 만드는 조건이 아니다.

## 기존 DrivingTarget 필드와 연결할 의미

| 필드 | 이 단계에서의 대응 |
| --- | --- |
| 기본 경로 | 명시적 origin을 가진 `StationedPath.from_global_route()` |
| `reference_x/y/global_s` | `StationedPath.from_reference()` |
| `lat_offset`, `entry_s/length`, `return_s/length` | `entry_return_moves()` → `build_offset_path()` |
| `chain_offset/start_s/length_m` | `moves_from_chain()` → `build_offset_path()` |
| 검색할 전역 station 범위 | `local_window()`로 변환해 `ProgressTracker.acquire()`에 전달 |
| 새 진행 관리의 `station_m` | `global_at_local()`로 전역 라벨 복원 |
| `stop_s` | 여전히 앞바퀴 기준. 차체 기준점 변환/종방향 제어는 후속 작업 |

이번 함수들은 ROS 메시지를 구독하거나 모드 우선순위를 추정하지 않는다. 메시지 충돌, active reference의 ID 불변성, 해제 요청과 시간 만료는 별도의 선택 계층을 거쳐야 실행 연결할 수 있다.

## 구현 전 고정한 검사 기준

- 일반 경로의 양수/음수 원점, 비균일 global station, 양방향 보간과 범위 밖 거부.
- 참조 배열 길이/순서/수치/중복 좌표 검증과 입력 복사.
- 직선에서 entry/hold/return의 위치, 좌우 반사, 전환 경계와 겹친 움직임 거부.
- 회피로 실제 길이가 늘어나도 같은 global station으로 역변환됨.
- 곡선의 좌측 offset 방향, 좌표 이동/회전 불변성, 기준 진행 방향 반전 거부.
- 잘못된/과도한 offset, 범위 밖 전환, 표본 수 초과와 경로 ID 충돌 거부.
- 세 경로 형태를 진행 관리자와 조향 계산까지 연결. 전역 station이 단조이고 검색/조향 제한을 지키는지 검사.

실제 주행 연결·도로/충돌 검사와 MORAI 시험은 이번 완료 기준에 포함하지 않는다.
