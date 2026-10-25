# 고속도로 차로 변경

CP10–CP13 고속도로 미션 구간을 담당한다. `run_competition.sh`에 통합돼 있으며 `src/ctrl_planning/config/highway.yaml`의 기본 모드는 `control`이다. 제어 송신 없이 판단만 보려면 `drive:=false`로 실행한다. 이 문서와 `highway.yaml`이 현재 기준이다.

![고속도로 지도 차로](images/highway-map.svg)

## 경로

- 지도 링크(`link_set.json`) 중심선을 연결해 5→4 → 4→3 → 3→2 순서로 한 차로씩 변경한다. CP11(4차로 s1286)은 2026-09-30 규정에서 빠져 4차로에서 기다리지 않는다(`highway.yaml` `hold_checkpoint: null`). 1차선은 제외한다.
- 모든 차로 경로의 진행거리는 원래 전역경로 s로 표시한다. 미션 진행거리와 지역 추종 경로 거리는 분리한다.
- CP12 40m 전부터 20m 후까지 위치·접선·곡률을 잇는 5차 복귀 곡선을 미리 한 번 만든다. CP12 반경 통과, 곡률 감소, 차체(위치오차 여유 포함)의 도로 내부 여부를 검사하고 실패하면 지도 차로 경로를 그대로 쓴다.
- 곡선 끝에서 원래 경로와 위치·방향이 정렬되면 `reference_release`로 전역경로 추종에 인계한다. CP13에서 미션이 끝난다.

## 판단 순서 (`HighwayPolicy.tick`)

| 상태 / 이유 | 조건 |
|---|---|
| `SCAN_UNKNOWN` | 스캔 무효 또는 0.25초 초과 — 정지 요청 |
| `ROUTE_FALLBACK` (`MERGE_CHECK`/`MERGE_YIELD`/`CP11_ROUTE`) | 5차로 끝(s1286, 옛 CP11 자리) 전 첫 5→4 변경 공간 소진. 기본 전역경로로 5차선 테이퍼 자연 합류 |
| `CHANGE` → `SETTLE` → `CHANGE_COMPLETE` | 변경 허가 후 경로 고정, 목표 차로 정렬 0.3초 유지 시 완료 |
| `REJOIN` (`REJOIN_CURVE`/`REJOIN_ALIGN`/`REJOIN_COMPLETE`) | CP12 이후 복귀 곡선 추종 및 인계 |
| `WAIT_GAP` | 변경 대기. 아래 준비 전략 사용 |
| `CHECKPOINT_PENDING` | `hold_checkpoint` 가 있을 때만: 그 체크포인트 실제 통과 전 4→3 금지 (현재 없음) |
| `CHANGE_DEADLINE` / `SLOW_TO_FIT` | 남은 변경 공간 부족 — 마감점 전 정지 가능 속도로 제한 |
| `MISSED_REJOIN` | 최종 2차선에 못 들어간 채 CP12 통과 — 정지, 강제 복귀 없음 |

- (`hold_checkpoint` 가 있을 때) 통과는 진행거리만으로 판단하지 않고 실제 궤적 선분이 반경을 지났는지로 본다. 반복된 동일 위치는 연속성 시간 기준을 옮기지 않는다.
- **즉시 변경 우선**: 현재 실측 속도와 후보 곡선으로 `assess_maneuver`가 통과하면 대기 없이 `CHANGE_STARTED`로 확정한다.
- 막히면 0.5초마다 `prepare_gap`이 `PASS_AHEAD`(가속) → `KEEP_SPEED` → `FALL_BEHIND`(검증된 감속) 순으로 준비 속도만 요청한다. 실제 변경은 다시 현재 속도 허가를 받아야 한다. 감속 목표는 낮아진 속도에서 반복해서 빼지 않는다.
- 자연 합류(`assess_route_merge`)는 후행차를 최대 20초 예측하고, 연속 정상 스캔 `clear_hold_s` 확인 후 진행한다. 테이퍼 진입 뒤에는 같은 후행차 때문에 다시 멈추지 않는다.

## 충돌 검사 (`assess_maneuver`)

- 자차가 목표 속도(필요 시 설정 가감속)로 변경 곡선을 따라가는 궤적 하나를 0.1초 간격으로 예측한다(최대 8초).
- 상대 군집은 현재 윤곽에 `속도 × (t + 관측 age)`를 더해 같은 시점에서 비교한다. 종·횡 속도를 모두 반영하므로 NPC 차로 변경도 포함된다.
- 다른 차로 차량의 횡속도 추정 범위가 0을 포함하면 횡이동을 외삽하지 않는다.
- 속도 미확정 군집: 자차 옆에 나란하면 `MOTION_UNKNOWN`으로 보류, 뒤쪽이면 `npc_speed_bound_mps`(20m/s) 접근 가정, 앞쪽이면 정지 가정.
- 변경 여유 폭(`margin_m`)과 현재 경로 추종 폭(차폭 + 위치오차)은 분리한다.
- 예측 창은 변경 완료(settle)에서 끝나므로, 끝난 시점에 더 빠른 뒤차와 `standstill_gap + 여유 + 접근속도 × rear_closing_s`(2 s) 가 없으면 `REAR_CLOSING` 으로 막는다. 가속 가정 판단(`ACCELERATING`)이 짧은 창으로 4 m 뒤 접근 차량 앞에 끼어들어 추돌한 뒤 추가했다.
- 늦게 시작하는 후보 경로(대기 후 변경)는 경로 시작점이 아니라 자차 실제 위치부터 예측한다.
- 직전 `history_s` 동안 진입을 막았던 물체가 이번 스캔에서만 사라졌으면 허가하지 않는다(`BLOCKER_UNSEEN`).
- 현재 속도로 막히면 `accel_mps2` 로 가속하는 궤적(속도 상한·곡선 상한 이내)으로 한 번 더 판단한다(`ACCELERATING`). 저속(`min_merge_speed_mps` 8 m/s 미만)에서는 먼저 속도를 올리고, 변경 길이를 짧게 잡아 빨리 끝낸다.

## 속도 상한 (`_follow_cap`)

- 곡률: 앞쪽 각 곡선까지의 경로 거리, 제동 한계, 지연으로 현재 허용 속도를 계산한다. 확정된 변경 중에는 변경 경로 최대 곡률을 유지한다.
- 앞쪽 경로는 항상 `v²/(2·제동한계) + lookahead` 이상을 본다. 100 m 창을 60 m 남을 때까지 재사용하던 시절 92 km/h 에서 곡선을 늦게 봐 overspeed FAULT 가 났다 (2026-09-26).
- 앞차 간격은 규칙 하나다 (`highway.yaml` 의 `following:` 5개 값, 2026-09-30 간소화).
  - 목표 간격 = `standstill_m + time_gap × 앞차속도`. 차선변경 곡선 위에서는 `time_gap_change_s`.
  - 멀면 `앞차속도 + min(sqrt(2 × approach_decel × 오차), gain_hz × 오차)`: 상대 감속 접근. 목표 근처(`2·approach_decel/gain²` = 16 m 이내)는 gain_hz 선형이라 간격 잡음에 상한이 튀지 않는다.
  - 가까우면 `앞차속도 + gain_hz × (간격 − 목표)`: 앞차보다 조금 느리게 가며 간격을 다시 벌린다.
  - 안전 상한: 앞차가 `follow_brake_mps2`(4.0)로 서도 지연 `latency_s` 뒤 `standstill_m` 앞에서 설 수 있는 속도. 항상 적용.
  - 변경 중 떠나는 차로에만 있는 앞차는 안전 상한만 본다. 변경 계획이 뒤에 붙기로 한 차(`CHANGE_BEHIND`)는 목표 간격만큼 앞설 때까지 계획 속도를 넘지 않는다.
  - 제어기는 v_max 가 내려가는 비율을 브레이크에 피드포워드로 더한다(`ctrl_core.py` `target_fall`). 비례 브레이크만으로는 2.2 m/s² 로 내려가는 상한을 약 7 km/h 뒤에서 따라가 목표 간격을 1.5–3 m 파고들었다.
  - 제동은 `JerkBrake`(`brake_jerk_mps3` 4, `brake_max_mps2` 8)로 부드럽게 건다. 비상 범위(지연 + 8 m/s² 정지거리 + standstill) 안의 물체만 예외.
- 속도 미측정 물체는 목표 간격 규칙 없이 안전 상한(정지 물체 가정)만 받는다. 속도는 트래커가 아니라 박스 가장자리 움직임으로 잰다. 박스는 추종 범위와 같은 100 m 까지 둔다(`boxes.front_m`). 40 m 였을 때는 그 밖의 차가 미측정(정지 가정)이라 45–58 m 앞 차 하나에 65–73 km/h 로 묶였다. 100 m 로 늘리니 판단 1틱이 28.6 → 31.5 ms. 차체 뒤에 완전히 있는 물체는 추종 대상에서 제외한다. 가까운 끼어들기는 0.6초 횡이동까지 본다.
- 고속도로 고정 순항 상한은 없다(`missions.yaml`의 `speed_kmh: null`). 제어기는 같은 체크포인트로 CP10–CP13 구간의 60km/h 상한을 해제하고 곡률 프로파일은 유지한다.

## 관측 (`HighwayObservations`)

- 현재 정상 전체 스캔의 군집만 사용한다. 사라지면 이력을 지우고 재등장하면 새 관측으로 시작한다.
- 약 0.6초 이력에서 중심·양 끝 경계를 선형 회귀해 축마다 잔차가 가장 작은 면의 속도를 쓴다. 새로 드러난 차체 끝이 속도로 튀지 않는다.
- 군집 ID가 바뀌어도 예측 위치 1m 이내로 일대일 대응되면 0.3초까지 직전 실측 속도를 승계한다. 분리·다중 대응·빈 스캔은 승계하지 않는다.

## 제어 연동

- 판단은 `DrivingTarget`에 `reference_id/global_s/x/y`로 지역 경로를 보낸다. 추종기(`ctrl_reference_control.py`)는 기존 `Controller`를 지역 경로에 새로 만들어 조향·적분 상태를 넘겨받는다.
- 지역 경로가 원래 경로와 연속으로 일치하는 끝부분은 원래 경로의 속도 프로파일을 계승한다(복귀 순간 과속 FAULT 방지).
- 연속 주행 중 전역경로 끝보다 앞에서 끝나는 지역 창은 목적지로 보지 않는다. 명시적 정지선과 실제 경로 끝 정지는 유지한다.
- 인계 시 실제 속도에서 `a_acc`로 올라가는 프로파일을 다시 만든다. ID 동일·형상 변경, 경로 이탈, 커버리지 부족은 `v_max=0`으로 막는다.
- 판단의 제동·횡가속·가속 가정은 `low_speed.yaml`의 `decel_mps2`/`a_lat_mps2`/`accel_mps2`와 `bounds` 중 작은 값을 쓴다.

## 코드

| 파일 | 역할 |
|---|---|
| `ctrl_planning/ctrl_highway_config.py` | 설정 검증, 제어 프로파일 결합 |
| `ctrl_highway_route.py` | 지도 차로 체인, 변경 마감점, CP 통과 판정 |
| `ctrl_highway_observation.py` | 군집 이력·속도 추정 |
| `ctrl_highway_policy.py` | 상태 판단, 속도 상한 |
| `ctrl_highway_maneuver.py` | 변경 곡선 생성, 충돌 검사 |
| `highway_gap_strategy.py` | PASS_AHEAD / KEEP_SPEED / FALL_BEHIND 준비 |
| `highway_merge_guard.py` | CP11 자연 합류 검사 |
| `ctrl_highway_rejoin.py` | CP12 복귀 곡선 |
| `ctrl_highway_runtime.py` | 미션 범위 어댑터, 진단 |
| `ctrl_highway_scene.py`, `ctrl_highway_markers.py` | RViz 표시 전용 |
| `ctrl_lidar/ctrl_highway_visibility.py` | 고속도로 구간 판정과 라이다 ROI(전방60/후방40/횡20m) |
| `ctrl_control/ctrl_reference_control.py`, `ctrl_speed_policy.py` | 지역 경로 추종, 구간 상한 해제 |

도구: `tools/record_highway.py`(기록), `replay_highway.py`(재생), `inspect_highway_route.py`(지도 차로 확인), `measure_highway.py`(CSV 요약, 제어 허가와 무관), `practice/sim/record_collision.py`(MORAI 충돌 패킷 기록, 연습 전용).

## RViz

- 노란 선: 변경 후보, 초록: 확정 변경 경로, 청록: `CP11_ROUTE` 기본경로.
- 군집: 변경 차단 빨강, 앞차 추종 주황, 나머지 회색. 옆에 맵상 차로 번호와 속도(미확정 `?`), 목표 차로 기준 `FRONT`/`REAR`/`SIDE`.
- `GAP NOW` 띠: 목표 차로 앞차 뒤끝–뒤차 앞끝 현재 공간. 테두리는 검사 전 회색/허가 노랑/거부 빨강/확정 초록. `OPEN END`는 관측 범위 끝이지 빈 도로 보장이 아니다. 표시는 판단에 다시 입력하지 않는다.
- HUD는 상태/차로와 이유 두 줄. 제어기가 FAULT/HOLD면 제어 원인을 우선 표시한다.

## 한계와 미검증

- 스캔 age가 0.25초를 넘으면 즉시 정지 요청(`SCAN_UNKNOWN`)이 나간다. 실주행에서 age 0.4초가 관측됐으며 순간 제동 원인 후보다.
- 속도 미확정 앞 군집을 정지로 가정해 첫 변경 부근에서 `LEAD_FOLLOW` 감속이 남는다.
- 관측 밖·가림 뒤 차량을 완전히 배제하지 못한다. 깜빡이는 입력으로 쓰지 않는다. `bounds`는 조정값이지 측정 인증이 아니다.
- 마지막 성공 실주행: `/home/foscar/foscar_logs/highway_relative_20260926_042441` — 세 변경·CP11·CP12·CP13 통과, 제어 FAULT 0, 최고 69.3km/h. 그 이후 추가된 ID 변경 속도 승계, CP12 정렬 인계 조건은 실주행 미검증이다. 혼잡 조건의 PASS_AHEAD·자연 합류 양보는 소수 주행으로만 확인했다.
