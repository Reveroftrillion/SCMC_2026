# Local Planner MORAI 시험 결과

아래 양식을 **실험마다 복사**하여 기록한다. 아직 실행하지 않은 phase는 NOT_RUN으로 유지한다.
PASS는 해당 시험 조건에만 적용되며 전체 대회 주행 완료를 의미하지 않는다.
실행 명령/수집 방법은 [오프라인 도구 안내](LOCAL_PLANNER_OFFLINE_TOOLS.md)를 따른다.

## 실험 기본 정보

| 항목 | 기록 |
|---|---|
| 날짜/시간/시간대 | TODO |
| 담당자 | TODO |
| branch | TODO |
| commit | TODO |
| git status / diff | TODO: clean 여부, 변경 파일; 변경 시 diff 첨부 |
| vehicle | TODO: MORAI 차량 모델, 폭/길이/wheelbase, current_pose 기준점 |
| network | TODO: MORAI/ROS IP, 상태 패킷 layout, 포트, 통신 이상 |
| YAML | TODO: 실제 파일 경로, SHA256, 복사본 첨부 |
| runtime rosparam | TODO: collector 폴더의 적용값과 YAML 차이 |
| 코스/zone | TODO: 시나리오 버전, 실제 index/UTM 구간 |
| TF/센서 | TODO: LiDAR frame, extrinsics 또는 외부 TF 공급자, 수신 주기 |
| test phase | TODO: Phase 번호, 반복 회차 |
| diagnostic folder | TODO: debug_logs/... |
| 전체 PASS / FAIL | NOT_RUN |

## Phase 0~10 진행표

min clearance와 selected offset은 m, 속도는 km/h로 기록한다. min clearance를 측정할 수 없으면
UNKNOWN과 이유를 적고 0이나 추측값을 넣지 않는다. state/marker/로그에 보이는 값과 실제 접촉/추종 상태를 함께 기록한다.

| Phase | 시험 내용 | PASS / FAIL | selected offset | min clearance | speed 목표/실제 | stop 요청/실제 | return-to-global | screenshot | rosbag | issue |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | Ubuntu catkin build + isolated ROS smoke, preflight 준비 | NOT_RUN | — | — | — | — | — | TODO | TODO | TODO |
| 1 | Local OFF baseline: 기존 Global/Control 기본 주행 | NOT_RUN | — | — | TODO | TODO | — | TODO | TODO | TODO |
| 2 | Local ON + empty zones: NORMAL/inactive, 기본 주행 유지 | NOT_RUN | — | — | TODO | TODO | — | TODO | TODO | TODO |
| 3 | Static zone + no obstacle: 정상 입력/TF, zone speed cap | NOT_RUN | TODO | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| 4 | 중앙 장애물: valid 측방향 선택·실제 회피 | NOT_RUN | TODO | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| 5 | 좌측 장애물: 충돌 없는 우측/center 후보, 실제 clearance | NOT_RUN | TODO | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| 6 | 우측 장애물: 충돌 없는 좌측/center 후보, 실제 clearance | NOT_RUN | TODO | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| 7 | 전체 후보 차단: stop=true/full brake·실제 정지거리 | NOT_RUN | TODO | TODO | TODO | TODO | — | TODO | TODO | TODO |
| 8 | 활성 구간 LiDAR input loss: stale→HOLD/full brake·복구 | NOT_RUN | TODO | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| 9 | fresh LiDAR 유지, scan 시각 TF failure: HOLD/full brake | NOT_RUN | TODO | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| 10 | RETURN_TO_GLOBAL: 중심 복귀·연속 정렬 후 NORMAL/Global | NOT_RUN | TODO | TODO | TODO | TODO | TODO | TODO | TODO | TODO |

Phase 번호는 [README](../README.md)의 검증 순서와 동일하다. Phase 1 baseline도 실제 차량 시험이다.
주행 전에 송신 OFF로 센서/TF/실측값을 먼저 확인하고 담당자의 기존 통제 절차를 따른다.
다중 장애물·소실/재등장·pose/Planner 중단·곡선/경로 끝·동적 정책은 별도 추가 시험으로 상세 양식에 기록한다.

## Phase 상세 기록 — 반복마다 복사

- 날짜:
- 담당자:
- branch / commit / dirty diff:
- vehicle:
- network:
- YAML / SHA256 / runtime 변경:
- test phase / 반복 회차:
- PASS / FAIL / NOT_RUN:
- 입력 조건: 위치/index/yaw, 장애물 배치·크기·움직임, zone, 센서 주기:
- 예상 state / 실제 state와 전환 시각:
- selected offset / candidate cost / 선택 변경 횟수:
- min clearance / 확인 방법:
- speed 목표 / 실제 / 접근 감속 시작 위치:
- stop 요청 / 실제 제동 / 정지거리 / 정지 지연:
- return-to-global: 진입·완료 시각, 횡오차/heading 오차, 전환 후 움직임:
- screenshot:
- rosbag: 파일 경로, 시작·끝 시각, 기록 topic:
- diagnostic folder:
- issue: 재현 순서, 기대/실제, 관련 로그, 심각도:

## 결과 수신 후 개발 기록

| issue | 증거/재현 조건 | 오프라인 재현 fixture/test | 원인 | Local 내부 수정 | 재시험 phase | 상태 |
|---|---|---|---|---|---|---|
| TODO | TODO | TODO | TODO | TODO | TODO | OPEN |

조건이 다른 실험을 한 결과로 합치지 않는다. 센서/차량/zone/commit/YAML이 달라지면 새 실험으로 기록한다.
신호등·합류·GPS 음영 충돌은 별도 issue로 기록하며 Local 시험 결과만으로 기존 로직을 통합 변경하지 않는다.
