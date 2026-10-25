# 오른쪽 정적 회피·보행자 연속 입력 검증 — 2026-10-22

목표: 사용자가 요청한 1차로→오른쪽 우회·복귀를 구현하고, 기존 완주 기록의
보행자 분류/양보 미완료 원인을 재현해 근거가 확보된 부분만 최소 수정한다.
실제 MORAI 주행과 제어 UDP 송신은 이번 작업에서 실행하지 않는다.

브랜치 `fix/right-static-pedestrian-20261022`. 이전 브랜치
`test/fullcourse-20261022`의 수정·미추적·삭제 상태를
`docker/artifacts/right-pedestrian-20261022-141846/`의 incoming-source.tar.gz,
incoming-files.json, incoming-status.txt, incoming.patch에 보존했다.
이전 왼쪽 회피 완주 기록과 과거 검증 JSON/manifest는 변경하지 않는다.

영향 범위: static_course.py와 관련 회귀, 보행자 실패 원인에 해당하는
인지/판단 파일, 재현 도구 및 이번 문서. 경로·CP·메시지·포트·신호 정책과
근거 없는 threshold 변경은 범위 밖이다.

방법: 원본 지도·장애물 geometry로 오른쪽 곡선의 차체 여유/포장도로/CP4를
검사하고 기존 연속 footprint로 제동 유효성을 확인한다. 보행자는 같은 run-02의
원본 LiDAR/위치/미션을 순서대로 재생해 특징·추적·분류·YIELD를 대조한다.
완료 기준: 수정 전 실패와 수정 후 결과, 정적·보행자·터널 관련 회귀,
확인 불가능한 실제 폐루프/공식 차선 채점 범위를 명시한다.

## 정적: 오른쪽 진입·복귀

기존 MOVES는 `(왼쪽+4m, s310, 길이12m) → (원점, s333, 길이18m)`였다.
단순히 -4m로 뒤집으면 s312~322와 s335~346의 회전한 차체가 지도 경계를
벗어난다. 검사를 완화하지 않고 같은 지도/상자/차체 여유로 오른쪽 후보
282개를 조사했다. 장애물 비접촉240개 중 포장도로 검사까지 통과한 후보는40개였다.

채택한 MOVES는 `(오른쪽-3.5m, s316, 길이12m) → (원점, s331, 길이12m)`.
도로가 열리는 구간 안에서 진입하고 우측 경계가 좁아지기 전에 복귀한다.
두 곡선은 C2 연속이며 기존 조향 곡률 한계 안이다. 제공 상자의 회전·크기를
포함한 차체 충돌 검사, 기존 포장도로 검사 통과. s343에서 복귀가 끝나므로
CP4(s345.14, 반경3m)의 계획된 경로는 원래 경로와 일치한다.

실제 BEV·신호 제동, 경로 SHA 일치, 다른 경로/미션 fallback, 늦은 진입
거부는 유지했다. 기존 정적 테스트의 신규 장애물 fixture는 왼쪽 고정 좌표에서
현재 MOVES의 회피 경로 위 좌표로 변경했다. 검사 기준인 '진로의 새 장애물이
있으면 동일 chain을 유지하고 정지'는 그대로이며 이전 원문은 스냅샷에 보존했다.

기존 왼쪽 완주 기록을 연속 footprint/신호 입력으로 재생해679개 target tick을
확인했다. 새 오른쪽 chain147개 중 최종 속도 상한0 요청22개가 있었다.
그중12개는 관측 box와 계획 경로 충돌, 나머지10개는 시야 부족이었다.
비교용 -4m 후보는 관측 box 충돌38개로 더 많아 채택하지 않았다.
기존 녹색/적색 정책과 관측 장애물·시야 부족 제동을 제거하지 않았다.
왼쪽으로 이동한 실제 pose/시야를 고정한 재생이므로 이 수치는 오른쪽 주행의
성공·정지 시간을 예측하지 않는다. 새 경로의 실제 추종·무충돌·차선 접촉 시간과
동일 방향 2차로 준수 여부는 MORAI 폐루프 및 차선/공식 판정 확인이 필요하다.

![오른쪽 계획 경로와 이전 왼쪽 경로](../docker/artifacts/right-pedestrian-20261022-141846/static-right-geometry.png)

그림의 회색은 지도 포장도로 검사 영역이다. 실제 바퀴 궤적·차선 변경 허용선이나
공식 차선 채점 그림이 아니다. 지도 검사 통과를 무감점으로 해석하지 않는다.

## 보행자: 높이 기준 실패 재현과 최소 수정

입력은 `fullcourse-20261022-130819/run-02/diagnostics_0.bag`의
1792643916~1792643934초, 원본 cloud149개와 위치·미션·TF다.
추출 manifest SHA256:
`3c2ac78788d8897dcd432754cc5d18458f05378eaf92a53a23fc1dcb4a14e0d3`.
입력 순서·센서 stamp를 보존하고 별도 디렉터리에 수정 전후 결과를 썼다.

원인: 이 구간의 도로가 기존 지면 기준보다 높아 일반 평면 추정의 절편
보호 조건을 만족하지 못했다. 149스캔 중 일반/기존 보정 평면은6개뿐이었다.
보행자가 차로를 횡단하는 동안 top이2.18~2.39m, 몸통 높이 점 비율이
0~0.43으로 관측돼 기존 PED_TOP 상한2m·PED_MID_SHARE0.5에서 탈락했다.
교차 대상의27개 분류는 OBSTACLE6/DYNAMIC_UNKNOWN21이었다.

수정은 obstacle_node.detect의 기존 재질 기반 지면 보정 허용 미션에
PEDESTRIAN을 추가한 것이다. 새 임계값·센서 장착 보정·보행자 강제 분류는
없다. fresh 미션+intensity가 있고 일반 평면이 실패한 경우에만 기존
estimate_material_ground_plane/material_height_region을 사용한다.
같은 스캔의 넓은 asphalt 지지·점수·평면 잔차·기울기 검사를 그대로 적용한다.
비터널 intensity0/차선 점 제거와 군집 정책은 바꾸지 않았다. 지지가 부족하면
기존 높이 fallback을 유지한다. HIGHWAY/ROUNDABOUT/TUNNEL은 이번 변경의
추가 대상이 아니며 분류·YIELD·속도 정책 자체도 수정하지 않았다.

같은 입력에서 평면126개가 검증됐고 교차 대상27개 모두 PEDESTRIAN이었다.
전체 윈도우의 PEDESTRIAN track은 수정 전1개(부분 관측6회), 수정 후1개
(연속41회)였다. 다른 종류로 바뀌거나 끊기던 횡단 대상의 정체성이 유지됐다.
제공 sample_scene의 보행자 초기 위치(-56.744,-11.614), 속도10km/h와
관측 위치·횡단 속도약2.75m/s가 부합한다. 시뮬레이터 객체 ID를 직접
대응한 ground truth 검증은 아니다. 이 윈도우 밖 오분류 부재도 의미하지 않는다.

생성된 실제 Event를 production BehaviorNode.take_event로 변환하고 원래
Yield/Behavior/MissionSupervisor에 순서대로 넣어 비교했다:

| 항목 | 수정 전 보존 코드 | 수정 후 |
| --- | --- | --- |
| YIELD 입력 종류 | DYNAMIC_UNKNOWN | PEDESTRIAN |
| YIELD cloud tick | 7 | 10 |
| 첫 YIELD 시각·s | 1792643924.859 / s452.57 | 1792643923.977 / s447.67 |
| 보행자 전용 완료 | 없음 | 1792643925.227 / s454.32, released: moved away |

완료를 강제로 true로 만들거나 대상 소실을 통과로 취급하지 않았다. 원래의
관측된 이탈 조건이 해제를 판단했다. 원본 pose를 고정한 1cloud당1회 평가이며
카메라 신호 합성·ROS executor20Hz·변경된 제동 후 차량 운동은 재현하지 않는다.
따라서 실제 정지거리·양보 완료 시각·새 코스 완주 성공은 아직 미검증이다.

## 파일·회귀 및 남은 확인

production 변경은 static_course.py의 우회 chain과 obstacle_node.py의
지면 보정 조건/주석뿐이다. 새 원본 cloud fixture2개(각 SHA·센서 stamp·
실제 위치 포함), test_pedestrian_ground_recording.py, test_static_right.py,
재현 도구 tools/replay_pedestrian_sequence.py와 이번 기록을 추가했다.
기존 경로·CP·모델·메시지·포트·과거 결과·원본 저장소는 유지했다.

수정 전 보존 코드로 같은 새 회귀를 실행하면2 FAIL/3 PASS였고, 수정 후
관련 기존/새 검사10개가 PASS였다. 검사 초기 버전은 객체 위치를 대략8m로
가정해 다른 점을 선택하려 했으며, 원본 재생의 실제 center와 위치/지도 문맥을
사용하도록 harness를 수정했다. 실패 로그와 수정 후 v3를 모두 보존했다.
생산 분류 기준을 바꿔 검사에 맞추지 않았다.

최종 격리 검증 결과(final-checks/checks):

| 검사 | 결과 |
| --- | --- |
| control/planning/lidar/camera 전체 알고리즘 | **944 PASS**, 실패0·skip0, 337.38초 |
| replay·감독 도구 | 13 PASS |
| ROS1 어댑터 | 27 PASS |
| native 직렬화/TF | PASS |
| 격리 catkin build/install | 각각 PASS |
| native entrypoint9개 import, launch 노드/파일 정적 확인 | PASS |
| 구문·현재 문서 상대 링크20개·CRLF 허용 diff --check | PASS |
| packaging/packaging-unit | 기존 메시지 줄바꿈 및 ROI/behavior.yaml resource hash 불일치로 FAIL(단위7 PASS/1 ERROR) |
| identifier-ast/boundary/preflight | 기존 기능 수정·원본 기준 불일치로 FAIL; 이번 지면 분기도 boundary 차이에 포함 |
| devel/install 환경 | 각각9 PASS/1 FAIL(resources)/1 KNOWN_SOURCE_ISSUE(model README hash) |

과거 기대값/원본 manifest를 갱신해 FAIL을 지우지 않았다. 이전 세션에도
실패하던 검사이며 새 알고리즘 회귀 실패와 구분한다. Windows·실제 MORAI·
drive=false native 수신 시험은 이번 작업에서 실행하지 않았다.
source 복사본의 catkin build/install과 native 메시지·import까지만 검증했다.

입력 보존 검사는 수정4개(생산2·기존 검사1·현재 안내1), 신규7개, 기존 누락0개다.
기존 static/ground/avoid 수정과 삭제4개도 그대로 보존했다. 구문 검사는 변경/
신규 Python6개를 대상으로 했다. preservation.json, session-changes.patch,
diff-check.log에 근거를 남겼다. commit/push/merge는 하지 않았다.

이전 전체 코스 완주는 왼쪽 회피·보행자 변경 전 결과로 남긴다. 이번 오른쪽
회피와 보행자 수정본의 실제 주행은0회다. 다음 실제 시험은 drive=false의
시작점·센서·Status·TF·target·UDP0 확인 후 감독·원본 기록을 켜고 수행한다.
확인할 순서는 오른쪽 곡선 추종/차체·차선/CP4/복귀, 보행자 제동/관측된 이탈/
완료 후 재출발, 정적 이후 전 구간 회귀다. 고속도로 보행자 이벤트5개의 원인은
이 미션 한정 지면 보정으로 해결했다고 주장하지 않는다.

## 재현 명령

아래 명령은 저장소 루트에서 실행하는 오프라인 재생이다. ROS master나
제어 소켓을 생성하지 않는다. 출력 디렉터리는 매번 존재하지 않는 이름을 쓴다.

```bash
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 \
  python3 tools/replay_pedestrian_sequence.py \
  --input docker/artifacts/right-pedestrian-20261022-141846/ped-input \
  --output docker/artifacts/right-pedestrian-20261022-next/pedestrian

bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 \
  python3 tools/replay_course_avoidance.py \
  --bag docker/artifacts/fullcourse-20261022-130819/run-02/diagnostics_0.bag \
  --output docker/artifacts/right-pedestrian-20261022-next/static \
  --stop-at 1792643918
```

수정 전 비교는 첫 명령에
`--obstacle-source docker/artifacts/right-pedestrian-20261022-141846/obstacle-before.py`
를 추가하고 별도 새 출력 경로를 사용한다. 보존 코드만 읽으며 작업 트리를 되돌리지 않는다.

검사 결과는 [최종 검사 JSON](../docker/artifacts/right-pedestrian-20261022-141846/final-checks/checks/results.json),
보행자는 [수정 전](../docker/artifacts/right-pedestrian-20261022-141846/ped-yield-before/pedestrian-summary.json)/
[수정 후](../docker/artifacts/right-pedestrian-20261022-141846/ped-yield-after/pedestrian-summary.json)를 참조한다.

복구용 incoming-source.tar.gz SHA256:
`d150b162952da3386e3d0bd0ca189a022990e6478077c06eb635d46dfc00d2a5`.
먼저 별도 빈 디렉터리에 해제하고 incoming-files.json과 SHA를 비교한 뒤
필요한 파일만 복구한다. 기존 삭제 상태는 incoming-status.txt 기준으로
유지한다. 전체 reset/clean으로 사용자 변경을 지우지 않는다.
