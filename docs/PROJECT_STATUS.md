# 프로젝트 개발 상태

기준: feat/local-planner 작업 트리, 2026-10-06. 상태는 실제 실행 코드 기준이다.
✅는 해당 기능의 필요한 검증까지 끝난 경우, 🟡는 구현 후 MORAI 검증이 남은 경우,
🟠는 미션/통합 일부만 구현한 경우, ❌는 실행 기능이 없는 경우다.
수학 테스트 통과를 전체 주행 검증으로 표시하지 않는다.

| 모듈 | 상태 | 근거와 남은 일 |
|---|---|---|
| Global Planning | 🟡 구현됨, MORAI 검증 필요 | 고정 경로/부분 경로 구현. README 한 바퀴 보고 있음. 현재 경로 중복/곡률과 index/종료 재검증 필요. 이번에 원본 경로/Global node는 변경하지 않음 |
| Local Planning | 🟡 구현됨, MORAI 검증 필요 | ROS 없는 core, reference 품질 방어, 접근 속도 제한, 기억/복귀/HOLD 구현. Python 합성 50개 통과; 현재 C++/ROS/MORAI는 미실행 |
| LiDAR | 🟡 구현됨, MORAI 검증 필요 | static DBSCAN 헤더/빈 scan/overflow 보호 구현. ROI·자체영역·지면 높이·TF 실측 필요. car 검출기는 stale 배열/빈 scan/경계 문제 잔존; 이번 수정 범위 밖 |
| Control | 🟡 구현됨, MORAI 검증 필요 | 기본 Pure Pursuit/P 속도 제어 유지. LocalPlan validator/stop/접근 cap만 수정. C++ 계약 검사와 ROS smoke test 준비; 현재 환경에서 실행하지 못함 |
| Traffic Light | 🟠 일부 구현 | YOLO 문자열 연결. 정지 이벤트는 sentinel만 존재. red 외 통과/미검출 freshness/대상 신호 선택 미완성. 기록만 하고 코드 미변경 |
| GPS Shadow | 🟠 일부 구현 | HSV 차선 조향/45km/h/임의 index 증가. 위치 융합·영상 freshness·복구 미완성. 코드 미변경 |
| Static Obstacle | 🟠 일부 구현 | 중앙/좌/우/차단/기억/복귀 합성 검증. 실제 zones와 extrinsics 비어 있음; 치수/도로 폭 실측과 실제 회피 주행 필요 |
| Dynamic Obstacle | 🟠 일부 구현 | 기존 중심 경로 차단 정지/10km/h 진행 정책 유지. zones 미설정, tracking/속도 예측 없음 |
| Merging | 🟠 일부 구현 | 구형 함수 존재하나 in_merging_zone=false. 시간 경과 후 장애물 무시 분기 재설계 필요. 코드 미변경 |
| Local 수학/상태/adapter 합성 검사 | ✅ 구현 + 검증 완료 | Windows Python에서 50개 통과. ROS transport·TF 보간·차량 운동을 검증한 것은 아님 |
| 동적 객체 tracking/예측 | ❌ 미구현 | 현재 실제 planner에는 객체 속도·미래 위치 모델 없음 |
| GPS 음영 추측항법 | ❌ 미구현 | IMU는 수신하나 현재 위치 추정에 미사용 |
| 전체 미션 상태 조정 | ❌ 미구현 | 신호/합류/음영/Local 상태와 정지 원인의 통합 manager 없음 |

## 이번 branch 변경

- 원본 경로와 Global/Control 기본 추종 계산 유지.
- ROS import에 막히던 Local core를 분리. 기존 utils helper API는 유지.
- near-duplicate/짧은 segment 제거, yaw/곡률/길이/spline 실패·NaN 방어.
- index/UTM zone 검증, 경로상 접근 속도 cap, startup geometry 로그.
- scan 취득 시각 TF 요구 및 명확한 HOLD reason, 장애물 기억의 원자적 갱신.
- 안전한 center 후보로 구간 안에서도 복귀, HOLD 때 정렬 타이머 초기화.
- LocalPlan stop을 active와 무관하게 처리; 경로/시간/frame/quaternion 계약 강화.
- 기존 marker에 selected offset/cost/clearance와 HOLD/STOP 이유 표시.

## 검증 결과와 한계

- `python -B -m unittest discover -s src/planning/tests -v`: 50개 통과.
- `python -B -m unittest discover -s src/MORAI_UDP_NetworkModule/tests -v`: 기존 UDP 회귀 13개 통과.
- 변경 Python 파일의 Python 3.8 문법, launch XML, `git diff --check` 확인 완료.
- 새 C++ standalone validator 테스트: Ubuntu 실행용, 현재 컴파일러 없어 미실행.
- 확장 `ros_local_planner_smoke.py`: 합성 ROS 실행용, 현재 ROS 없어 미실행.
- MORAI 실제 주행/RViz 정합: 미실행. 이전 문서의 ROS 통과 보고는 이번 변경 검증을 대체하지 않음.

## 현장 입력과 다음 우선순위

1. 실제 미션 zones, current_pose 기준점, LiDAR TF, 차량/도로 치수를 확정한다.
2. Ubuntu C++ 검사·catkin 빌드·ROS smoke test를 실행한다.
3. UDP 송신을 끈 상태로 센서/TF/state 정합을 확인한다.
4. 접근 감속→정적 회피→차단 정지→복귀를 낮은 속도부터 검증한다.
5. 제동거리·추종오차로 approach/geometry/속도 파라미터를 조정한다.
6. Local 검증 후 신호등/음영/합류의 정지 원인·우선순위를 별도 작업에서 조정한다.

GPS 음영에서는 기존 차선 분기가 접근 cap을 적용하지 않는 문제가 남는다.
신호등/합류/GPS 음영 코드를 이번 Local 개발에서 통합 수정하지 않았다.
구간/센서값을 임의로 채우지 않았으며 commit/push도 수행하지 않았다.
빌드/launch/토픽/예상 로그는 [LOCAL_PLANNING.md](LOCAL_PLANNING.md)를 따른다.
