# 수정 코드 전체 코스 시험 — 2026-10-21 KST

## 개발 목표와 범위

사용자 승인: 수정 코드의 짧은 `drive=false` 검사 후 MORAI 시작점부터
`drive=true` 전체 코스 시험, 원본 입력과 판단·제어 기록, 실패 시 종료,
속도 0 및 Manual/P 확인. 주행 중 production 코드를 수정하지 않는다.

영향 파일: 새 시험 감독 도구 `docker/audit_course.py`, 종료 판정 보조 도구와
검사, 이 문서. 기존 LiDAR 수정은 incoming.patch와 SHA256으로 동결한다.
주행 후에만 재현 근거가 있는 production 문제를 최소 수정한다.

구현: 별도 ROS master/recorder를 먼저 준비하고 전체 토픽·UDP 원본을 기록한다.
충돌 패킷, 지속적인 지도/경로 이탈, 센서 상태 유실, 120초간 1m 미만의
진행, 1500초 제한, 기록기/필수 launch 종료를 감시한다. 이 수치는 시험
종료 조건이며 차량 제어 파라미터를 변경하지 않는다. 충돌 패킷 미수신은
무충돌 증거로 간주하지 않는다. 지도 공백도 보수적인 시험 종료를 일으킬 수
있으며 원본 영상/경로와 대조하여 실제 이탈과 구분한다.

검증: 종료 판정 경계 검사, Python 구문 검사, native ROS의 drive=false,
그 후 승인된 주행. 완료 기준: 도달 구간과 미도달 구간, 각 체크포인트 최소
거리, 미션 전환, 정지/target 지연/신호/인지/회피/이탈, 종료 인계 근거와
다음 수정 우선순위를 기록한다. 완주 자체를 성공으로 가정하지 않는다.

## 사전 확인

- 브랜치 `test/fullcourse-fixed-20261021`, 기준 HEAD
  `0fe5ae635f5acbc79aae9e9c7f88a3474a63b233`.
- 기존 ROS master/컨테이너 및 입력 포트 점유 없음. MORAI 실행 중.
- 새 근거 폴더 `docker/artifacts/fullcourse-fixed-20261021/`.
- 수정 코드 native drive=false 25초: PASS, errors 0, 제어 UDP 0.
  센서·위치·TF·target·제어 요청 수신. target 약20Hz.
- 최초 위치 s=14.7036m, 속도 약0, P. 시작점 밖이어서 사용자가 리셋 완료를
  회신했다. 실제 시작 위치는 주행 전 다시 관측하여 기록한다.

## 실행과 결과

**무개입 전체 코스 시험 실패: s=2.99→323.57m에서 정적 객체 ID 2 충돌로 종료.**
이후 구간을 통과하기 위한 수동 이동·로직 해제·재출발을 하지 않았다.
차량은 종료 후 원본 Status에서 정지·Manual/P를 확인했다.

![이번 시험의 도달 구간과 속도](../docker/artifacts/fullcourse-fixed-20261021/analysis-02/coverage.png)

### 실행 구분과 인계

- `run-01`: 외부 감독 도구가 CollisionData의 type=-1/id=0 무충돌 슬롯을
  오판하여 출발 전에 종료했다. 제어 UDP 0, 차량 이동 없음. 주행 검증으로
  집계하지 않는다. 제공된 원본 `practice/avoid_test/avoid_mon.py`의
  `typ >= 0 and oid != 0` 판정과 실제 raw packet을 대조하여 감독 도구만 수정했다.
- `run-02`: 실제 Auto(2)/D(4), 3,865개 제어 UDP, 최대58.73km/h.
  bag 기록77.63초, 감독 프로세스84.06초. 충돌 직전 s≈323.48m,
  XY≈(-59.75,-146.21), 속도0.593m/s. 첫 충돌 패킷 wall epoch
  1792577108.0719635, type2/id2, 객체 위치≈(-60.613,-142.153).
  종료 트리거 파일은 감시 스레드가 마지막으로 관측한 후속 충돌 패킷을 담는다.
- 종료 후 서로 다른 Status stamp의 연속30개/0.657초에서 속도 절댓값
  최대0.000843m/s, mode1/gear1. 마지막 수신 속도 -0.000002305m/s.
  따라서 정지 허용오차0.01m/s 안에서 **Manual/P 인계 확인**.
- master/recorder/launch 모두 종료코드0, 컨테이너·ROS master·909 수신 점유
  제거 확인. `source-before.json`과 `source-after.json` 동일:
  시험 중 `src/` 소스·설정·모델·경로 변경 없음.

### 체크포인트와 미션

| 항목 | 관측 | 판정 |
| --- | --- | --- |
| START | 초기 후륜축 위치와 지정 좌표 거리2.988m | 로컬 위치 기준 반경3m 안 |
| CP1 | 최소거리0.397m | 통과 |
| CP2 | 최소거리0.080m | 통과 |
| CP3 | 최소거리0.061m | 통과 |
| CP4~10, CP12~15, END | CP4 앞 약21.57m에서 종료 | **미도달·미검증** |
| SIGNAL_1 | red457 / green29, 적색 정지 후 재출발 | 해당 시험의 신호 통과 확인 |
| STATIC_OBSTACLE의 intersection_02 | red477 / green8, 적색 정지 후 재출발 | 신호 해제 확인, 장애물 회피 실패 |
| 보행자·후속 신호·회전교차로·고속도로·터널·종료 | 도달하지 않음 | **미검증** |

거리는 `/localization/odometry`의 후륜축 기준이며 공식 MORAI 채점 결과가
아니다. START와 END는 같은 좌표여도 구분한다. 시작점 근접을 END 통과로
집계하지 않는다. 기록 뒤쪽 CP의 다른 도로에서 측정한 최소거리도 통과로
집계하지 않는다.

### 기록과 증상 분석

- 원본 GPS774, IMU1,876, LiDAR cloud638, 카메라 JPEG1,680,
  차량 Status2,016, odometry3,857, footprint633, 이벤트234,
  road_users185, target1,527, control request/telemetry 각1,543개를 bag에 보존.
  TF/static TF·경로·인지 및 계획 마커·clearance·신호 판단도 포함한다.
- 원본 UDP는 pcap, CollisionData/Status/제어 파싱은 `wire.jsonl`.
  충돌 패킷 채널은 실제3,496개 수신했다. 카메라는
  [camera.mp4](../docker/artifacts/fullcourse-fixed-20261021/analysis-02/camera.mp4)로도
  내보냈다. 재생용 MP4는 관측 평균 FPS이며 정확한 프레임 시각은
  `camera-timestamps.csv`와 원본 bag을 사용한다.
- 2초 이상 정지: 첫 신호19.65초, 두 번째 신호18.65초.
  둘 다 기록상 적색 정지 및 녹색 해제와 연결된다. 이번 도달 구간에서
  장기 무진행 종료는 발생하지 않았다.
- target 최대 간격0.120918초, 1초 이상 공백0건. 초기 준비 중
  NO_TARGET 12행은 별도로 보존하며 주행 중 반복 목표 단절로 해석하지 않는다.
  과거27~98초 target 단절이 해결됐다는 의미도 아니다.
- 신호 보고 age와 bag 수신 시각의 추가 차이 최대0.104738초.
  두 교차로 모두 원본 영상·YOLO 로그·debug·신호·목표를 확보했다.
  녹색 미채택은 이번 시험에서 재현되지 않았다.
- 경로 횡오차 최대0.5643m, 감독 도구의 지속 이탈 알람0.
  **이탈 대신 직진 상태에서 장애물 충돌**로 끝났다.
- 충돌 직전 BEV는 OCCUPIED를 보고했고 대상에 대한 회피 후보도 있었다.
  따라서 단순한 전체 장애물 미검출로 설명되지 않는다. 실제 모든 객체의
  정답 목록을 확보한 평가는 아니므로 장애물 누락0건이라고 주장하지 않는다.

## 종료 후 확인한 원인과 최소 수정

목표: 장애물 뒤에 있는 다음 신호 정지점이 앞의 장애물 회피를 금지하지 않게 한다.
영향 파일: `states/avoid.py`, `test_avoid_signal_hold.py`.
구현: 기존 STATIC_OBSTACLE 대기 분기에 `lateral_hold_s < plan.nearest_s`
조건을 추가한다. 장애물 앞의 정지선 대기·적색 정지·속도/ROI/watchdog/
지도 검사·제어·터널 파라미터는 변경하지 않는다.

원인: 두 번째 정지선309.8735m를 지난 s≈309.881m에서
`SignalStopPolicy.lateral_hold_s()`는 다음 정지점430.9174m를 반환한다.
기존 STATIC_OBSTACLE 분기는 그 정지점이 장애물 앞에 있는지 검사하지 않았다.
그 결과 앞의 장애물(중앙 box 시작≈327.9m)보다 훨씬 뒤의 정지점까지
`lat_offset=0`, entry/chain 없음으로 유지했다. 후보 마커의 왼쪽+5m 선택은
실제 주행 곡선 발행과 다르다. 녹색 해제 이후에도 실제 target offset은 계속0이었다.

기존 일반 AVOID 분기는 `hold < plan.nearest_s`를 확인하지만,
STATIC_OBSTACLE 전용 분기에는 그 검사가 빠져 있었다. 새 재현 검사는
수정 전 **1 FAIL/6 PASS**, `lateral_started=False`로 실패한 로그를 보존했다.
이는 정지선 수치나 threshold를 바꿀 근거가 아니라 누락된 순서 조건을
복원할 근거다.

실제 기록580개 목표 시점의 연속 footprint·위치·신호 재생에서 기존 코드는
nonzero offset0회, 수정 후481회였다. 수정 후 처음으로 s≈309.881m에서
왼쪽3m 진입 곡선을 발행했고, 적색일 때는 v_max=0을 유지했다.
같은 기록의 녹색 해제 후에도 회피 곡선이 남는다.
기록 궤적을 따르는 재생은 수정 후 실제 궤적·무충돌·완주 증거가 아니다.

**수정 후 재계산 지연은 여전히 중대한 문제다.** 재생580틱의 처리366.30초,
개별 계산 최대7.728초(기존 대기 분기 최대0.081초)였다. 실시간20Hz나
target freshness를 충족했다는 결과가 아니다. 마지막28틱은 기존 직진
기록 궤적을 계속 따라가므로 planner hold이며, 이 결과로 수정 후에도 같은
충돌이 발생한다거나 회피가 성공한다고 예측할 수 없다. 곡선 생성 조건의
오류 해소와 새 곡선의 실시간 계산·실제 추종 검증을 분리한다.

추가 잔여 문제: BEV OCCUPIED와 제동 요청이 있었는데도 물리 충돌했다.
차체 앞 범퍼·후륜축·센서 원점·footprint 앞면과 실제 객체의 관계 및 제동
여유는 별도로 재현해야 한다. 이번 조건 수정만으로 충돌 방지가 검증됐다고
보고하지 않으며 차체 길이·TF·제동 상수를 추정해서 바꾸지 않는다.

## 재현 명령과 다음 우선순위

다음은 저장소 루트 기준이다. 각 output 폴더는 새 경로를 사용해야 한다.

```bash
# 실제 제어를 송신하므로 시작점 Reset·정지 및 주행 승인 범위에서 실행
bash ros1-port/docker/run.sh --receive-root --capture-control \
  python3 docker/audit_course.py \
  --output-dir docker/artifacts/fullcourse-fixed-20261021/run-NEW

# 이후 명령은 기록만 읽고 실제 제어를 송신하지 않음
bash ros1-port/docker/run.sh python3 tools/analyze_course_audit.py \
  --run docker/artifacts/fullcourse-fixed-20261021/run-02 \
  --output docker/artifacts/fullcourse-fixed-20261021/analysis-NEW
bash ros1-port/docker/run.sh python3 tools/replay_course_avoidance.py \
  --bag docker/artifacts/fullcourse-fixed-20261021/run-02/diagnostics_0.bag \
  --output docker/artifacts/fullcourse-fixed-20261021/replay-NEW
```

1. **P0:** 이번 수정으로 실행되는 회피 재계산의 병목과 최대7.728초 경계를
   먼저 프로파일링한다. 과거 지연 개선의 회피 회귀/롤백 이력이 있으므로
   geometry 검사·완전한 진입/복귀 곡선·인지 기억을 생략해 시간을 줄이지 않는다.
   실제 차체·제동 경계도 확인한 뒤 시작점부터 재시험한다.
2. **P0:** 수정 후 CP4 반경3m·회피 추종·충돌 여부 확인. 과거 CP4 누락은
   이번 미도달 시험으로 해결 판정할 수 없다.
3. **P1:** 과거 긴 planning tick/target 단절 및 오래된 판단 stamp 재현.
   이번 짧은 도달 구간에서 단절이 없었다는 사실과 구분한다.
4. **P1:** CP4 이후 모든 미션과 터널 지면 수정의 실제 연속 주행 검증.
   예전 별도 구간 시험을 이번 완주 결과에 합산하지 않는다.

대용량 bag/pcap/trace/영상은 Git 제외 artifacts에 유지한다. 기존 dated
증거 문서와 원본 비교 프로젝트는 수정하지 않았다. commit/push/merge 없음.

## 검사 결과와 제한

- 새 외부 감독 도구 판정: host 및 Noetic 환경에서 초기4개 통과;
  무충돌 슬롯 회귀 추가 후5개 통과. 실제 run-02의 충돌 종료와
  정지·Manual/P 피드백도 확인했다.
- 신호 대기/BEV/터널 직접 native 검사:61 PASS, 1 FAIL.
  실패는 RViz 검사에서 ROS2 `builtin_interfaces`가 없는 환경 경계이며,
  해당 검사는 아래 명시적 ROS 대역 전체 회귀에 다시 포함했다.
- 명시적 ROS 대역 전체 회귀: **926 PASS / 3 FAIL / skip0**, 340.58초.
  3개 모두 이식본 `tools/measure_highway.py`, `tools/replay_highway.py`
  부재에 따른 FileNotFoundError. 원본 검사 기준이나 기대값은 고치지 않았다.
  이 저장소의 `verification/run_differential.py`가 원본 tools 사본을 fixture로
  쓰는 방식에 맞춰 일회용 소스 사본에서 해당3개를 별도 재확인:
  **3 PASS**, 9.42초. 최초3 FAIL을 지우거나 전체 한 번에929 PASS로
  바꾸어 기록하지 않는다. `missing-tool-regression.log/json`을 참조한다.
- 새 도구와 변경 Python AST 구문 검사, 새 보고서 내부 링크 검사 PASS.
  기존 CRLF 보존 파일의 CR을 허용하는
  `git -c core.whitespace=cr-at-eol diff --check` PASS.
  원본 비교 프로젝트 tracked diff 없음.
- 종료 시 camera_bridge/yolo_signal_node의 기존 ExternalShutdownException
  traceback과 YOLO 로그 파일명 끝 개행도 재현됐다. 원본 로그를 보존하고
  이번 회피 수정에 관련 없는 종료/파일명 로직은 바꾸지 않았다.
- Windows 전용 회귀 wrapper, 원본/이식 전체 차등 비교, 수정 후 native
  drive=false/drive=true, 센서 원점·차체 제동 동등성은 이번 최소 수정 후
  미실행/미확인이다. 앞서 통과한 drive=false는 **주행 전 LiDAR 수정 상태**의
  결과이며 종료 후 회피 수정의 검증으로 바꾸어 집계하지 않는다.

전체 회귀 재현 명령(새 결과 파일명으로 변경):

```bash
bash ros1-port/docker/run.sh env \
  FOSCAR_TEST_TREE=/workspace/ros1-port \
  FOSCAR_TEST_REPORT=/workspace/ros1-port/docker/artifacts/fullcourse-fixed-20261021/regression-NEW.json \
  PYTHONPATH=/workspace/ros1-port/verification PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -p pytest_bridge -p no:cacheprovider -q \
  src/foscar_planning/test src/foscar_lidar/test \
  src/foscar_control/test src/foscar_camera/test \
  docker/test_course_audit_safety.py tools/test_replay_lidar_ground.py \
  tools/test_replay_lidar_sequence.py
```

이번 신규/변경 파일: 이 보고서, `docker/audit_course.py`,
`docker/course_audit_safety.py`, `docker/test_course_audit_safety.py`,
`tools/analyze_course_audit.py`, `tools/replay_course_avoidance.py`,
`src/foscar_planning/foscar_planning/states/avoid.py`,
`src/foscar_planning/test/test_avoid_signal_hold.py`.
작업 시작 당시 존재한 LiDAR 수정·문서·replay 도구는 보존했다.
근거44개(원본 bag/pcap 포함)의 SHA256을 새 `evidence-sha256.json`에 저장했다.
