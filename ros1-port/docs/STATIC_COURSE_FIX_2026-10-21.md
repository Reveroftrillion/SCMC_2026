# 정적 장애물 실제 통과 수정 — 2026-10-21

목표: 반복 실패하는 STATIC_OBSTACLE 구간을 신호 대기 후 충돌 없이 통과하고,
CP4 조건과 이후 기본 경로 복귀를 실제 MORAI에서 확인한다. 사용자 요청에
따라 제공 고정 코스/상자의 위치를 이용한 경로도 허용한다.

영향 후보: 정적 미션의 LiDAR 지면 판단, Avoid의 정적 계획 단계/계산,
필요시 고정 코스 경로와 그 검증 도구/검사. 다른 미션은 범위를 분리한다.
기존 수정 상태를 `docker/artifacts/static-course-20261021/incoming.patch`로
보존하고 `fix/static-course-20261021` 브랜치에서 진행한다.

방법: 직전 충돌의 연속 점군·위치·신호 및 과거 통과 구간을 재생하여
실제 상자와 오탐·도로 밖 시설물을 분리한다. 고정 상자 주변의 단순 경로를
검토하며 신호·실제 관측 장애물·차체 sweep·지도 검사는 유지한다.
원인 없는 ROI/threshold/watchdog 변경은 하지 않는다. 주행 중 소스를 동결한다.

검증: 재현 검사 → 연속 입력과 계산 시간/경로 안전성 → drive=false →
MORAI 실제 시험/기록/충돌 또는 장기 정지 종료 → 속도0/Manual/P 인계.
완료 기준은 실제 정적 구간 통과 및 CP4/복귀 결과이며 offline 성공만으로
완료 판정하지 않는다. 미도달 구간과 남은 실패는 별도 기록한다.

제공 `2026_molit_comp_sample_scene.json`의 objectList는 하나:
DataID40100001, UNIQUEID2, 위치(-60.610,-142.178), yaw -83.219°,
scale(2,3,2), moveSpeed0. 이 시나리오의 정답이며 모든 실행 환경에서
항상 동일하다고 가정하지 않는다. 라이브 객체/충돌/센서 기록과 대조한다.

## 실제 결과

**고정 코스 정적 장애물 우회·CP4 통과·기본 경로 복귀를 실제 MORAI에서 확인했다.**

- `drive-01`: 시작 s6.528→기록상401.039m. s400m 종료 조건 도달 후
  정상 제동·Manual/P 인계. 무개입 전체 코스 완주 시험은 아니다.
- 실제 회피 최대 왼쪽 offset **4.0041m**. CP4 최소 XY 거리 **1.09775m**로
  요구 반경3m 안을 통과했다. s354.098m에서 원래 경로 거리0.00252m,
  이후 STATIC_OBSTACLE 종료388.4m를 지나 CRUISE 복귀했다.
- 원본 CollisionData **7,534개 수신, 유효 객체 충돌0개**, 파싱 오류0.
  이탈 알람0, launch/master/recorder 종료코드 모두0.
- target 최대 발행 간격 **0.116133초**, 0.5초 초과0.
  정적 미션 내 follower `NO_TARGET`0행. 초기 준비 중11행은 별도로 보존한다.
- 종료 후 서로 다른 stamp의 Status31개/0.666초에서 mode1/gear1,
  속도 절댓값 최대0.009621m/s. 마지막 속도0.000006877m/s.
  정지 판정0.01m/s 안에서 **Manual/P 인계 확인**.
- 주행 전후 `src/` SHA256 manifest 동일. 주행 중 코드/설정 변경 없음.
- 최초 live 확인에서는 차량 Status만0이었고 위치/target을 만들지 못했다.
  사용자가 Competition Status/Ego Ctrl을 다시 연결한 뒤 사전 검증 통과:
  센서·위치·TF·target 정상, drive=false 제어 UDP0.

![실제 우회와 복귀 궤적](../docker/artifacts/static-course-20261021/analysis-drive-01/static-pass.png)

[전체 카메라 영상](../docker/artifacts/static-course-20261021/analysis-drive-01/camera.mp4)은
원본 JPEG를 관측 평균 FPS로 내보낸 편의 파일이다. 정확한 프레임 시각은
`camera-timestamps.csv`와 원본 bag을 사용한다. 원본 입력을 덮어쓰지 않았다.

### 이번 시험의 체크포인트·미션 범위

| 항목 | 결과 |
| --- | --- |
| START | 시작 s6.53m이므로 START 반경 통과 자체는 미검증 |
| CP1 | 최소0.4068m, 통과 |
| CP2 | 최소0.0804m, 통과 |
| CP3 | 최소0.0691m, 통과 |
| CP4 | 최소1.0978m, 통과 |
| SIGNAL_1 | 적색 정지69.90초 후 녹색 인식·재출발 |
| intersection_02 | 적색 정지61.35초 후 녹색 인식·재출발 |
| STATIC_OBSTACLE | 우회·복귀·미션 종료 확인 |
| CP5 이후·보행자·회전교차로·고속도로·터널·END | 이번 집중 시험에서는 미도달·미검증 |

거리는 로컬 odometry/후륜축 기준이며 공식 MORAI 채점 결과가 아니다.
일반 분석 JSON의 `max_cross_track_m=4.004`는 원래 global route와의 거리다.
계획한 +4m 우회를 그대로 포함하는 값이므로 이 값을 추종 오류4m 또는
위험 이탈로 해석하지 않는다. 위 그림의 목표/실제 offset과 지도 검사를 대조한다.

## 원인과 채택한 변경

### 1. 실제 정적 미션의 바닥·도색 오탐

직전 충돌 기록234개 원본 점군에서 세 대표 프레임의 generic plane은 모두
None이었다. 아스팔트 지지점은1,295/2,305/2,666개였고, 재질 기반 평면
절편은 약0.458/0.469/0.479m였다. 이 평면에서 높이±0.12m인 도색점이
기존 고정 기준에서는0.4m 이상으로 남았다(대표56/134/250점).

기존 터널 수정의 관측 아스팔트 지지 영역 내 상대 높이 계산을 **fresh
STATIC_OBSTACLE에도 적용**했다. generic 평면의 절편 guard, ROI, DBSCAN,
높이 threshold, 센서 TF는 변경하지 않았다. STATIC에서는 터널의 intensity0
제거·고정 slope 필터를 적용하지 않으므로 zero-return 물체도 유지한다.
미션 만료·intensity 부재·다른 미션의 기존 처리는 그대로다.

의도한 범위 확장이므로 기존 검사에서 STATIC을 '변경하지 않는 다른 미션'
목록에서 분리하고, 저상 zero-return 상자 보존·도색 제거·소스 bytes 불변·
ownership 만료 시 원래 높이 기준 복귀를 별도 검사로 명시했다. 기존 기록의
검사 기대값/JSON을 소급 변경하지 않았다.

같은 위치 s≈309.88m의 연속150프레임에서 footprint 객체는 모두1개였고,
주요 box는 s327.9m 부근의 실제 상자였다. 이전의 도색 조각을 여러 회피
단계로 만드는 조건이 사라졌다.

### 2. 이 코스에서만 사용하는 고정 우회

사용자가 고정 코스 하드코딩을 허용했다. 새 `static_course.py`와
`avoid.static_course: true`로 다음 사전 검증 곡선을 사용한다.

```text
진입: s310→322m, offset 0→왼쪽4m, quintic
유지: s322→333m, offset 왼쪽4m
복귀: s333→351m, offset 왼쪽4m→0, quintic
```

- 제공 box의 위치/yaw/scale(2×3)을 직사각형 계획 prior로 해석하여,
  기존 차체 앞3.8m·뒤0.835m·반폭+margin1.446m로 sweep을 검사했다.
  왼쪽3.5/3.75m는 이 prior와 충돌했고 +4m는 통과했다.
  오른쪽-2.5/-2.75/-3m 후보는 지도 차체 검사에서 탈락했다.
  이 prior를 실제 센서 관측으로 발행하거나 occupancy에 가짜 객체를 넣지 않는다.
- 정확한 정규화 route4406개 점 SHA256
  `c464e34356f76eda8d9901cdbdb7b88f324fab0bd0caab6bb69cf7526077e0ab`와
  전역 s offset을 확인한다. follower의 finish runout suffix는 허용한다.
  다른 경로나 다른 미션은 기존 플래너로 간다. 기본 flag는 false이며
  현재 이 코스용 behavior.yaml에서만 명시적으로 true다.
- 현재 적색 정지선 전에는 기존 직진 대기를 유지한다. 녹색이면 기존 신호
  정책이 해제하며, 장애물 뒤의 다음 신호 때문에 우회를 미루지 않는다.
- 지도 검증 실패·관측 장애물과 고정 곡선 충돌 시 v_max0.
  진입점을 이미 지난 위치에서 새로 시작하면 정지한다.
  실제 BEV OCCUPIED/UNKNOWN/stale 제동과 원래 신호 정책·watchdog는 유지한다.
- 전체 진입/복귀 chain을 안정적으로 발행하므로 매 틱 조합 탐색을 하지 않는다.
  원래 global route 파일·CP 좌표·ROS 메시지/토픽/포트는 바꾸지 않았다.

기존 일반 정적 플래너의 fallback에는 후보 offset 범위+완전히 회전한 차체의
최대 반경으로 도달 불가능한 먼 box만 계획 단계에서 제외하는 검사를 추가했다.
원시 관측/memory/BEV는 보존한다. 앞선 롤백 당시 도구/검사를 출처로 명시하고
새 검증 로그를 사용한다. 고정 코스에서는 이 fallback 탐색에 의존하지 않는다.

### 3. 지연·충돌·신호를 구분한 확인

- 지면 수정 후234개 연속 점군 재생:11.07초. 고정 곡선 포함 재생도234개/
  11.04초. 실제 기록의 직진 궤적을 고정하므로 후반 UNKNOWN 제약이 남는
  사실을 무충돌 성공이나 실패 예측으로 쓰지 않는다.
- 실제 통과 시험: 두 신호 모두 실제 녹색을 인식했고, 신호 토픽은
  intersection_01 red1484/green32, intersection_02 red1060/green10.
  원본 영상·YOLO JSONL·debug·신호·target을 함께 저장했다.
- 정적 구간에서의 긴 정지는 적색 대기였으며, 녹색 이후 NO_TARGET 반복
  정지 없이 우회했다. 신호 우회·가짜 녹색·timeout 증가는 사용하지 않았다.
- 충돌 채널을 실제 수신하면서 객체 충돌0을 확인한 **한 번의 실제 집중 시험**이다.
  여러 초기 위치·신호 위상·센서 preset·변경된 장애물 배치의 성공을 뜻하지 않는다.

## 검증과 재현

- 지면/정적 도달범위/신호/BEV/터널 회귀174 PASS.
- 최종 고정 코스/도달범위/신호/LiDAR 회귀124 PASS.
- 초기 새 지면 검사의 만료 후 paint cluster 개별 검출 기대는, 지면과
  DBSCAN으로 합쳐지는 fixture 때문에 실패했다. 만료 경계의 실제 계약인
  raw clearance 높이 복귀를 확인하도록 검사했다. 초기 로그를 보존했다.
- 새 고정 코스 검사 첫 실행은 데이터 경로 오류4 FAIL/119 PASS였고,
  경로를 고친 뒤123 PASS, 실제 follower runout 검사를 추가한 뒤124 PASS.
- 전체 패키지 회귀: **931 PASS / 1 FAIL / skip0**, 343.87초.
  실패는 고속도로 `test_planning_is_fast_enough_for_the_behavior_tick`의
  시간 조건으로, 평균44.15ms가 기존40ms 기준을 초과했다. 기대값·기준은
  변경하지 않았다. 정적 구간의 실제 target 최대 간격0.116초와는 별도
  검사이며, 부하 영향/기존 고속도로 성능 여부는 아직 재확인하지 않았다.
  `regression-final.log/json`에 실패를 보존한다.

2026-10-22 상태 확인: 시험·분석 컨테이너, ROS master 및909 수신 점유가
없고 MORAI 프로세스도 실행 중이지 않았다. 주행 시험은 위 종료 조건으로
정상 종료했으며, 전체 회귀도 종료됐다. 최종 보고 정리가 대화 종료 전에
전달되지 않았던 상태였다. 추가 주행은 시작하지 않았다.

저장소 루트에서 실행한다. output은 반드시 새 폴더로 바꾼다.

```bash
# 실제 제어 송신: 시작점 부근 정지·P, 센서/Status/제어 Connect 확인 후
bash ros1-port/docker/run.sh --receive-root --capture-control \
  python3 docker/verify_live.py --seconds 10 \
  --output docker/artifacts/static-course-20261021/preflight-NEW/live.json
bash ros1-port/docker/run.sh --receive-root --capture-control \
  python3 docker/audit_course.py --stop-at-s 400 --max-seconds 500 \
  --output-dir docker/artifacts/static-course-20261021/drive-NEW

# 제어 없는 원본 기록 분석
bash ros1-port/docker/run.sh python3 tools/analyze_course_audit.py \
  --run docker/artifacts/static-course-20261021/drive-01 \
  --output docker/artifacts/static-course-20261021/analysis-NEW
```

정적 통과 이후에는 시작점에서 전체 코스 무개입 시험을 다시 진행해 다음
미션 실패를 기록하는 것이 우선이다. 이번 결과로 터널 지면 수정·고속도로·
전체 완주를 통과 처리하지 않는다. 코스/상자 배치가 바뀌면 고정 곡선의
적용 범위와 안전성을 다시 확인해야 한다. 제동·차체 원점/TF의 일반적인
동등성도 이번 한 차례의 무충돌 통과만으로 확정하지 않는다.
