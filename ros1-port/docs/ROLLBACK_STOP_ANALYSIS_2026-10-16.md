# 2026-10-16 롤백 후 정지 및 수정본 회피 실패 분석

사용자 요청: 주행·녹화를 종료하고 첫 영상을 제출용으로 찾은 뒤, 롤백 상태와
정지 원인을 분석한다. 이번 작업에서는 주행 로직을 추가로 수정하거나 재실행하지 않았다.
브랜치는 `run/ros1-update-20261016`, 기준 HEAD는 `2d34dc2`다.

## 제출 파일과 종료 상태

첫 녹화에서 만든 제출 파일을 아래 폴더에 바이트 그대로 복사하고 파일 관리자로 열었다.

```text
/home/foscar/다운로드/CTRL_제출_20261016/CTRL.mp4
/home/foscar/다운로드/CTRL_제출_20261016/CTRL_sensor.json
```

영상은 465초(7분 45초), 1920×1080 H.264, 185,964,945바이트다.
첫 녹화 끝의 IDE/채팅 화면을 제외한 파일이며, 전체 프레임 디코딩 검사를 통과했다.
전체 미션 완주 영상은 아니다. 첫 실행의 최대 경로 진행거리는 약394.26m였다.
원본 제출 파일은 `docker/artifacts/midcheck-20261016/submission/`에 그대로 있다.

| 파일 | SHA256 |
| --- | --- |
| CTRL.mp4 | a7c20683818915f9222eebfd19e23359874aef90f575dcfeda5149bcfbdb2dce |
| CTRL_sensor.json | b59ac8166efdb936ae7059c1d6bdd1c5f0dfcb59667755a2ea6d201576fa9774 |

마지막 주행 launch에 SIGINT를 한 번 보내고 `Manual/P, brake released` 인계를 확인했다.
진단 bag과 화면 녹화도 정상 종료하고 마지막 MKV를 같은 실행 폴더의 MP4로 remux했다.
Docker 실행 컨테이너와 주행·녹화 프로세스가 남아 있지 않음을 확인했다.
MORAI와 WeatherTimeChanger는 종료하지 않았다.

## 롤백 확인

추적되는 `ros1-port/src`와 `ros1-port/docker`는 `git diff --exit-code`로 HEAD와 동일하다.
회피 수정, 시작점 보정 옵션 및 추가 테스트는 이미 되돌렸다.
수정 diff는 Git 제외 artifact의 `rolled-back-changes/changes.patch`에 보존했다.
원본 `2026foscar-competition/`은 변경하지 않았다. commit/push/merge도 하지 않았다.

실행 중이던 컨테이너에서 실제 import 경로와 소스 해시를 확인했다.
`avoid.py` 해시는 `4de47a3353be8842f25bebedc3b931a7c37256c46a9c9904d22a2569c49548e5`,
`core.py` 해시는 `b77e33e082215ff6195d817c72d4c7119628dccddfe827856fb90b8eef2e0440`이며
모두 HEAD 및 작업트리와 같다. 추가했던 시작점 helper도 없었다.
식별자 검사 `check_identifier_refactor.py`는 PASS였다.
따라서 이번 정지를 근거로 롤백이 누락됐다고 판단할 수 없다. 복원한 원본에도 문제가 재현됐다.

## 세 실행 비교

| 실행 폴더 | 코드 | target 최대 발행 공백 | follower NO_TARGET | 최대 진행 s |
| --- | --- | --- | --- | --- |
| drive-20261016-062727 | 원본, 첫 영상 | 73.608초 | 6,333 / 9,737행 | 394.26m |
| retake-20261016-160848 | 폐기한 수정본 | 1초 초과 공백 없음 | 13 / 3,047행 | 356.58m |
| rollback-retake-20261016-161848 | 원본으로 완전 복원 | 98.545초 | 3,702 / 7,065행 | 309.90m |

CSV는 약20Hz 기록이다. 행 수는 실패 횟수가 아니라 실패 상태의 샘플 수다.
발행 공백은 bag 수신 시각 차이로 계산했다. 서로 다른 실행의 교통신호·센서 입력까지
동일한 것은 아니므로 이 표를 성능 비교 시험으로 해석하지 않는다.

## 원본의 반복 정지: 회피 계산이 목표 발행을 막음

첫 bag을 제어 송신 없이 재생했을 때 멈춘 worker의 traceback은
`Avoid.obstacle_target → tunnel_slalom.plan → paved_moves → LaneMap.contains_intersection_road`
후보 탐색·차체 도로 검사에 있었다. 정적 장애물 근처의 먼 도로 시설물까지 단계로 묶으면서
탐색이 길어졌다. 자세한 재현 과정은 [당시 수정 기록](NO_TARGET_FIX_2026-10-16.md)에 있다.

마지막 원본 실행에서도 target 공백98.545초, 56.074초, 34.268초가 확인됐다.
정지 시점 behavior_node CPU 사용률은 약77%였고 위치·제어 쪽 프로세스는 계속 동작했다.
Follower는 `follower.py`에서 target 유효시간을 검사하며, 설정 `target_timeout=0.5`와
dropout 유예를 넘으면 `NO_TARGET`의 속도0 목표를 만든다.
따라서 목표가 오랫동안 안 오면 정지하고, 뒤늦게 목표가 도착하면 잠깐 움직이는 현상이 설명된다.
유효시간을 늘리는 방식은 오래된 계획으로 운전하게 하므로 해결로 간주하지 않는다.

추가로 시간 처리 결함이 확인됐다. `behavior_node.py`의 tick은 시작 시각을 `ctx.now`에
저장한 뒤 회피를 계산하고, 신호 freshness도 그 시각으로 판단한다. 하지만 출력 메시지에는
계산 완료 시각을 stamp로 넣는다. 마지막 원본 실행의 한 GO 결정은 실제 관측 후 약98.56초가
지났는데 `age_s=0.043`으로 보고됐다. 첫 영상에도 같은 차이가 약73.58초였다.
오래 걸린 계산 결과가 새 메시지처럼 발행되는 문제이며, 정지·재출발 판단을 불안정하게 만든다.

## 마지막 정지: 신호 인식 결과가 red로 유지됨

종료 직전 기록은 아래와 같다.

```text
rear_s = 309.88165m
signal_stop = STOP, intersection_02, red, age_s ≈ 0.06094
target = AVOID|SIGNAL_STOP, v_max = 0, stop_s = 312.8735m
lat_offset = -2.0m, 회피 chain 존재
```

후륜 위치와 앞축 정지선의 약3m 차이는 기존 좌표 정의다.
회피 chain이 있어도 신호 정지 제약이 합성되면 차량은 움직이지 않는다.
즉 마지막에는 회피 목표 자체가 없었던 것이 아니라 신호 정지 명령을 유지했다.

마지막 bag의 intersection_02 인식은 red1,420 / green3 / yellow32개였다.
이 수치는 bag에 기록된 토픽 메시지 수이며 전체 추론 프레임 수는 아니다.
사용자 화면에는 녹색이지만 모델이 쓰는 Camera-7 JPEG·검출 box·ROI와 전체 시뮬레이터 화면은
서로 다른 입력이다. **같은 시각의 원본 프레임→box→ROI→선택색을 대조하지 않아
녹색 미채택이 카메라 조건, 오검출, 다른 방향 신호 선택 중 무엇인지는 확정하지 못했다.**
긴 tick이 신호 콜백 처리를 지연시키는 문제와, 마지막에 새 red 관측이 들어온 사실을 구분해야 한다.
기존 카메라 해상도·pitch 및 LiDAR 장착값과 코드 좌표의 차이는 조사 후보로 남겨 두며
이번 정지의 확정 원인으로 단정하지 않는다.

## 폐기한 수정본: 지연 개선과 실제 회피 실패를 구분

수정본 재녹화에서는 1초 초과 target 공백이 없었다. 그러나 실제 주행은 실패했다.
CSV에서 s309.70→345.10m 구간은 CRUISE였고 경로 횡오차 최대값은 약0.237m였다.
시나리오 장애물은 경로상 약s327.5m에 있다. behavior 로그의 AVOID 진입은
이미 s344.6m, 약47km/h인 시점이었다. 사용자가 보고한 회피 미수행과 일치한다.

AVOID 진입은 `Avoid.check_entry`의 기억된 장애물·거리 조건과 소유권 조건에 달려 있다.
이번 기록만으로 장애물 입력 누락·필터·상태 전환 중 어느 경계에서 진입이 늦었는지는
확정하지 못했다. production 수정은 `obstacle_target` 단계 생성에 있었으므로
늦은 진입을 해당 필터의 직접 영향이라고 단정해서도 안 된다.

이후 s356.52m에서 멈춘 직접 원인은 신호가 아니라 아래 BEV guard였다.

```text
BEV STOP: UNOBSERVED
FSM: AVOID
cap 0.0 km/h | scan age 0.07s
conflict 0.00m along path
```

동시 target은 `AVOID`, 속도0, offset0, chain 빈 값이며 signal decision은 OFF였다.
라이다 scan은 최근 것이지만 차체 진행 영역을 관측되지 않은 영역으로 판정했다.
따라서 라이다 연결이 끊겼다고 해석할 수 없다. 관측 영역 구성·차체 검사 경계의 원인 분석이 필요하다.
후보 debug의 RIGHT CHOSEN -5m 문구는 최종 발행 target과 다르므로 실제 회피 수행의 증거가 아니다.

당시 입력 재생 및 단위 검사는 계산 지연 감소를 검증했지만, 실제 센서 입력에서의
회피 진입 시점·차량 횡이동·BEV guard까지 정상임을 증명하지 못했다.
그 상태로 수정본 주행을 진행한 판단은 부족했다. 수정본은 폐기하고 완전히 롤백했다.

## 근거와 후속 조사 범위

원시 자료는 Git 제외 `docker/artifacts/` 아래 보존한다.

- `no-target-fix-20261016/rollback-analysis/csv-summary.json`: 세 실행 요약·롤백 해시.
- 같은 폴더의 `bag-summary.json`, `analyze_runs.py`: target 공백·신호·시간 차이·guard 분석.
- `no-target-fix-20261016/rollback-stopped-topics.txt`: 정지 중 live 진단.
- `no-target-fix-20261016/live-replay.log`, `profile.log`: 원본 회피 계산 병목 재현.
- `midcheck-20261016/`의 세 실행 폴더: 원본 bag, follower CSV, 노드 로그 및 영상.

이번 완료 범위는 제출 파일 확보, 주행·녹화 종료, 롤백 검증 및 원인 분석이다.
주행 수정·검증 완료나 전체 미션 통과를 주장하지 않는다.
향후 수정 시에는 먼저 신호 동일 프레임 대조와 늦은 AVOID 진입의 입력 경계를 확보하고,
오프라인 계산 지연·시간 freshness·회피 시작·BEV 관측 판정을 각각 검증해야 한다.
실제 제어 송신을 포함한 추가 시험은 실행하지 않았다.


## 제출 이후 추가 실제 주행 점검

사용자의 별도 승인으로 원본 무개입 전체 코스 시험과 후속 구간 시험을 수행했다.
터널 진입 장기 정지와 CP4 반경 누락 등 새 결과는
[전체 코스 현장 점검](FULL_COURSE_AUDIT_2026-10-16.md)에 기록했다.
이 문서의 앞선 실행 기록·수치와 당시 결론은 그대로 보존한다.
