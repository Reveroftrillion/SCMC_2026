# 터널 돌발 대비 개발 — 2026-10-23

최신 진행은 [추가 실험 현황](#추가-실험-현황)에 있다.
1,167개 계산 조합·108개 관측 공백 조합을 비교했고, 반복 충돌의 entry 취소를 수정했다.
최종 회귀979개 중978PASS/기존1FAIL이며 수정 후 왼쪽·중앙·오른쪽 감독 주행을 각각 완주했다.
최종 상태는 [오른쪽 완료 요약](tunnel-contingency-20261023/batch-01/final-state-right-complete.json)을 따른다.
아래 초기 오프라인 결과·소스 manifest와 후속 수정/주행 버전은 구분한다.

## 목표와 작업 범위

main `52ebabe`에서 `feat/tunnel-contingency-20261023`을 분기했다.
반복 완주 이후의 우선 과제는 돌발 조건을 바꿔 정지·우회·재출발의 실패 경계를 확인하는 것이다.
계획·영향 파일·완료 기준은 [현재 개발 계획](DEVELOPMENT_WORKFLOW.md)에 기록했다.
이번 작업에서는 노드·UDP·MORAI 주행을 시작하지 않았다.

- 기능 수정: `src/foscar_planning/foscar_planning/bev_avoidance.py` 하나.
- 신규 검사: `src/foscar_planning/test/test_tunnel_contingency.py`.
- 신규 도구: [probe_tunnel_contingencies.py](../tools/probe_tunnel_contingencies.py),
  [도구 검사](../tools/test_tunnel_contingencies.py).
- 현재 문서·분류와 새 검증 요약을 추가한다. 과거 원본·검증·기대값·지도·모델은 보존한다.

## 재현한 문제와 최소 수정

기존 BEV 점유가 생성된 이후, 우회 후보용 box를 만들 때 그 점유 다각형의
꼭짓점 중 하나가 도로 안에 있어야 했다. 폭이 넓어 모든 꼭짓점이 양쪽 도로 밖에 있지만
내부는 도로를 가로막는 물체는 이 검사에서 빠졌다. AVOID 진입에 필요한 box가 사라져
터널 기본 속도 목표가 남았다. STATIC_OBSTACLE에서도 같은 필터를 사용한다.
별도로 지도 없음에서는 `deep_on_lanes`가 false여서 관측된 물체가 모두 빠졌다.

기존 꼭짓점 검사가 실패할 때 **이미 점유된 셀의 중심**도 같은 도로/깊이 검사에 넣는다.
새 점유를 채우거나 ROI를 늘리지 않는다. 지도가 없을 때는 관측된 물체를 보존하고,
기존 지도 기반 경로 검증이 우회를 거부해 정지하도록 한다.
도로 밖·얕은 가장자리 제외, 중앙 분리 공간, 차폭·여유·속도·기억·신선도 기준은 유지한다.
메시지·토픽·파라미터·YAML·launch·포트와 공개 인터페이스 변경은 없다.

새 회귀6개를 같은 입력/기대값으로 실행했다. Ubuntu 호스트 Python3.8에서
수정 전3 FAIL/3 PASS, 수정 후6 PASS였다. 넓은 물체의 TUNNEL/STATIC_OBSTACLE 누락과
지도 없음의 누락이 실패3개다. 도로 밖·얕은 가장자리·중앙 분리 공간의 제외3개는 전후 통과했다.

## 오프라인 시나리오와 결과

현재 지도 `link_set.json`, 경로 `global_path.txt`, `behavior.yaml`, `missions.yaml`,
checkpoint 기하를 읽어 실제 MissionSupervisor와 Behavior/AVOID/YIELD/TUNNEL을 호출한다.
합성 사각형은 해당 경로의 접선·법선으로 map 좌표에 배치한다. 객체 이벤트와 점유 관측을
함께 전달하며 시간은 명시적인 입력이다. 각 시나리오는 새 상태로 시작하고 내부 단계는 기억을 공유한다.
설정·소스 SHA256, 모든 입력·출력·단계별 판정을 새 디렉터리에 남긴다.

운영 Docker Python3.10에서 **17개 시나리오/33단계, 16 PASS/1 FAIL, 소스 변경0**이다.
호스트 Python3.8에서도 같은 행동 결과였다. 명령은 실패가 남으면 exit1을 반환한다.

| 시나리오 | 요구 행동 | 오프라인 결과 |
| --- | --- | --- |
| 빈 터널 GPS/DR·50km/h 입력 | 양의 속도 목표, 미션 상한 준수 | PASS |
| 터널 출구에서 GPS 복귀 입력 | TUNNEL→CRUISE 인계 | PASS |
| 지도 없는 상태의 관측 장애물 | 장애물 제약 보존·정지 | PASS |
| 전폭 차단→명시적인 비점유 근거 수신 | 정지→양의 속도 목표 복구 | PASS |
| 빈 관측 다음 가까운 전폭 물체 등장 | 다음 입력에서 정지 목표 | PASS |
| 기존 장애물 관측이 오래됨→새 관측 복구 | 정지→새 입력에 따라 복구 | PASS |
| 보행자 정지→도로 밖 이동→재진입 | 대기→출발 허용→재대기 | PASS |
| 정적 우회 도중 보행자 등장 | 기존 우회 곡선을 보존하며 정지 | PASS |
| 횡이동 DYNAMIC_UNKNOWN | YIELD 정지 | PASS |
| 중앙·우측 정적 장애물 | 양의 속도 목표와 우회 곡선 | PASS |
| 왼쪽 폭2m 물체, 지도상 통로 부족 | 정지 | PASS |
| 왼쪽 폭1m 물체 | 우회 요구 | **FAIL: planner hold** |
| 20/50km/h 입력 중 가까운 보행자 등장 | 다음 입력에서 YIELD 정지 목표 | PASS |

처음 만든13개 프로브에서는 지도 없음·전폭 물체·왼쪽 우회 요구가 실패했다.
같은 입력13개로 수정 후 지도 없음과 전폭 물체의 실패만 해소됐다.
왼쪽 폭2m 사례를 우회 가능하다고 가정했던 초기 명세는 지도 기하와 맞지 않았다.
이 초기 결과는 로컬 `.work/tunnel-contingency-20261023-02/`와 `-03/`에 그대로 보존했다.
최종 시험표에는 폭2m 물체의 정지 조건과, 폭1m 물체에 대한 별도 우회 요구를 분리했다.
후자의 실패를 숨기거나 여유 폭·지도·기존 기대값을 조정하지 않았다.

폭1m 실패에서 실제 점유 box의 횡 범위는 약 `[+0.180, +1.833]m`, 차량 반폭과
마진은 `1.746m`다. box 충돌을 피할 수 있는 후보는 오른쪽 `d≤-1.75m` 또는 왼쪽
`d≥+3.75m`였지만, 해당 구간의 지도 포함 검사를 통과한 후보는0개였다.
관측 셀과 차량 여유를 유지한 현재 지도 조건에서의 정지다.
실제 터널 벽 관측으로 통로가 더 넓게 확인될지는 MORAI에서 검증해야 한다.
이 프로브는 벽 관측/WallMemory를 합성하지 않으며 지도만 제공한다.

[공유 결과](tunnel-contingency-20261023/summary.json) ·
[시나리오 전체 입력·출력](tunnel-contingency-20261023/cases.json) ·
[소스·설정 SHA256](tunnel-contingency-20261023/sources.json).

## 재현 명령

저장소 루트에서 실행한다. 출력은 실행마다 새 경로를 사용한다.

```bash
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 \
  python3 tools/probe_tunnel_contingencies.py \
  --output verification/.work/tunnel-contingency-$(date +%Y%m%d-%H%M%S)
```

위 전체 실행은 알려진 `static_left_narrow` 실패를 포함해 exit1이다.
한 조건만 재현하려면 `--case fully_blocked_then_cleared` 같은 인자를 추가한다.
기존 출력 경로를 사용하면 덮어쓰지 않고 실패한다.

```bash
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 \
  python3 -m pytest -q -p no:cacheprovider tools/test_tunnel_contingencies.py
```

## MORAI에서 다음에 확인할 순서

현재 수정본으로 실제 주행은 미실행이다. 아래는 준비할 시험표이며 성공 기록이 아니다.
한 번에 하나의 배치/출현 시점만 바꾸고 시작점 Reset·Manual/P·채널 연결,
수신전용 점검 후 별도 주행 단계로 진행한다. 실행 방법은 [ROS1 README](../README.md)를 따른다.
주행 중 소스는 고정하고 raw LiDAR·기본 bag·follower·영상과 별도 원본 충돌 판정을 남긴다.

| 순서 | 바꿀 조건 | 확인할 결과·필요 근거 |
| --- | --- | --- |
| 1 | 기존과 같은 배치 | 수정으로 정적 구간/터널/END·정지에 회귀가 없는지. 공식 판정과 기록 분리 |
| 2 | 좌·중앙·우측 정적 물체 | raw 외곽·벽 관측·지도·차체 sweep 비교. 통로가 있으면 우회, 없으면 정지 |
| 3 | 전폭 차단·일부 통로 재개방 | 차단 중 정지 유지, 실제 새 비점유 근거 이후 출발. 물체 삭제만으로 성공 판단 금지 |
| 4 | 보행자/동물 갑작스러운 등장·가림 | 실제 인지 시각·분류·거리·속도·제동·충돌을 함께 확인 |
| 5 | 도로 밖 이동 후 재진입 | 재출발과 재정지, 정적 우회 곡선 유지, 다른 물체 제약 보존 |
| 6 | 입력 손실·재연결·GPS 복귀 | 센서/clock/미션 소스별 시각과 정지·복구. 별도 시험으로 분리 |

오프라인의 `v_max=0`은 제동 명령 근거일 뿐 실제 정지거리·무충돌을 입증하지 않는다.
특히 50km/h에서 가까운 물체가 나타나는 경우 물리적으로 정지할 수 있는지 별도 실증이 필요하다.
GPS/DR는 주어진 문자열을 전달한 것이며 실제 위치추정·TF 드리프트를 실행하지 않았다.
비점유 해제는 올바른 id/last_seen 증거를 직접 넣어 소비 경로를 확인한 것으로,
LiDAR 노드가 실제로 그 근거를 생성하는지는 별도다.

## 검사와 남은 범위

- 신규 도구 검사5 PASS. 잘못된 무제한 주행·곡선 취소를 실패 처리하고 기존 출력 보존을 확인했다.
- 격리 소스 사본·Docker Python3.10·명시적 ROS 계산 대역의 전체959개 검사에서
  955 PASS/4 FAIL/skip0, 경고10, 343.62초였다. 새 회귀6개도 이 실행에 포함됐다.
  실패3개는 검사 사본에 원본 보조 도구 `measure_highway.py`, `replay_highway.py`가 없어 발생했다.
  새 사본에 보존 원본의 tools를 복사해 해당3개만 재검사한 결과3 PASS/FAIL0/skip0였다.
  최초 전체 결과는 수정하지 않는다. 이를 단일 전체958 PASS 실행으로 표현하지 않는다.
- 남은 알고리즘 실패1개는 기존 `test_tunnel_intensity.py:75`의 ROUNDABOUT 고정 높이 정책 충돌이다.
  이번 변경으로 새로 생긴 실패가 아니며 기대값은 유지했다.
- [검사 범위 요약](tunnel-contingency-20261023/checks.json),
  [최초 전체 결과](tunnel-contingency-20261023/algorithm-initial.json),
  [누락 fixture 보완 후3개 재검사](tunnel-contingency-20261023/fixture-rerun.json).
- [보존 검사](tunnel-contingency-20261023/preservation.json): 검사 사본의 src345개와 현재 파일이
  모두 같고 비교 원본530개 Git blob 변경0이다. 실험/검증 JSON·기존 테스트 기대값은 바꾸지 않았다.
- [동일 초기 입력의 수정 전후](tunnel-contingency-20261023/regression-comparison.json)는
  지도 없음/전폭 차단만 비교한다. 범위와 실행별 입력을 구분한다.
- 실제 native ROS·executor·UDP·MORAI·Windows·catkin 검사는 미실행이다.
- 마감 구문4개·공유 JSON9개·현재3문서 내부 링크210개를 확인했다.
  일반 `git diff --check`는 기존 CRLF를 유지한 production 파일의 추가13줄에서 CR을
  trailing whitespace로 보고한다. `git -c core.whitespace=cr-at-eol diff --check`는 통과했다.
  [마감 정적 검사](tunnel-contingency-20261023/static-checks.json)를 참고한다.
- 공식 차선·신호·충돌 감점, 속도 예외 경계는 여전히 별도 과제다.
- 이번 변경은 작업 브랜치에만 있으며 commit/push/main 추가 병합을 수행하지 않았다.

## 실환경 후속 시험과 복귀 후보 수정

사용자가 MORAI를 켰으며 필요한 실험과 개발을 계속하라고 명시적으로 지시했다.
로컬 자료는 `docker/artifacts/tunnel-contingency-live-20261023-162509/`다.
기존 수동 실행은 FINISH 후에도 켜져 있어 임시 ROS 로그를 보존한 뒤 SIGINT로 정상 종료했다.
기존 bag은 정상적으로 닫혔고 별도 Status 수신에서 정지·Manual/P를 확인했다.
사용자에게 새 시작점 Reset·Manual/P·채널 준비를 확인받았다.

### run-01: 넓은 점유 누락 수정본의 기존 배치 회귀

- drive=false 25초 점검 PASS: 센서·TF·target, 제어UDP9093 송신0, launch exit0.
  시작점 거리1.101m, 고정 소스345개 변경0이었다.
- 기존 감독의 충돌·이탈·입력 유실·무진행 기준을 유지해 주행했다.
  START·제공 CP1~10/12~15·터널·종료 지점을 통과하고 `route_end_stopped`로 종료했다.
  203.436초는 준비·신호 대기·종료 인계를 포함하며 공식 주행시간이 아니다.
- 원본 CollisionData8,540패킷을 다시 decode해 유효 충돌0·decode 오류0을 확인했다.
- 종료 후 fresh Status30샘플/0.641초의 정지·Manual/P, 최대 절대속도0.001265m/s.
  자식 master/recorder/launch exit0, callback 오류0, 검사 소스 변경0이다.
- bag의 target 최대 간격0.130117초, 1초 초과0. GPS/DR가 모두 관측됐다.
  최고 속도124.730km/h, 최대 횡오차3.257m다. 차선/속도 규정의 공식 감점을 검증한 결과가 아니다.
- 이번 구간에는 별도로 배치한 왼쪽 합성 장애물이 없다. 아래 복귀 수정의 실제 검증으로
  이 결과를 소급하지 않는다.

### 실제 벽 관측으로 남은 실패 경계 좁히기

새 [벽 관측 비교 도구](../tools/probe_tunnel_wall_clearance.py)는 종료된 bag에서
TUNNEL 벽 관측을 시간순으로 넣어 WallMemory를 복원한다. s1880 이상 첫 유효 스캔에서
멈췄으며 실제 마지막 관측 위치는 s1880.842m, 누적45스캔·corridor1개다.
그 이후 관측을 가져오지 않는다. 동일한 s1880 합성 자세·장애물로 지도 단독/벽 포함을 비교한다.
실제 callback 순서나 현재 raw 인지를 완전히 재생한 결과가 아니다.

지도 단독에서는 왼쪽 장애물을 피할 수 있는 상수 offset 후보가 없었다.
벽을 포함하면 후보는 생겼지만, 선호하는 늦은 복귀가 도로 검사를 거부당한 뒤
더 이른 충돌 비접촉 복귀를 검사하지 않아 전체 체인을 버렸다.
`tunnel_simple.way_back`이 우선 collision-clear 복귀 하나를 반환하고, 그 후 `road_ok`가
실패하면 해당 side 전체를 버리는 흐름이다.

신규 `test_tunnel_return_pavement.py`에서 같은 입력/기대값으로1 FAIL/3 PASS를 재현했다.
우회 차로가 s70에서 끝나고 선호 복귀가 s80인 경우, 실제 가능한 더 이른 복귀가 있어도
전체 경로를 거부했다. 모든 복귀가 거부될 때/진입이 거부될 때의 정지와 정상 늦은 복귀는
수정 전에도 통과한 보존 조건이다.

`tunnel_simple.py`에서 기존 선호 경로의 도로 검사가 실패한 경우에만 진입 검사를 유지하고,
기존 복귀 후보 중 충돌·도로/BEV 검사를 모두 통과하는 것을 재검색하도록 최소 수정했다.
정상 선택 경로·속도·차체 여유·후보 간격·ROI·기억·watchdog는 바꾸지 않았다.
새4개와 기존 단순 터널4개는8 PASS다.

같은 벽45스캔·같은 합성 입력의 수정 전후 비교에서 왼쪽은 정지→우회/복귀 가능으로 바뀌었다.
offset+3.75m, 진입s1880/길이17.932m, 복귀s1907.932/길이29.277m다.
중앙·우측 target 전체는 전후 동일하며 비교한 production source 차이는 `tunnel_simple.py` 하나다.
지도 단독의 왼쪽 실패는 그대로 남는다. 지도/벽을 늘리거나 기존 기대값을 바꿔 통과시키지 않았다.

새 도구 검사3 PASS: TUNNEL 이전/무효 관측 제외, 선택한 스캔 이후 관측 미사용,
벽 없는 입력에서 통로 생성 금지, 활성 bag 거부를 확인했다.
이는 실제 변경 장애물이 벽을 가리거나 raw 인지·제동·추종에 미치는 효과의 증거는 아니다.

### run-02 준비

추가 수정 후 소스346개를 새 manifest에 고정했다. 사용자에게 시작점 Reset·Manual/P·채널 준비를
다시 확인받았고 drive=false 25초 점검 PASS·제어UDP0·시작점 거리1.730m를 확인했다.
전체 회귀 검사 후에만 새 감독 주행을 진행한다. 실제 결과는 수행 후 추가한다.

### run-02 결과와 현재 버전

추가 수정 후 격리 전체963개는 **962 PASS/기존 로터리 정책 충돌1 FAIL/skip0**, 경고10,
355.26초였다. 원본 도구를 포함한 새 사본을 사용했으며 검사 소스 변경0이다.
[전체 결과](tunnel-contingency-20261023/live/algorithm-after-return.json).

그 후 같은 기본 배치에서 두 번째 감독 주행을 실행했다.
START·제공 CP 전체·터널·종료 지점을 통과해 `route_end_stopped`로 종료했고,
204.805초는 준비/신호/인계를 포함한 감독 시간이다. 원본 충돌8,594패킷의 유효0·decode 오류0,
정지·Manual/P30샘플/0.646초·최대 절대속도0.001229m/s, callback 오류0·자식 exit0,
검사 소스346개 변경0이다. target 최대 간격0.142053초·1초 초과0,
최고 속도98.051km/h·최대 횡오차3.235m, GPS/DR 관측을 확인했다.
공식 점수·감점·모든 배치 통과나 새 왼쪽 물체의 실제 우회 성공으로 확대하지 않는다.

- [run-01 종료](tunnel-contingency-20261023/live/run-01-summary.json) ·
  [원본 충돌 재검](tunnel-contingency-20261023/live/run-01-collision.json) ·
  [bag 분석](tunnel-contingency-20261023/live/run-01-analysis.json).
- [run-02 종료](tunnel-contingency-20261023/live/run-02-summary.json) ·
  [원본 충돌 재검](tunnel-contingency-20261023/live/run-02-collision.json) ·
  [bag 분석](tunnel-contingency-20261023/live/run-02-analysis.json).
- [동일 벽/입력·소스 차이 비교](tunnel-contingency-20261023/live/wall-comparison.json) ·
  [수정 전](tunnel-contingency-20261023/live/wall-before.json) ·
  [수정 후](tunnel-contingency-20261023/live/wall-after.json).

### 실제 돌발 배치 준비

실행 중 MORAI의 실행 파일 경로와 같은 Launcher의 SaveFile에서
`2026_molit_comp_sample_scene.json`을 찾았다. 파일 SHA256은
`c9ab17cef9b07dd3a7d4a2f56c6cc39f89f07cadccce8edce3d579e891237c85`다.
이 파일의 정적 물체는 s327.52/d+0.75에1개, 보행자는 s458.07/d-3.54에1개이며
터널 정적 물체는 없다. 파일 내용만으로 현재 GUI에 로드된 상태를 독립적으로 확정하지 않는다.
따라서 반복 완주와 터널 돌발 배치의 검증을 구분해야 한다.

[장면 생성 도구](../tools/prepare_tunnel_scene.py)로 기존 모든 actor/ego/신호/spawn/센서 항목을
보존하고 기존 static asset `DataID40100001` 복제1개만 추가했다. 생성 도구5개 검사를 통과했다.
새 UNIQUEID39, s1905/d+1, map pose약[-2.371,-506.936,28.400], yaw-151.547도,
scale[3,1,1]이다. scale은 물리적인 미터 단위 크기를 보장하지 않으며 실제 LiDAR로 확인한다.
높이는 같은 지도의 가까운 선분에서 보간했으므로 로드 후 접지/충돌 형상을 확인해야 한다.

원본을 그대로 두고 Launcher SaveFile의 같은 map 폴더에 `CTRL_tunnel_left_box.json`을
새 이름으로 추가했다. 원본 byte 보존, 생성본과 설치본 SHA256 일치, 기존 장면 항목 불변을 확인했다.
[생성 근거](tunnel-contingency-20261023/live/scene-variant.json).
사용자에게 Scenario Load와 시작점/Manual/P/채널 준비를 요청했다.
생성·설치는 로드 성공이나 실제 물체 인지/우회 성공을 뜻하지 않는다.

```bash
# 이 PC의 저장소 루트. 호스트 Python3/numpy를 사용하며 생성만 한다.
python3 -B ros1-port/tools/prepare_tunnel_scene.py \
  --scene '/home/foscar/다운로드/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/2026_molit_comp_sample_scene.json' \
  --output ros1-port/docker/artifacts/tunnel-scene-$(date +%Y%m%d-%H%M%S)

# 종료된 run-01 bag의 같은 벽 관측으로 현재 복귀 후보를 다시 검사한다.
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 \
  python3 tools/probe_tunnel_wall_clearance.py \
  --bag docker/artifacts/tunnel-contingency-live-20261023-162509/run-01/diagnostics_0.bag \
  --output docker/artifacts/wall-probe-$(date +%Y%m%d-%H%M%S)
```

다음 실험은 이 장면 로드/연결 확인→수신전용 점검→감독 주행 순서다.
현재 추가 수정 소스는 고정하며 실패 시 입력부터 분석한다. commit/push/main 추가 병합은 하지 않았다.

마감 현재 이 장면의 실제 로드/주행은 미실행이며 사용자 준비 응답을 기다린다.
기본 장면 완주2회를 새 물체 배치 성공으로 합산하지 않는다.

### run-03: CTRL_tunnel_left_box 실제 시험

이후 사용자가 새 장면 로드·시작점·Manual/P·채널 준비 완료를 확인했다.
고정 소스346개와 설치 장면 SHA256을 다시 확인했다. preflight-03의25초 실제 수신은
센서/TF/target PASS·제어UDP0·launch exit0·START 거리1.963m였다.
기존 중단/인계 기준을 유지한 별도 run-03 감독 주행을 시작했다.
아래 후속 결과가 추가되기 전까지 이 시험의 통과를 주장하지 않는다.

[변형 장면 분석 도구](../tools/analyze_tunnel_variant.py)는 종료된 bag에서 추가 물체 위치의
관측 후보·target·실제 횡이동·GPS/DR·접근 카메라 원본을 새 폴더에 추출한다.
위치4m 이내/관측 높이0.2m 이상은 분석용 후보 조건이며 production ROI나 threshold를 바꾸지 않는다.
위치 근접만으로 actor 고유 ID를 확정하거나 가려진 물체 크기·차체 여유를 실측했다고 표현하지 않는다.

```bash
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 \
  python3 tools/analyze_tunnel_variant.py \
  --run docker/artifacts/tunnel-contingency-live-20261023-162509/run-03 \
  --scene-manifest docker/artifacts/tunnel-contingency-live-20261023-162509/scenes/manifest.json \
  --output docker/artifacts/tunnel-variant-analysis-$(date +%Y%m%d-%H%M%S)
```

end.json과 닫힌 bag이 있는 실행만 분석하며 기존 출력 경로는 덮어쓰지 않는다.

#### run-03 결과: 코스 종료, 추가 물체 시험은 성립하지 않음

378.888초 뒤 `route_end_stopped`로 종료했고 제공 CP 전체를 지나갔다.
원본 충돌 UDP15,186패킷에서 유효 충돌0·해석 오류0이었다. 정지·Manual/P는32표본/0.669초,
최대 절대 속도0.001260m/s로 확인했고 자식 종료0·callback 오류0·생산 소스346개 불변이었다.
이는 공식 점수나 follower FINISH 판정으로 확대하지 않는다.

그러나 추가 위치 주변 관측 후보0개, target offset은 전부0이었다. 접근 s1885 카메라에
추가 상자가 보이지 않았고 같은 시각의 원본 LiDAR를 지도 좌표로 옮겼을 때 예정 위치 반경2m는
59점 모두 바닥 높이(차체 좌표 z0.467~0.564m)였다. 따라서 이 실행을 **상자 우회 성공으로
집계하지 않는다**. 인지·우회 임계값을 바꾸지 않고 장면 로드부터 확인했다.

근거: [종료](tunnel-contingency-20261023/live/run-03-summary.json),
[원본 충돌 재해석](tunnel-contingency-20261023/live/run-03-collision.json),
[터널 관측·횡이동](tunnel-contingency-20261023/live/run-03-tunnel-analysis.json),
[원본 점군 주변 높이](tunnel-contingency-20261023/live/run-03-raw-at-actor.json).
원본 bag/카메라/점군/전체 분석은 같은 artifacts의 run-03 관련 새 디렉터리에 보존했다.

#### MORAI GUI로 장면 로드 실패 경계 확인

사용자가 물체 목록 확인 방법을 모른다고 답해 정지 상태에서 MORAI 창에 직접 접근했다.
확인 경로는 **Edit → Scenario → Scenario Edit Mode(F2) → 왼쪽 아래 Object Setting의
장애물 아이콘 → 선택 목록**이다. 위쪽 Spawn Objects List는 생성 가능한 모델 목록이다.
편집 모드는 Pause 상태이며 이 점검 중 차량 주행/ROS 제어 송신은 실행하지 않았다.

Unity Player.log에는 `CTRL_tunnel_left_box.json` 로드 완료가 기록되어 있었다.
같은 파일을 다시 로드하고 다른 이름으로 native 저장해도 objectList에는 기존2번만 남았다.
로드 완료 메시지나 JSON에 actor가 있다는 사실만으로 실제 생성을 판정하면 안 된다.
새 파일과 출력은 보존했고 원본 sample SHA는 변하지 않았다.

| 진단 입력 | GUI의 장애물 목록 | 판단 |
| --- | --- | --- |
| 최초 ID39, 추가 경로 등록 없음 | 2 | 추가 물체 누락; native 저장도 동일 |
| ID39 + 빈 ObstaclePath - 39 등록 | 2 | 경로 등록 하나만으로 해결되지 않음 |
| ID16, 등록 없음, 이전16번이 남아 있던 상태 | 2,16 | 기존 물체 상태가 결과에 영향 |
| ID39 배열 순서만 뒤집음 | 2 | 순서 변경으로 해결되지 않음; 이전16번 제거 |
| 16번 없는 상태에서 ID16, 등록 없음 | 2 | ID 변경만으로 재생성되지 않음 |
| MORAI 직접 생성 CargoBox16의 native 저장본 | 2,16 | native 형식으로 새 물체 복원 가능 |
| ID16 + 빈 ObstaclePath - 16 등록 | 2,16 | 16번 없는 상태에서도 생성; 터널 좌표 native 저장 확인 |

마지막 native 저장의16번은 x=-2.371, y=-506.936, z=28.408, scale=(3,1,1)이었다.
scale은 실측 크기가 아니며 높이·충돌체·센서 가시성은 후속 시험 대상이다.
ID39 미생성에 관한 MORAI 내부 ID 범위/할당 구현은 확인하지 못했다. 16번 성공을 모든 ID의
지원 보장으로 일반화하지 않는다. [비교 요약](tunnel-contingency-20261023/live/scene-native-load-comparison.json).

`prepare_tunnel_scene.py`에 명시적 `--actor-id` 필수 CLI 인자, ID 충돌 검사와
waypoint mode actor의 빈 ObstaclePath 등록을 추가했다. 기존 순수 build 호출의 자동 ID 선택은
호환성을 위해 유지하나 CLI는 max+1을 native 검증된 ID로 간주하지 않는다.
기존5검사 기대값은 유지했고 신규9검사는 수정 전 실패→수정 후 총14 PASS다.
새 생성 결과는 실제 로드한 `CTRL_tunnel_left_box_registered16.json`과 JSON 구조가 동일하다.
기존 actor/ego/신호/네트워크/경로는 보존하고 actor1개와 그 빈 경로 등록1개만 추가했다.
[새 생성 manifest](tunnel-contingency-20261023/live/scene-registered16.json).

```bash
# 현재 MORAI에서 검증한16번을 사용한다. 다른 source에서는 ID 충돌/native 로드를 다시 확인한다.
PYTHONDONTWRITEBYTECODE=1 python3 ros1-port/tools/prepare_tunnel_scene.py \
  --scene ros1-port/docker/artifacts/tunnel-contingency-live-20261023-162509/scenes/original-scene.json \
  --output ros1-port/docker/artifacts/tunnel-scene-registered-$(date +%Y%m%d-%H%M%S) \
  --actor-id 16
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  ros1-port/tools/test_prepare_tunnel_scene.py
```

현재 GUI는 `CTRL_tunnel_left_box_registered16`/Pause이며 장면 로드로 채널이 끊겼다.
사용자에게 같은 장면 유지·시작점 Reset·Manual/P·채널 재연결을 요청했다.
다음은 수신전용 점검 후 실제 센서 상자 관측·우회/복귀·충돌·정지 인계 확인이다.
아직 이 수정 장면의 실제 우회 통과를 주장하지 않는다. 추가 push/main 병합은 하지 않았다.

#### run-04 준비·시작

사용자가 현재 장면 유지·시작점 Reset·Manual/P·재연결 준비 완료를 확인했다.
추가 source 변화0/346과 설치 장면 SHA를 대조했다. 첫 preflight-04는 실행 명령의
ROS_LOG_DIR 누락으로 master 이전에 exit1했으며 제어UDP0이었다. 기록을 보존하고
새 preflight-04-retry1에 ROS_LOG_DIR를 지정했다. 25초 수신/TF/target PASS,
제어UDP0·drive=false·launch exit0·s2.755m였다. 새 run-04 감독 주행을 시작했다.
이전 결과 경로/감독 기준/생산 소스는 바꾸지 않았다.

#### run-04 결과: 실제16번 충돌과 원인 재현

run-04는318.945초/s1901.869에서 `collision_packet_alarm`으로 중단됐다.
원본 충돌12,833패킷을 재해석해 유효9개·해석 오류0·상대 ID16을 확인했다.
정지·Manual/P31표본/0.669초, 최종 속도0.00000979m/s, 자식 모두 exit0,
callback 오류0·생산 source346개 불변으로 종료했다. 충돌 이전 제공 CP1~10·12~14는 통과했고
터널 뒤/END는 미통과다. [종료](tunnel-contingency-20261023/live/run-04-summary.json),
[충돌](tunnel-contingency-20261023/live/run-04-collision.json),
[첫 경보](tunnel-contingency-20261023/live/run-04-collision-trigger.json),
[관측 분석](tunnel-contingency-20261023/live/run-04-tunnel-analysis.json).

이번 접근 카메라에는 낮은 상자가 보인다. s1885/예정 위치 반경2m 원본60점은 바닥뿐이었지만,
s1890/1896 원본에는 약0.4m 높이와 반사도44~61의 상자 관측이 있었다.
단일 프레임 재입력은 해당 두 시각에 검출1개씩을 만들었다. 따라서 센서의 영구적 미관측으로
단정하지 않고, 동일 raw64프레임과 선행 pose/속도 이력으로 검출/추적을 재생했다.

- 수정 전: 상자 주변 검출3프레임, 외곽 발행0프레임. 검출이 간헐적이라 확인 점수3을 얻지 못했다.
- 지면 분리 후보만 적용: 검출4프레임, 발행0프레임. 밝고 낮은 상자가 페인트 밴드에서 빠지는
  경계 하나는 고쳤지만 추적 발행 누락은 남았다. 이 후보를 단독으로 주행하지 않았다.
- 최종 후보: 검출4/발행4프레임, 첫 외곽 s1881.898(예정 중심 약23.1m 전).
  의미상 확정 이벤트의 확인 프레임3은 유지한다. 없는 스캔을 임의로 본 것으로 만들지 않는다.

[동일 원본 비교](tunnel-contingency-20261023/live/lidar-raw-comparison.json).
카메라·원본·각 후보의 전체 결과·단일 프레임 진단·재현 스크립트는 run-04 artifacts의 새 경로에 있다.
재생은 ROS doubles의 계산/추적 검증이며 middleware/TF callback 위상·제동/조향 동역학 검증이 아니다.

#### LiDAR 최소 수정과 검증

영향: `foscar_lidar/obstacle_node.py`, 신규 `test_tunnel_raised_paint_band.py`,
`test_tunnel_first_outline.py`, [시퀀스 도구](../tools/probe_tunnel_lidar_sequence.py), 현재 계획/문서/요약.

1. 유효 지면 평면이 실제 적용된 영역에서는 기존 지면 높이/물체 크기 검사로 밝은 상승 물체를
   구분한다. 지면 검증 영역 밖과 지면 평면 없음에는 기존 낮은 페인트 제거를 유지한다.
   원본 point cloud와 raw clearance는 유지한다. ROI·기존 수치 임계값은 바꾸지 않았다.
2. fresh TUNNEL의 실제 현재 도로 외곽은 기존 고속도로 처리처럼 첫 관측부터 전달한다.
   구조물/도로 밖 검사는 유지한다. 확정 이벤트·의미 분류의 확인 프레임3과 기존 정적 보존 기준은
   바꾸지 않았다. 한 번 본 외곽을 새 미관측 기억으로 연장하지 않는다.

신규 회귀는 페인트2 FAIL/2 PASS→4 PASS, 첫 외곽1 FAIL/4 PASS→5 PASS로 재현했다.
합계9 PASS. 격리 전체972개는971 PASS/기존 로터리 정책 기대 불일치1 FAIL/skip0/경고10,
350.93초·사본 source348개 변화0이었다. 실패는 기존 `test_tunnel_intensity.py:75`의 ROUNDABOUT
지면 기대이며 첫 터널 검출 assertion이 아니다. 기대값·과거 결과는 변경하지 않았다.
[전체 요약](tunnel-contingency-20261023/live/algorithm-after-lidar.json).

원본의 모든 새 외곽과 선행 벽/pose/속도를 순수 계획기에 넣어 추가 이벤트 없이 반응을 비교했다.
상자 전51프레임의 속도 상한/횡경로는 동일하지만8프레임은 상태가 TUNNEL→AVOID로,
유한한 먼 정지 목표가 추가되어 전체 target의 완전 동일성은 성립하지 않는다.
상자 관측을 받으면 감속 및 이후 정지 목표가 생긴다. 주어진 원본 궤적으로 재생하므로
실제 제동 후 달라질 관측·위치·통과를 예측/검증했다고 확대하지 않는다.

```bash
# native bag -> 새 raw/pose/속도 입력 폴더. ROS 통신이나 제어를 시작하지 않는다.
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 \
  python3 tools/probe_tunnel_lidar_sequence.py extract \
  --bag docker/artifacts/tunnel-contingency-live-20261023-162509/run-04/diagnostics_0.bag \
  --start 1792745540 --end 1792745548 \
  --output docker/artifacts/tunnel-raw-sequence-$(date +%Y%m%d-%H%M%S)
# --input에 위 새 디렉터리를 지정한다. --baseline-head는 HEAD의 obstacle_node만 메모리에 읽는다.
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 \
  python3 tools/probe_tunnel_lidar_sequence.py replay \
  --input docker/artifacts/tunnel-contingency-live-20261023-162509/run-04-lidar-inputs \
  --actor-xy -2.371 -506.936 \
  --output docker/artifacts/tunnel-lidar-replay-$(date +%Y%m%d-%H%M%S).json
```

#### run-05 준비·시작

사용자가 registered16 재로드·시작점·Manual/P·재연결 준비 완료를 확인했다.
소스348개와 설치 장면 SHA 불변, 전체 회귀/원본 재생·계획기 비교를 확인했다.
preflight-05는25초 실제 수신/TF/target PASS·drive=false·제어UDP0·exit0·s1.730m였다.
같은 registered16 장면에서 기존 감독 기준을 유지한 새 run-05를 시작했다.
그 결과를 추가하기 전까지 충돌 해소·실제 우회/완주 성공을 주장하지 않는다.

#### run-05 중단과 사용자 직접 재시험 구분

run-05는 고속 구간 s1308.79/283.771초에서 `raw_vehicle_status_lost`로 중단됐다.
종료 시 정지·Manual/P raw 확인은 없었고 마지막 상태는 Auto/D·29.608m/s였다.
모든 자식은 exit0, 생산 source348개는 불변이었다. 사용자에게 Pause/정지 인계를 요청했고,
사용자는 중간에 잘못 조작했으며 이미 직접 재시험 중이라고 답했다.
이 실행은 터널 수정 성공도 새 알고리즘 실패 재현도 아니며, 조작과 통신 끊김의 상세 원인은
독립 확정하지 못했다. [원본 종료 요약](tunnel-contingency-20261023/live/run-05-interrupted-summary.json).
종료 확인 없는 시험을 정상 인계로 바꾸지 않는다.

이후 사용자가 호스트 `roslaunch ctrl_start.launch`를 직접 실행한 컨테이너
`c2d7f69c58c7`/기록 폴더 `ctrl_start_foscar_7_7691998378760118893`를 확인했다.
중복 drive를 실행하지 않고 소스348개를 고정한 채 기록을 관찰했다.
Player.log의 마지막 로드도 `CTRL_tunnel_left_box_registered16.json`이었다.
사용자는 **상자를 피했고 화면상 충돌 없었음**, 이어 **Ctrl+C 종료·정지·Manual/P 확인**을 답했다.

닫힌 raw/진단 bag2개와 follower CSV의 SHA256을 새 요약에 남겼다.
직접 실행은 외부 collision UDP 감사기가 붙지 않았으므로 `유효 충돌 패킷0`이라고 판정하지 않는다.
사용자 화면 보고와 bag/CSV의 경로·속도를 각각 근거로 기록한다.

| 확인 항목 | 직접 재시험 기록 |
| --- | --- |
| 제공 CP | CP1~10·12~15 전체 반경3m 내; 종료점 최소거리0.328m |
| 제어 종료 | follower FINISH, s2234.552, 최종 속도0.00000122m/s, brake1 |
| 상자 주변 관측 | 31개, 최초 ego s1880.969(예정 중심 약24.0m 전) |
| 상자 우회 | AVOID target282개, 최대 목표 offset4.5m, 실제 최대 d4.614m |
| 복귀 | s1989.32 부근 d≈-0.095m; s1990.75 GPS 복귀; s2005.19 d=-0.239m |
| 기존 지도/이탈 기준 재적용 | 원본 pose1067개, 지도 밖+경로거리3.5m 초과0, 지속 이탈 경보0 |
| 충돌·인계 | 화면상 무충돌/정지·Manual/P는 사용자 확인; 이 실행의 raw 충돌0/인계 표본 수 미검증 |

[직접 재시험·원본 SHA·CP/FINISH](tunnel-contingency-20261023/live/user-retest-01-summary.json),
[실제 관측·우회/복귀](tunnel-contingency-20261023/live/user-retest-01-tunnel.json),
[기존 지도·이탈 기준](tunnel-contingency-20261023/live/user-retest-01-path.json).
관측 위치 매칭은 simulator actor ID의 독립 검증이 아니고, 실제 d/지도 포함은 차체 전체의
벽 여유·가려진 상자 외곽 간격 실측이 아니다. 공식 서버 점수/모든 돌발 배치 통과로 확대하지 않는다.

`analyze_tunnel_variant.py`는 종료 확인 파일이 없는 사용자 실행도 명시적인
`--termination-record`(같은 run_dir, 사용자 정지·Manual/P 확인)와 닫힌 bag이 있을 때만 읽는다.
출력에 외부 감사/사용자 확인 종료를 구분한다. 원본 run에 감사 end.json을 만들어 넣지 않았다.

```bash
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 \
  python3 tools/analyze_tunnel_variant.py \
  --run docker/artifacts/ctrl_start_foscar_7_7691998378760118893 \
  --scene-manifest docker/artifacts/tunnel-contingency-live-20261023-162509/scenes-registered16/manifest.json \
  --termination-record docker/artifacts/tunnel-contingency-live-20261023-162509/user-retest-01/termination-user.json \
  --output docker/artifacts/tunnel-user-analysis-$(date +%Y%m%d-%H%M%S)
```

#### 이번 단계 마감·남은 검증

현재 source는 동일한 수정 후보348개이며 사용자가 실행을 종료하고 정지·Manual/P를 확인했다.
새 낮고 밝은 터널 상자의 **인지→감속/우회→복귀→FINISH**를 사용자 직접 재시험의 기록으로 확보했다.
외부 충돌 패킷 감사가 있는 전체 주행 성공과 구분한다. 새 source의 Windows/catkin/native 설치 빌드는
이번에 실행하지 않았다. Ubuntu의 실제 ROS1/MORAI 실행은 위 범위로 확인했다.

| 주요 변경 파일 | 변경 내용 |
| --- | --- |
| `src/foscar_planning/foscar_planning/bev_avoidance.py` | 넓은 점유체 내부 도로 겹침/지도 없음 제약 보존 |
| `src/foscar_planning/foscar_planning/tunnel_simple.py` | 도로 검증에 실패한 늦은 복귀 대신 유효한 기존 후보 재검색 |
| `src/foscar_lidar/foscar_lidar/obstacle_node.py` | 지면 검증 영역의 밝은 상승 물체 유지/현재 터널 외곽 전달 |
| 신규 planning 검사2개 / lidar 검사2개 | 위 실패 경계·도로/지면/임무/이벤트 보존 회귀 |
| `tools/prepare_tunnel_scene.py` / 검사 | 원본 보존 장면 생성, native 확인 ID/빈 경로 등록 |
| `tools/probe_tunnel_contingencies.py` / 검사 | 조건별 판단 프로브 |
| `tools/probe_tunnel_wall_clearance.py` / 검사 | 선행 실제 벽을 사용한 복귀 후보 비교 |
| `tools/probe_tunnel_lidar_sequence.py` | 원본 연속 입력 추출/검출·추적/HEAD 비교 |
| `tools/analyze_tunnel_variant.py` | 종료된 실제 시험의 관측/우회/복귀·카메라 분석 |
| 현재 문서/개발 계획/문서 분류/새 요약 | 재현 명령·실패·검증 범위·후속 조건 기록 |

다음 우선순위는 같은 왼쪽 상자의 외부 충돌 감사 포함 반복1회, 중앙/우측·연속 장애물,
보행자 재진입/진로 차단 후 해제다. 새 장면마다 native 물체 생성·실제 크기/가시성부터 확인한다.
이번 성공 한 번을 이 조건들의 완료로 합산하지 않는다. GitHub push/main 추가 병합은 하지 않았고
작업은 `feat/tunnel-contingency-20261023`의 미커밋 변경으로 남긴다.

마감 정적 점검: 변경 Python15파일 구문, 문서 링크242개, 새 요약 JSON38개를 확인했고
고정 source348개/비교 원본530개 변화0이었다. 일반 `git diff --check`는 보존된 planning
CRLF의 CR-at-EOL로 exit2였고 `git -c core.whitespace=cr-at-eol diff --check`는 통과했다.
활성 Docker 컨테이너0개를 확인했다. [마감 요약](tunnel-contingency-20261023/live/final-checks-20261023.json).

## 추가 실험 현황

### 반복·조합 실험 1차 — 동일 배치의 재실패와 관측 공백 수정

사용자의 추가 실험 요청으로 기존 수정 후보 source348개를 고정하고 외부 감독 주행과
오프라인 조합 검사를 시작했다. 이 단계에서 **같은 왼쪽 상자 첫 주행은 완주, 두 번째는 충돌**했다.
단순 완주 이력을 반복 안정성 확보로 확대할 수 없다. 기록은
`docker/artifacts/tunnel-batch-20261023-01/`에 따로 저장했다.

| 시험 | 이번 측정 결과 | 판정 범위 |
| --- | --- | --- |
| 첫 수신전용 점검 | 25초 PASS, 제어UDP0, 시작 s2.127/P/정지 | 실센서·ROS1·TF·판단 수신 |
| 왼쪽 반복1 | 245.22초, s2174.552, route_end_stopped | raw 충돌10,138패킷 중 유효 충돌0, 인계31표본/0.679초 |
| 두 번째 수신전용 점검 | 25초 PASS, 제어UDP0, 시작 s0.937/P/정지 | 새 장면 재로드 후 독립 점검 |
| 왼쪽 반복2 | 378.18초, s1903.248에서 collision_packet_alarm | 상자 ID16, 충돌 보고12패킷은 한 연속 사건; 인계31표본/0.667초 |
| 초기 합성 조합 | 159조건/240프레임, 150통과/9실패 | 순수 계산, 차체 Frenet 사각형 근사 |
| 정지 축 기준을 반영한 확장 | 267조건/348프레임, 263통과/4실패 | 앞바퀴 stop_s를 후륜 위치로 변환 |
| 큰 조합, 수정 전/최종 후보 각각 | 1,167조건/1,248프레임, 1,163통과/4실패 | 동일 조건 반복을 별도 독립 조건 수로 합산하지 않음 |
| 관측 범위 경계 | 독립 위치30조건, 22통과/8전체경로 겹침 | 이미 주행 중인 곡선의 재계획 시험은 아님 |
| 연속 원본 LiDAR 재생 | 성공 기록68프레임, 현재 검출12/외곽25, HEAD 검출6/외곽0 | ROS 계산 대역, native TF·콜백 스케줄 재생 아님 |
| 실제 벽 복원 | 선행 벽42스캔, 좌·중앙·우측 후보3개 통과 | 실제 벽+합성 상자; native 장애물 배치 시험과 구분 |

[반복1 원본 재검증](tunnel-contingency-20261023/batch-01/live-left-01.json),
[반복2 충돌·인계](tunnel-contingency-20261023/batch-01/live-left-02.json),
[초기 조합](tunnel-contingency-20261023/batch-01/matrix-initial.json),
[최종 큰 조합](tunnel-contingency-20261023/batch-01/matrix-dense-final.json),
[관측 범위 경계](tunnel-contingency-20261023/batch-01/horizon-fresh-poses.json),
[LiDAR 동일 입력 비교](tunnel-contingency-20261023/batch-01/lidar-differential.json),
[기록된 벽 비교](tunnel-contingency-20261023/batch-01/recorded-wall-probe.json).

초기 검사기는 `Target.stop_s`를 무시하고 상자 뒤까지 경로를 연장했다.
제어기 `core.py`의 계약은 앞바퀴 정지 위치이므로 후륜 끝점은 `stop_s - FRONT_AXLE(3m)`다.
새 검사기는 그 정확한 끝점까지 검사한다. 초기 JSON은 보존했으며 무제한 직진/끝점 접촉/
멀리 있는 상자 겹침을 여전히 잡는 독립 SAT 검사를 추가했다. 유한 정지 목표가 실제 제동거리
확보를 뜻하지는 않는다.

남은 큰 조합 실패4개는 첫 상자 d=-1, 두 번째 d=+1, 간격20/30m, 속도0/20km/h다.
ego s1880일 때 두 번째 상자는 40m 관측 범위 밖이어서 첫 계획에 포함되지 않았고,
그때 출력한 전체 경로를 계속 유지하면 두 번째 상자와 겹친다. 독립 새 위치에서 s1885/1895로
접근하면 두 번째 상자가 제약에 포함되어 정지 목표가 생성됐다. **이미 채택한 곡선 중에
두 번째 장애물을 새로 만나는 실제 재계획·제동 성공은 미검증**이다. 이 실패를 삭제하거나
관측 범위 밖으로 검사 범위를 줄여 통과 처리하지 않았다.

#### 반복2 충돌 경계

처음 외곽 전달은 ego s1881.890, planner의 첫 곡선은 s1882.4였다. 다음 네 스캔에서는
상자 외곽이 없었고 같은 원본 재생에서도 빈 관측을 재현했다. 일부 스캔은 상자 주변 높은
raw 점 자체가0개였다. 밝은 물체 필터를 다시 조정할 근거는 없었다.

약0.567초 뒤 같은 트랙을 다시 관측했으나 그 사이 첫 곡선이 취소되어 TUNNEL/직진 상한으로
돌아갔다. s1889.870에서 새 곡선을 채택할 때 실제 속도는 약48km/h였고, 충돌 직전 s1903 부근
실제 d는 약1.61m였다. 성공 주행의 같은 위치 d는 약4.16m였다. 이는 기록된 추정 위치 비교이며
숨은 상자 외곽/차체 여유 실측은 아니다. 충돌 여부는 raw 패킷의 type2/id16으로 확인했다.

원인은 `states/avoid.py`의 두 경로였다. 첫째, 막 채택한 entry의 현재 offset이 .05m 이하면
빈 관측에서 상태를 종료했다. 둘째, 상태를 유지하더라도 빈 groups로 재계획하며 entry/return을
없앴다. 신규 검사에서 변경 전 2실패/4통과로 재현했다.

최종 후보는 TUNNEL의 유효한 미완료 entry/return만 이어간다. 새 관측은 원래 재계획·충돌
검사를 거치고, 빈 관측 중에도 갱신된 벽의 차체 포장면 검사를 수행한다. 좁아진 벽에서는
기존 곡선을 정지시킨다. 새 기억·시간 제한·ROI·threshold·속도·watchdog·메시지 필드는 추가하지 않았다.
한 스캔 물체 비기억과 유효 곡선의 완료 후 해제도 유지한다.

초기 신규 속도 검사 한 줄은 기존 `held_speed`의 완만한 상승(5.0→5.03m/s)을 금지하는
잘못된 계약이었다. 기존 속도 정책은 바꾸지 않고 같은 시각·위치의 연속 관측 대조군보다
상한이 높아지지 않는 차등 검사로 바로잡았다. 중간 실패 기록을 보존했다.
[신규 검사 계약 설명](tunnel-contingency-20261023/batch-01/new-test-contract-note.json).
이후 추가한 벽 갱신 검사도 초기 후보에서 실패를 확인한 뒤 보완했다.

최종 신규7검사 PASS. 제어/계획/LiDAR/카메라 전체 계산 회귀는 **979개 중 978PASS, 기존
ROUNDABOUT 지면 기대값1FAIL, skip0**였다. 중간 전체 회귀는 벽 검증을 포함시키기 위해
411PASS에서 명시적으로 중단했고 최종 후보로 다시 실행했다. 원본 시험·기대값은 수정하지 않았다.
실험 도구28검사 PASS, 경고10개는 기존 matplotlib/pyparsing 경고다.
[회귀 이력·로그 SHA](tunnel-contingency-20261023/batch-01/regression.json),
[최종 source349개](tunnel-contingency-20261023/batch-01/source-final.json).

중앙·우측 장면 복제본도 생성·설치했으나 반복2 충돌 후 실제 주행은 보류했다.
[생성 장면 SHA](tunnel-contingency-20261023/batch-01/generated-scenes.json).
원본 sample_scene와 기존 registered16 파일은 보존했다. 목록에 설치됐다는 사실을 native
물체 검증·주행 통과로 합산하지 않는다. 다음 감독 주행은 같은 왼쪽 장면에서 관측 공백 수정 효과 확인이다.

#### 공백 길이·크기·속도 차등 비교

신규 `tools/probe_tunnel_dropout.py`는 같은 단일 관측→1/4/8빈 스캔→재등장 입력을
전·후 `Avoid`에 각각 넣었다. d=-1/0/+1m, 관측 폭1/2.1m, 속도0/20/50km/h,
접근 거리25/35m의108조건이다. 처음부터 우회 가능한81조건은 이전 코드에서 공백 중 유지0,
최종 후보에서81개 모두 유지였다. 기타 판단 검사 실패0이다. 처음부터 정지를 선택한27조건은
우회 성공으로 집계하지 않았다. prescribed ego는 출력 속도와 무관하게 진행하므로
실제 제동·추종·충돌 성공으로 해석하지 않는다.
[변경 전](tunnel-contingency-20261023/batch-01/dropout-matrix-before.json),
[변경 후](tunnel-contingency-20261023/batch-01/dropout-matrix-after.json).

재현할 때 출력 경로는 반드시 새 이름을 사용한다. 아래 비교 기준은 이번 수정 직전의
불변 격리 사본이다. HEAD에는 앞선 LiDAR/계획 수정도 없으므로 전체 코드를 HEAD로 바꾸지 않는다.

```bash
# 기하 조합: ROS/UDP/주행 없음
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 python3 tools/probe_tunnel_matrix.py \
  --dense --output docker/artifacts/tunnel-matrix-$(date +%Y%m%d-%H%M%S)
# 관측 공백: 기본은 현재 Avoid
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 python3 tools/probe_tunnel_dropout.py \
  --output docker/artifacts/tunnel-dropout-$(date +%Y%m%d-%H%M%S)
# 동일 입력의 수정 직전 Avoid만 비교
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 python3 tools/probe_tunnel_dropout.py \
  --baseline-avoid verification/.work/tunnel-dropout-before-20261023-01/tree/src/foscar_planning/foscar_planning/states/avoid.py \
  --output docker/artifacts/tunnel-dropout-baseline-$(date +%Y%m%d-%H%M%S)
# 종료 bag 분석 뒤, raw 충돌 재해석/원본 SHA를 포함한 새 요약 생성
python3 ros1-port/tools/summarize_tunnel_run.py \
  --run ros1-port/docker/artifacts/tunnel-batch-20261023-01/live-left-01 \
  --analysis ros1-port/docker/artifacts/tunnel-batch-20261023-01/left01-analysis \
  --output /tmp/tunnel-live-left01-summary-$(date +%Y%m%d-%H%M%S).json
```

#### 최종 후보의 왼쪽 감독 재시험

사용자가 같은 장면의 시작점·정지·Manual/P·채널 준비를 확인한 뒤,
수신전용 preflight-03에서 s0/P/정지·센서/TF/target PASS·제어UDP0을 확인했다.
고정 source349개로 `live-left-03-after-dropout`을 실행했다.

- `follower_finish`, 400.91초, s2174.552. source349개 불변, callback 오류0, child 종료코드 모두0.
- raw 충돌16,001패킷 재해석 결과 유효 충돌0·파싱 오류0.
- 종료 인계31표본/0.658초, 최대 절대 속도0.001133m/s, Manual/P 확인.
- 상자 주변 관측27개, 최초 ego s1880.785, 최대 목표 offset4.5m·실제 d4.614m.
- s1903 부근 d4.309m, GPS 복귀 s1990.224.

[실제 실행·bag/원본 SHA·raw 충돌](tunnel-contingency-20261023/batch-01/live-left-03-after-dropout.json),
[세 주행의 실제 비교](tunnel-contingency-20261023/batch-01/left-comparison.json),
[수신전용 1~3차](tunnel-contingency-20261023/batch-01/preflights.json).

실패 반복2는 첫 우회 후 s1903까지 target36개 중10개가 AVOID를 이탈했고,
수정 후 반복3은42개 모두 AVOID를 유지했다. 다만 두 native 주행의 입력은 같지 않다.
실패 주행의 최대 초기 관측 공백은0.567초, 수정 후는0.234초였다. 같은0.567초 공백을 실제
MORAI에서 강제로 재현해 통과한 것은 아니며, 공백 입력의 차등 증거는 위 오프라인108조건이다.
한 차례 수정 후 성공을 모든 초기 탐지 거리·조건의 안정성 확보로 확대하지 않는다.

이후 중앙 장면을 재로드했으나 preflight-04는 Pause 상태에서 모든 입력0으로 실패했다.
제어UDP 송신0·launch 종료0이며 감독 주행은 시작하지 않았다. 화면의 D 상태도 있어
Pause를 유지하고 사용자에게 같은 중앙 장면의 원래 시작점·정지·Manual/P·연결 준비를 요청했다.
이 실패는 알고리즘 실패/충돌/우회 시험 횟수로 합산하지 않는다.

#### 중앙 배치 감독 시험

사용자 준비 확인 뒤 새 preflight-05가 PASS했다. 시작 s1.962m/P/정지·제어UDP0이며
앞서 입력0이었던 preflight-04를 덮어쓰지 않았다. 같은 최종 source349개로 중앙 배치를 시험했다.
`live-center-01`은391.88초에 follower_finish, s2174.552였고 raw 충돌15,689개 중
유효 충돌0·파싱 오류0이었다. 정지/Manual/P 인계30표본/0.650초, 최대 절대 속도0.001160m/s,
source 변화0·콜백 오류0·child 종료코드 모두0이다.

추가 물체 위치 주변 관측22개, 최초 ego s1881.692, 최초 관측 d[-0.430,+1.051]m였다.
최대 목표 offset3.5m, 추정 실제 d3.647m, GPS 복귀 s1989.273을 기록했다.
이는 native에서 로드한 중앙 장면의 주변 관측과 경로이며 simulator actor ID/숨은 물체 크기/
차체의 최소 여유를 별도 실측한 것은 아니다.
[중앙 실행·관측·충돌 원본 SHA](tunnel-contingency-20261023/batch-01/live-center-01.json),
[중앙 새 수신 점검](tunnel-contingency-20261023/batch-01/preflight-05-center.json),
[이전 입력0 점검 보존](tunnel-contingency-20261023/batch-01/preflight-04-not-ready.json).

중앙 주행이 종료된 뒤 `CTRL_tunnel_right_box_batch01` 로드 완료를 MORAI 로그와 화면에서
확인했고 Pause로 유지했다. 사용자에게 같은 오른쪽 장면의 시작점·정지·Manual/P·연결 준비를
요청했다. 현재까지 새 source로는 왼쪽·중앙 각1회만 외부 감사 성공이며 오른쪽은 아직 미실행이다.

같은 왼쪽 장면의 [추정 횡위치·속도 비교 그래프](tunnel-contingency-20261023/batch-01/left-native-paths.png)와
[그래프 입력·출력 SHA](tunnel-contingency-20261023/batch-01/left-native-paths.json)를 추가했다.
충돌 경보 이전 pose만 사용했고, 점선 s1905는 배치한 상자의 예정 중심 위치다.
추정 localization 경로를 실제 차체 외곽·충돌체 실측으로 해석하지 않는다.

```bash
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 python3 tools/plot_tunnel_native.py \
  --batch docker/artifacts/tunnel-batch-20261023-01 \
  --output /workspace/ros1-port/docker/artifacts/left-native-paths-$(date +%Y%m%d-%H%M%S).png
```

#### 이번 추가 요청의 변경 파일과 미확인 범위

| 파일 | 이번 추가 작업 |
| --- | --- |
| `src/foscar_planning/foscar_planning/states/avoid.py` | 공백 중 유효한 터널 entry/return 유지, 갱신 벽의 기존 차체 포장면 재검증 |
| `src/foscar_planning/test/test_tunnel_initial_dropout.py` | 초기 취소·대조 속도·복귀 후 종료·stale·새 장애물·벽 축소·차단 해제7검사 |
| `tools/probe_tunnel_matrix.py`, `tools/test_tunnel_matrix.py` | 다양한 위치/크기/속도/사람/차단/결측 조합, 독립 SAT·앞바퀴 정지 끝점 검사 |
| `tools/probe_tunnel_horizon.py` | 먼 두 번째 장애물이 제약에 들어오는 독립 위치 경계 |
| `tools/probe_tunnel_dropout.py` | 동일108공백 순서의 변경 직전/현재 Avoid 비교 |
| `tools/summarize_tunnel_run.py`, `tools/test_summarize_tunnel_run.py` | 닫힌 감사 원본 충돌 재해석·SHA·인계·관측 요약 |
| `tools/plot_tunnel_native.py` | 충돌 이전의 실제 추정 경로·속도 비교 그래프/원본 해시 |
| 현재 터널 문서·개발 계획·문서 분류·새 `batch-01/` 증거 | 원인·실패·검사 범위·재현 명령·새 결과 정리 |

이 단계 이전부터 존재하던 `bev_avoidance.py`, `tunnel_simple.py`, `obstacle_node.py` 변경은
보존했으며 이번 반복 실험 중에는 추가로 수정하지 않았다. 원본 비교 트리·이전 검증 JSON·기대값은
보존했다. 테스트·인지 실패를 피하려고 threshold/ROI/모델/경로/메시지/launch/속도/watchdog를 바꾸지 않았다.
Windows 검사·새 catkin/native 설치 빌드는 이번에도 실행하지 않았다. Ubuntu Docker의 실제
ROS1/UDP/MORAI 실행과 계산 대역 검사 범위를 구분한다. 공식 점수, 숨은 물체 외곽의 실제
차체 여유, 원거리 연속 물체의 진행 중 재계획, 터널 보행자 재진입/차단 후 해제의 실제 시험은 미확인이다.

오른쪽 장면은 native 로드 후 Pause이며 사용자 준비 확인을 기다린다. 기다리는 동안
5초 Status 수신전용 확인은 패킷0·제어 송신 없음이었다. 이를 정지/P 확인이나 실제 주행으로
계산하지 않았다. [추가 실험 마감 상태](tunnel-contingency-20261023/batch-01/final-state.json).

#### 오른쪽 준비 응답 후 시험 재개

사용자가 같은 오른쪽 장면의 시작점·정지·Manual/P·채널 준비 완료를 확인했다.
설치된 CTRL_tunnel_right_box_batch01.json SHA는 생성 manifest와 일치했고 예정 s1905/d=-1/ID16이다.
source349개 변화0을 확인한 뒤 preflight-06을 새로 실행했다. 25초 수신·TF·target PASS,
제어UDP0·drive=false·launch 종료0, 최종 s1.962m/P/절대 속도0.000001m/s 미만이었다.
기존 외부 감독 기준으로 live-right-01을 시작했고, 주행 중 source349개를 유지한다.
이전 final-state.json은 오른쪽 준비 대기 당시 상태로 보존한다.
[오른쪽 수신 점검](tunnel-contingency-20261023/batch-01/preflight-06-right.json).

#### 오른쪽 완료와 이번 batch 최종 결과

live-right-01은 **352.89초, s2174.552에서 route_end_stopped**로 종료했다.
raw 충돌14,297개 재해석에서 유효 충돌0·파싱 오류0, 정지/Manual/P 인계30표본/0.646초,
최대 절대 속도0.001234m/s였다. source349개 변화0·콜백 오류0·child 종료코드 모두0이다.
추가 물체 근처 관측22개, 최초 ego s1880.309, 최대 목표 offset2.75m·추정 실제 d2.849m였다.
실제 접근 카메라에서 상자를 확인했다. GPS 복귀 s1989.348, s2000 부근 d=-0.144m를 기록했다.
[오른쪽 원본 SHA·관측·raw 충돌·인계](tunnel-contingency-20261023/batch-01/live-right-01.json).

| 구분 | 왼쪽 | 중앙 | 오른쪽 |
| --- | --- | --- | --- |
| 최종 source349개 감독 주행 | 완료 | 완료 | 완료 |
| 종료 | follower_finish | follower_finish | route_end_stopped |
| 추가 물체 주변 관측 | 27개 | 22개 | 22개 |
| 최초 관측 ego s | 1880.785m | 1881.692m | 1880.309m |
| 최대 목표/추정 실제 d | 4.5/4.614m | 3.5/3.647m | 2.75/2.849m |
| s2000 부근 추정 d | -0.088m | +0.039m | -0.144m |
| raw 충돌 패킷/유효 충돌 보고 | 16,001/0 | 15,689/0 | 14,297/0 |
| 제공 CP 반경 내 | 15/15 | 15/15 | 15/15 |
| target 수신 간격1초 초과 | 0 | 0 | 0 |
| 정지·Manual/P 인계 | 31표본 | 30표본 | 30표본 |

수정 후 세 배치에서 총45,987개의 충돌 원본에 충돌 보고가 없었다. 이는 세 주행의 결과이며
45,987번의 독립 주행 성공이 아니다. 이번 batch 전체는 감독 주행5회 중 완료4회,
**수정 전 같은 왼쪽 상자의 충돌 중단1회**다. 실패 원본과 준비 대기 당시 final-state.json은 보존했다.
[전체5회 비교](tunnel-contingency-20261023/batch-01/native-five-trials.json),
[현재 최종 상태](tunnel-contingency-20261023/batch-01/final-state-right-complete.json).

닫힌 bag의 압축 카메라 원본을 변환 없이 복사해
[왼쪽](tunnel-contingency-20261023/batch-01/left-after-approach.jpg)·
[중앙](tunnel-contingency-20261023/batch-01/center-approach.jpg)·
[오른쪽](tunnel-contingency-20261023/batch-01/right-approach.jpg) 관측 위치를 확인할 수 있다.
[카메라 원본 SHA](tunnel-contingency-20261023/batch-01/native-approach-cameras.json).
그림과 위치 주변 관측을 simulator actor ID·가려진 전체 물체 크기·최소 차체 여유의 독립 실측으로
확대하지 않는다. 제공 CP 반경 내 통과는 공식 점수 서버의 채점 결과가 아니다.

추가 코드 변경은 없으며 이번 재개에서는 새 수신/주행/분석/요약과 현재 문서만 작성했다.
이 소스의 이전 전체 회귀979개 중978PASS/기존1FAIL, 도구28PASS 결과를 유지한다.
소스가 바뀌지 않았으므로 전체 회귀는 중복 실행하지 않았다. Windows·새 catkin/native 설치 빌드는
미실행이며 인터페이스/모델/지도/경로/threshold/ROI/기억/속도/watchdog 변경은 없다.
다음 우선순위는 원거리 두 번째 장애물을 진행 중 만나는 재계획·제동, 터널 보행자 재진입,
차단 해제와 동일0.567초 공백의 실제 재현이다. 좌·중·우 각1회의 성공을 이 조건들의 완료로 계산하지 않는다.

오른쪽 완료 후 정적 점검은 변경 Python24파일 구문·문서 링크319개에서 오류0,
고정 source349개 변화0·비교 원본530개 변화0·카메라 원본 SHA 변화0이었다.
닫히지 않은 bag0·오른쪽 child 종료코드 모두0·실행 중 시험 Docker 컨테이너0을 확인했다.
일반 git diff --check는 기존 CRLF의 CR-at-EOL로 exit2였고 CR-at-EOL 허용 검사는 exit0이다.
새 근거는 batch-01/static-checks-right-complete.json에 저장했다. commit/push/main 병합은 하지 않았다.
