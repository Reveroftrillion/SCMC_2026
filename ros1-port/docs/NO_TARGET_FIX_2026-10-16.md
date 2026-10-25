# 반복 출발·정지 원인과 수정

**현재 상태: 아래 수정안은 폐기하고 전체 롤백했다.** 수정·테스트 내용은 당시 실험 이력이며,
현재 코드의 해결 완료 결과가 아니다. 이후 실제 주행 결과는
[전체 코스 점검](FULL_COURSE_AUDIT_2026-10-16.md)을 참조한다.

목표: 실제 주행의 Behavior 목표 발행 중단을 재현하고, 안전 타임아웃·충돌·도로 판정을
유지하면서 원인을 수정한 뒤 새 녹화를 준비한다. 기존 영상·bag·센서 설정은 보존한다.
작업 브랜치는 `run/ros1-update-20261016`, 기준 HEAD는 `2d34dc2`다.

## 실제 원인

첫 녹화의 follower CSV 9,737행 중 `NO_TARGET`이6,333행이었다.
동일 bag에서 odometry와 control request 발행은 계속됐지만 Behavior target은
최대73.608초 중단됐다. 설정된 target 유효시간을 넘으므로 follower가 제동하고,
늦게 계산된 target이 발행되면 다시 출발하는 현상이 반복됐다.

별도 ROS master11312에서 실제 기록 입력만 Behavior에 재생했다.
센서bridge나제어송신 노드는 실행하지 않았다. 멈춘 worker의 traceback은
`Avoid.obstacle_target → tunnel_slalom.plan → paved_moves → LaneMap.contains_intersection_road`
후보탐색·차체 도로 검사에 머물렀다. 센서 연결 중단이나 ROS master 종료가 원인은 아니었다.

첫 실패구간 s309.7m에서 BEV 시설물의 lateral 범위는 대략 다음과 같았다.

- s323.9m, d[-14.35,-12.35]
- s327.7m, d[-0.36,1.39]
- s333.2m, d[-21.37,-20.37]
- s342.2m, d[2.37,3.37]

차체가 어떤 후보로 이동해도 닿을 수 없는 먼 시설물 두 개까지 회피 단계로 묶으면서
불필요한 중간 이동·복귀 후보를 대량 탐색했다. 후보 간격을 바꾸는 시도는 기존 테스트에
영향을 줬고 최종안에서 제거했다. 최종안은 기존 후보 간격과 선택 로직을 유지한다.

## 최종 수정과 안전 범위

production 변경은 `src/foscar_planning/foscar_planning/states/avoid.py`의
정적 장애물 회피 단계 생성 부분뿐이다. 테스트는 `test_static_lateral_reach.py`를 추가했다.

정적 장애물 구간에서 후보 최대 offset, 현재 offset, 기존 확정 chain의 모든 offset을 포함해
최대 lateral 이동범위를 구한다. 여기에 회전한 차체가 가질 수 있는 최대 lateral 반경
`hypot(max(front,rear), planner.half)`를 더한다. 이 범위와 전혀 겹치지 않는 박스만
계획 단계 생성에서 제외한다. 경계에 닿는 박스는 유지하며 비유한 계산에는 필터를 적용하지 않는다.
원래 센서 증거·memory·BEV occupancy는 그대로 보존한다.

5차 이동은 시작·끝 offset 사이를 벗어나지 않고, 회전한 차체의 lateral 반경은 위 값 이하이므로
제외 박스는 기존 후보 경로와 충돌할 수 없다. 가까운 박스, 차체 회전, 지도상의 도로,
BEV clearance, 신호·속도·watchdog·target_timeout은 그대로 검사한다.
터널 알고리즘과 정지선·모델·ROI·경로 파일은 변경하지 않았다.

## 재현 결과

기록 입력1792132175–1792132250의75초를 실제 ROS1에서 재생한 최종 수정본은
target 발행간격1,498개 중 최대0.1217초,0.5초 초과0개였다.
같은 구간의 기존73초 공백이 재발하지 않았다. 새로운 planner는 가까운 두 박스를 모두 유지했고,
차체·도로 검사를 통과한 우회 chain을 생성했다. 실제 주행 완료를 뜻하는 검사는 아니다.

원시 근거는 Git 제외 `docker/artifacts/no-target-fix-20261016/`에 보존했다.
`bag-gaps.json`, 최초`live-replay.log`, 최종`fixed-04/live-replay.log`,
`fixed-04/target-gaps.json`을 참조한다. 폐기한 수정안의 로그도 남겼고 성공 결과로 사용하지 않는다.

## 검증·재녹화 인계

planning/control 회귀 검사는 일회용 사본에서 실행하며 결과는 `unit-fixed.log`에 남긴다.
새 테스트는 도달 불가능한 먼 시설물을 제거하되 센서 증거는 보존하고, 경계 접촉 박스를
계획에 유지하는지 검사한다. 최종 테스트 결과 및 재주행 결과는 후속 확인 후 추가한다.

사용자는 시나리오 재로드·P 설정과 재주행을 승인했다. 재로드 뒤 UDP 소켓이 사라져
Ego Ctrl Cmd/Competition Status 및 센서 Connect 재설정을 요청했다.
연결 확인 및 회귀 검사 완료 후 새 폴더에서 녹화·주행을 시작한다.
기존 `midcheck-20261016/submission/CTRL.mp4`와 `CTRL_sensor.json`을 덮어쓰지 않는다.

재녹화 정상 종료는 주행launch Ctrl+C1회 → Manual/P 인계 확인 → recorder 종료 →
ffmpeg터미널q → MP4변환·전체디코딩검사 순서다. 비상시MORAI Pause를 사용한다.


## 최종 검사 결과

planning/control 전체 사본 검사809개 중808PASS, offline도구가 사본에 누락돼
highway measurement1개가 실패했다. 기존도구를 사본에 추가하고 해당항목과 새테스트3개를
다시실행해4PASS를 확인했다. 기존807개+신규3개,총810개 항목의 통과를 확인했다.
기존기대값·검사기준은변경하지 않았다. 원시로그는`unit-fixed.log`,`targeted-final.log`다.
구문검사와CRLF를고려한git diff검사도통과했다.
프로파일링중기존highway초기화가재생성한kappa캐시는HEAD바이트로복원했다.
최종production변경은avoid.py1개이며실행·지도·경로설정변경은없다.

사용자가시나리오재로드후UDP재연결을완료하고재주행을명시승인했다.
새녹화폴더는`retake-20261016-160848/`(midcheck-20261016아래)다. 기존제출영상은보존한다.


## 시나리오 재로드 시 순환 경로 시작점 보정

재연결 후drive=false에서 실제 위치(-131.93,-428.83)의 최근접 투영이
순환 경로 끝s약2174m를 선택해 시작점 미션을 건너뛸 수 있음을 확인했다.
새 시나리오 시작용으로`docker/local.launch start_at_beginning:=true`옵션을 추가했다.
초기hint는global경로 시작에서5m 이내일 때만0m를 주고localization과follower의
첫 투영window를 맞춘다. spawn경로나멀리떨어진차량에서는거부한다. 차량위치·경로는바꾸지않는다.
기본값false로기존일반실행·중간재개최근접투영은유지한다.
core helper,localization,follower,local.launch를변경했고시작/끝모호성·먼위치·
누락/NaN위치4개케이스를추가했다. control과새reach검사까지119PASS를확인했다.
첫launch검증에서발견한인자전달오류는수정하고실패로그를보존한뒤다시검증했다.


## 16:14 KST 재녹화 시작

시작점옵션을drive=false로10초검증해s0.0m,센서·위치·Behavior정상,
제어UDP0개와launch종료0을확인했다. 근거`start-hint-live-02.json`.
사용자의재주행승인에따라새녹화를먼저시작하고
`roslaunch docker/local.launch drive:=true start_at_beginning:=true bag:=true`로재출발했다.
기존run.sh에호스트보존HOME/ROS_LOG_DIR/bag_dir을지정했다.
차량은시작점에서s202.25m까지이동했고첫교차로정지선에서대기했다.
아직두번째교차로이후실제수정효과·전체미션완주·최종영상검사는미확인이다.
새CTRL.mkv와bag을계속기록중이며기존CTRL.mp4는보존했다.


## 사용자 요청에 따른 전체 로직 롤백

사용자가 수정본의 실제 회피 실패를 보고하고 모든 로직 변경을 되돌리도록 요청했다.
주행·녹화를 종료하고 Manual/P 인계 완료를 확인했다. 수정한 production5개 파일을
HEAD2d34dc2 원본 바이트로 복원하고 추가 테스트2개를작업트리에서제거했다.
수정 diff와테스트사본은Git제외artifact의rolled-back-changes에보존했다.
따라서 앞선 성능·테스트 통과 기록은 채택된 수정 완료 결과가 아니다.
시작점옵션도제거했으며재녹화는원래launch와알고리즘으로진행한다.
기존영상·bag·검증기록은보존했고commit/push/merge는수행하지않았다.


## 16:21 KST 원본 코드로 재녹화 시작

사용자가시나리오·P·UDP재연결완료를알리고재녹화를승인했다.
원본코드drive=false검증에서센서·위치·목표정상,제어UDP0을확인했고현재s16.19m였다.
완전히복원한HEAD2d34dc2코드로drive=true/bag=true를실행했다.시작점옵션은없다.
새폴더`docker/artifacts/midcheck-20261016/rollback-retake-20261016-161848/`의
CTRL.mkv·diagnostics.bag와기본bag·로그를기록중이다. 실제s200m까지진행을확인했다.
기존원본영상과폐기한수정본영상은모두보존했다.현재녹화종료·MP4마감은아직하지않았다.


## 롤백 재녹화 종료 및 최종 분석

사용자 요청으로 원본 재녹화도 종료하고 Manual/P 인계를 확인했다.
첫 제출용 CTRL.mp4와 CTRL_sensor.json을 다운로드/CTRL_제출_20261016에 복사했다.
원본 코드 일치와 최대98.545초 target 공백, 마지막 red 정지 및 폐기한 수정본의
늦은 회피 진입·BEV UNOBSERVED 정지는 [최종 분석](ROLLBACK_STOP_ANALYSIS_2026-10-16.md)에 기록했다.
추가 로직 수정이나 주행은 수행하지 않았다.
