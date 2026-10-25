# 로터리 s868.79m 대기 재현과 수정 — 2026-10-23

## 개발 목표·범위·완료 기준

사용자가 주행 중 로터리 정지를 보고하며 수정을 요청했다.
브랜치는 `fix/roundabout-wait-20261023`이며 이전 호스트 실행·기록·종료 수정은 보존한다.

- 목표: 현재 입력으로 정지의 실패 경계를 재현하고 원인만 최소 수정한다.
- 영향 후보: LiDAR 지면/검출, 로터리 점유·합류 판단과 관련 회귀 검사.
- 방법: 실행 중 코드는 보존하고 읽기 전용 로그·target/prediction·raw 프레임을 확보한다.
  사용자 Ctrl+C·정지/Manual/P 확인 및 실행 컨테이너0 확인 후 오프라인에서 수정한다.
- 검증: 수정 전 같은 raw의 지면 오탐·로터리 점유·진입 거부 재현,
  수정 후 비교, 실제 차량 보존·기존 원인 관련 회귀·기존 성공 입력 비교.
- 완료 기준: 원인·재현 명령·소스/입력 보존·검사 결과를 기록한다.
  오프라인 통과와 이후 실제 로터리 재시험을 구분한다.

## 확보한 실패 경계

현재 기록은 `docker/artifacts/ctrl_start_foscar_7_2323905054591028057/`이다.
새 진단 자료는 `docker/artifacts/roundabout-stop-20261023-145847/`이다.
기존 src SHA256·Git 상태를 보존했고 기존 과거 자료·검증 JSON은 수정하지 않는다.

실제 위치는 약[-99.4455,310.5540], s868.786m, 속도 약0이다.
upstream은 CRUISE/v_max16.67이며 신호/YIELD가 아닌 로터리 box 판단이
ROUNDABOUT_WAIT/v_max0을 만든다. prediction은 TUCK_WAIT/launch_in=null이다.
RB_TUCK_EVAL은 지면 반사 위치의 후보를 lead로 보고 no_launch/lead_gap을 출력했다.
일부 판단 계산의 시작~출력은 약0.5~1.7초이며 NO_TARGET도 교차한다.
이 지연을 보정 없이 완화하거나 watchdog을 늘리지 않는다.

실행 중 읽기 전용 구독으로 raw 7,429점과 위치·mission·footprint를 확보했다.
프레임 stamp1792735267.269065, footprint 수신 당시 age 약0.116초다.
같은 raw의 수정 전 계산은 평면None·검출28개다. 대표 후보는 점 높이0.53m,
두께 약0.001m, 차선 재질 비율80~100%의 납작한 반사다.
아스팔트418개는 기존 재질 평면 검증을 통과하며 평면은 약
[0.003719,-0.004123,0.468511]이다. 기존 재질 fallback은 ROUNDABOUT을 포함하지 않는다.

이후 사용자가 Ctrl+C 종료·정지/Manual/P를 확인했고 실행 컨테이너0을 확인했다.
사용자의 MORAI Reset이나 새 실제 주행은 이번 원인 분석의 일부로 실행하지 않는다.

## 최소 수정과 오프라인 결과

obstacle_node.py에서 기존 재질 평면 fallback의 mission 목록에 ROUNDABOUT을 추가했다.
동일한 broad asphalt·기울기·절편·잔차·지원영역 검사와 현재 height window를 사용한다.
지면 평면이 이미 있으면 바꾸지 않으며 mission이 오래됐거나 intensity/지지점이 없으면
기존 고정 높이를 유지한다. 로터리 후보·안전 간격·대기 시간·ROI·속도·watchdog는 바꾸지 않았다.
기존 src 변경은 이 파일1개이며 새 회귀 검사 test_roundabout_material_ground.py를 추가했다.

- 같은 raw 프레임: 평면None→검증된 아스팔트 평면, 검출28→14.
  차선 재질의 납작한 검출을 제거했고 일부 ROI 밖/미지원 영역은 보수적으로 보존한다.
- 신규 회귀: 수정 전1 FAIL/2 PASS, 수정 후3 PASS. 높이0.7m·강도0 실제 물체를 보존하고
  원본 cloud 불변, 재질/지원 부족·만료된 mission·다른 mission의 fallback을 확인했다.
- 관련 기존 검사와 합쳐8 PASS/1 FAIL. 전체 격리 알고리즘은952 PASS/1 FAIL,
  skip0·경고10·339.04초다. 신규3개는 전체에 포함되므로 합산하지 않는다.
- 유일한 실패는 기존 test_tunnel_intensity.py:75의 ROUNDABOUT 고정 높이 정책이다.
  이번 사용자 요청의 로터리 보정과 의도적으로 충돌한다. 기존 검사/기대값은 수정하지 않았다.
  전체 회귀가 모두 통과했다고 표현하지 않는다.
- 별도 기본 bag과 raw bag의 로터리 접근/정지24초를 새 window.bag으로 묶어207스캔을 추출했다.
  원본 bag은 보존했다. 추출 manifest와 전후 원본 검출/추적/footprint 계산을 보존한다.
- 부분 로터리 판단 진단의 최초 실행은 누락된 mission.config로 실패했다.
  실제 설정 경로를 넣은 v2/v3는 전후 모두 출발을 허용하는 구간이 있어 실제 장기 대기를
  완전히 재현하지 못했다. 초기 관측/진입 상태·실제 executor 지연이 같은 증거가 아니므로
  이 재생을 실제 정지 해결의 증거로 사용하지 않는다.

## 실제 재시험 준비

사용자가 START Reset·Manual/P·채널 재연결을 완료했다고 확인했다.
별도 preflight에서 drive=false·UDP9093 송신0·센서/TF/target·종료를 확인한 뒤,
통과할 경우 기존 audit_course.py 감독으로 로터리 뒤 s940m까지 시험한다.
충돌·입력 손실·무진행 감독과 종료 후 실제 정지/Manual/P 판정을 유지한다.
수정 소스 SHA256은 fixed-source.json에 고정했다. 실제 결과는 수행 후 별도로 기록한다.

## 수정 후 실제 감독 시험 결과

수신전용25초 검증은 센서·TF·target PASS, 제어 UDP9093 송신0, launch exit0이었다.
사용자가 확인한 Reset·연결 상태에서 수정본을 실제로 주행했다.
run-01은 요청한 구간 끝 s941.452m에서 requested_segment_end로 종료됐다.
총221.046초는 시작 준비·신호 대기·종료를 포함하며 공식 주행 시간이 아니다.

- CP1~9가 각각 제공 반경3m 안을 통과했다. 정적 우회·실제 신호 대기/재출발·
  보행자 구간 후 로터리의 기존 정지 s868.79m를 지나 진입·출구·CP9를 통과했다.
- ROUNDABOUT 시작 s801.986→종료 s908.664m. 기록된 로터리 prediction 범위는10.269초다.
  target은 정상 대기 후 ROUNDABOUT_ENTER로 진행했고 전체 최대 간격0.101336초,
  1초 초과 공백0이다. 이 결과로 모든 부하/시간 조건의 지연 해소를 주장하지 않는다.
- 원본 충돌9,293패킷을 다시 decode해 유효 충돌0·decode 오류0을 확인했다.
  공식 차선·점수 판정과 구분한다.
- 종료 후 fresh Status31샘플/0.666초에서 Manual/P·정지 확인,
  구간 내 최대 절대 속도0.00954m/s, 최종 속도 약0이다.
- 검사한 src344개 해시 일치, 감독의 source_unchanged=true, callback 오류0,
  master/recorder/launch exit 모두0, 최종 실행 컨테이너0이다.
- 새 기록의 *.log에서 카메라 ExternalShutdownException traceback은 발견되지 않았다.
  이전 실패 이력과 오프라인 검사 결과는 삭제하지 않는다.

이번 결과는 같은 조건의 **로터리까지 부분 주행1회**이며 전체 코스 재완주·다른 NPC 배치·
공식 차선/감점·Windows·catkin 재빌드/설치의 성공으로 확대하지 않는다.
수정 전 실제 장기 정지는 raw와 진단으로 보존했고, 부분 재생의 초기 상태 한계도 유지한다.
기존 원본/식별자/리소스 검사 실패와 전체 회귀의 정책 충돌1개는 남아 있다.
이번에는 커밋·push·main 병합을 하지 않았다.
최종 구문9개·JSON2개·현재 문서 링크240개·git diff --check를 통과했다.
고정 소스344개·추출 manifest221개 일치, 비교 원본530개 Git blob 불변을 확인했다.

## 재현과 다음 시작

```bash
# 저장소 루트. 계산용 ROS 대역만 사용한 현재 회귀 기록은 artifacts의 run_regression.py와
# regression.log를 참고한다. 아래는 저장한 raw 한 프레임 진단이며 실제 제어를 하지 않는다.
bash ros1-port/docker/run.sh env PYTHONDONTWRITEBYTECODE=1 \
  python3 tools/replay_lidar_ground.py inspect \
  --cloud docker/artifacts/roundabout-stop-20261023-145847/live-frame/cloud.npz \
  --context docker/artifacts/roundabout-stop-20261023-145847/live-frame/context.json \
  --output docker/artifacts/roundabout-stop-20261023-145847/new-frame-analysis.json
```

출력은 매번 새 이름을 쓴다. 원본 bag과 대용량 진단은 로컬 artifacts에 남기며
[공유 요약](roundabout-20261023/summary.json)을 보존한다.
다음 사용자 주행은 MORAI를 원래 시작점 Reset·Manual/P·채널 재연결 후
기존 `roslaunch ctrl_start.launch`로 실행한다. 현재 차는 로터리 뒤에서 종료됐다.
