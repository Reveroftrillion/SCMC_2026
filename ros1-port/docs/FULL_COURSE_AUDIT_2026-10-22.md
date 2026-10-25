# 현재 수정본 전체 코스 시험 — 2026-10-22

최종 결과: 두 번째 실제 MORAI 시험에서 START→제공된 모든 CP→END를
순서대로 통과하고 FINISH로 종료했다. 충돌 채널의 유효 객체 충돌0,
실제 정지·Manual/P 인계를 확인했다. 보행자 전용 판단의 완료 여부는
검증되지 않았으며, 이 완주는 공식 채점 또는 모든 조건의 미션 성공을 뜻하지 않는다.

## 개발 목표·범위

사용자가 같은 sample_scene 시작점에서 drive=false 점검 후 drive=true 전체 코스
시험과 실패 분석·최소 수정을 명시 승인했다. 실제 차량은 대상이 아니다.
정적 우회·CP4·복귀의 10/21 단일 성공과 터널 오프라인 재현을 현재 기준으로
사용하며, 옛 별도 구간 통과를 이번 완주 결과와 합산하지 않는다.

시작 브랜치 `cleanup/unused-files-20261022`, 시험 브랜치
`test/fullcourse-20261022`, HEAD `0fe5ae635f5acbc79aae9e9c7f88a3474a63b233`.
미추적 static_course.py와 기존 수정·삭제를 포함한 2,678개 파일 및 삭제4개의
복구 스냅샷을 `docker/artifacts/fullcourse-20261022-130819/`에 보존했다.
incoming-source.tar.gz SHA256:
`4c5f55b53284f529b85979b325041f3c6566a5e0e689026e1c04d9f58cce7bbc`.
incoming.patch, incoming-files.json, incoming-deletions.json, incoming-status.txt,
snapshot.json을 함께 사용한다. 기존 과거 기록은 덮어쓰지 않는다.

영향 파일: 이 문서, 외부 감독 도구 audit_course.py/course_audit_safety.py와
검사. production 수정은 주행 종료 후 새 실패의 재현 근거를 확보할 때만 한다.
주행 중에는 source manifest를 동결한다.

방법: 기존 실행/포트 확인 → raw Status·충돌 채널 수신 → 짧은 native
drive=false → 시작점/Manual/P 확인 → 원본 기록기 선기동 후 전체 시험 →
충돌/이탈/무진행/입력 또는 기록기 유실 시 종료 → 실제 정지·Manual/P 확인 →
같은 연속 입력 분석·회귀. 완료 기준은 도달/미도달 구간, CP·미션 결과,
실패 위치/원인·수정 여부·검증 한계를 재현 명령과 함께 남기는 것이다.

## 시험 전 감독 조건

- 충돌: 실제 CollisionData 객체 슬롯(type>=0, id!=0) 즉시 종료.
- 이탈: route 거리8m 초과, 또는 지도 밖이며 route 거리3.5m 초과가1초 지속.
  지도 내 정상 정적 +4m 우회를 그 자체로 이탈 처리하지 않는다.
- 무진행: 120초간 진행거리 증가1m 미만. 전체 제한1,500초.
- 입력: 기동 유예20초 후 raw Status 또는 충돌 채널1초 유실,
  IMU/LiDAR/Camera/odometry 3초 유실 시 종료. GPS fix 단독 유실은
  터널 DR을 허용하되 모드와 원시 입력을 기록한다.
- launch/recorder 종료·관측기 오류 시 종료. 제어/센서/카메라 원본 pcap,
  raw/debug JPEG·인지/계획/제어/Status/TF bag 및 YOLO 로그를 새 폴더에 보존.
- 종료: bridge의 기존 SIGINT 제동·인계 후 서로 다른 Status stamp의
  Manual(mode1)/P(gear1)·|speed|<0.01m/s 연속 근거를 확보한다.

기존 감독 도구가 충돌 채널 유실과 IMU 유실을 감시하지 않는 경계를 확인해
위 감시와 회귀 검사를 외부 도구에만 추가했다. 차량 정책·threshold·watchdog·
속도 제한·TF·경로·메시지·포트에는 변경이 없다.

## 진행 기록

기존 ROS/master/주행·기록기 컨테이너 없음. MORAI PID32532 실행 중,
시험 자료 디스크 여유 약1.1TiB. 최초 Status9088/Ego9093 연결이 없었으며
사용자가 재연결 완료를 회신했다. 현재 세션 수신 결과는 아래에 추가한다.

### 최초 수신 점검

raw 4초: Status165개·Collision166개, parse 오류0, 객체 충돌0, 전부
Manual/P 및 |speed|<0.01m/s. native drive=false25초: PASS, 오류0,
제어 UDP0, launch exit0, target 약20Hz, 위치/TF 정상. 단, 실제 후륜
s=9.2766m여서 START 반경 밖이었다. 주행하지 않고 사용자에게 시작점
재배치를 요청했다. preflight-01/live.json과 passive-start-01.json 참조.

필수 인지 footprint freshness와 필수 노드8개의 master 등록도 외부 감독에
추가했다. 신호 구간 밖에서도 yolo 노드 종료를 감지하고, 실제 publish 유실은
입력 freshness/launch 종료와 함께 감시한다. master 등록은 프로세스 ping을
대신하지 않으므로 등록만 남은 정지 프로세스의 완전한 검출 증명은 아니다.

사용자 재배치 후 raw4초: Status162개·Collision163개, Manual/P·정지, 충돌0.
현재 세션 Player.log의 마지막 장면은 Practice_cwy.json이었고 파일 비교에서
공식 sample_scene과 NPC 초기 배치가 달랐다. 정확한 sample_scene 파일 로드를
요청했다. scene-preflight.json 및 player-before-drive.log에 초기 관측 보존.
정적/신호/지면/터널 관련 기존 회귀30 PASS, 최종 외부 감독7 PASS.
현재까지 drive=true 실행 및 차량 이동 명령은0회다.

### 시작점 오투영 재현과 선택적 수정

공식 sample_scene 로드 후 preflight-02도 센서/TF/target·UDP0은 통과했지만,
실제 후륜 XY=(-132.53060,-428.99823), START 거리1.07337m가 경로 끝
s=2173.56248m로 투영됐다. 시작/끝이 같은 폐루프이고 후륜이 첫 점보다
조금 뒤에 있어 최근접 끝 segment가 선택되는 조건이다. 기존 finish runout을
포함한 planner의 offset 추정도 이 조건에서 약-61m로 어긋날 수 있었다.
따라서 drive=true로 잘못된 마지막 구간을 달리지 않고 수신 시험을 종료했다.

최소 수정: localization/follower/behavior_node에 기본false인
start_at_beginning 옵션을 추가했다. 이번 전체 코스 시험의 from-start.launch
사본에서만 세 노드에 true를 명시한다. 새 route_start.py는 제공 checkpoints의
반경3m를 읽고, 폐루프 global 경로 시작점 안에 있는 유한한 초기 위치만
허용한다. localization과 follower의 첫 투영만 s0 근처로 초기화하고 이후
기존 연속 투영·제어를 쓴다. planner는 같은 global 원점 offset0을 쓴다.
정상 END 판정·중간 재시작·기본 launch 동작은 default false로 보존한다.

원본 경로/CP 좌표/TF/속도/신호/watchdog는 바꾸지 않았다. 감독 도구와
verify_live에는 선택적 launch-file 인자만 추가하여 기존 launch를 포함하는
시험 사본을 사용할 수 있게 했다. 실제 기록 좌표·연속 초기 경로·잘못된
위치/열린 경로 거부·기본 offset/정적 회귀를 새 검사와 함께 확인한다.

수정 후 회귀134 PASS, preflight-03 native drive=false25초 PASS: 실제
START 거리1.07337m, localization s0, follower s0 HANDSHAKE, target/TF
정상, UDP0·오류0·launch exit0. 두 신호/정적 실제 통과는 이 검사로
대체하지 않는다. 이어 run-01에서 from-start.launch를 사용해 전체 주행한다.

## run-01 종료: 둘째 신호 녹색 미채택

실제 Auto/D로 s0→309.6974m. 120초간1m 미만 진행 조건으로 종료,
총155.65초. source-before/after 동일, child 종료코드 모두0, callback 오류0.
원본 Collision6,474개 수신. 종료 후 Status31개/0.666초에서 Manual/P,
|speed| 최대0.000002813m/s. Pause 대신 실제 상태로 인계 확인.

원본 Camera-7에서 실제 녹색을 확인했고 같은 시각 YOLO는 green 약0.96,
정규화 중심(0.364,0.144)를 검출했다. 동시에 하단(0.871,0.938)의
green_left 약0.59 오검출이 현재 ROI에 들어왔다. 서로 다른 색을 혼합하지
않는 기존 choose_signal이 None을 반환해 이전 적색 STOP 기억이 유지됐다.
목표는 계속 발행됐고 신호 기억/watchdog를 우회하지 않았다.

같은 교차로 로그2,604개에서 하단 중심y[0.938,0.945]의 오검출을
제외하는 후보 ROI [0,0,0.9,0.9]는 green571개를 복구했다. 로그 중심값은
소수3자리이므로 이 결과만으로 채택하지 않고 연속60초 원본 JPEG를 GPU로
재추론해 경계를 확인한다. 다른 교차로/모델/신호 정책은 바꾸지 않는다.

### run-01 근거·신호 최소 수정

START 최소거리0.2702m, CP1/2/3 각각0.4666/0.0296/0.1373m 통과.
CP4 이후·정적 우회·보행자·회전교차로·고속도로·터널·END는 미도달.
첫 신호 green23개로 통과, 둘째는 red761개·green0으로 STOP.
target 최대 간격0.11514초, signal age 추가차이 최대0.10737초.
NO_TARGET11행은 초기 준비 구간이며 정지의 원인은 신호 STOP이다.
원본 충돌6,474개 중 유효 객체 충돌0·파싱 오류0.

원본 연속60초1,263 JPEG의 CUDA 추론에서 기존 ROI는 red249/None1011/
yellow3, 후보는 red629/green571/yellow63이었다. 기존 red/yellow를
green으로 바꾼 경우0, 연속2프레임 green 확인 가능. 원본 JPEG sequence
SHA256은12a35a51c399e67c71f35e1eab6f89d3b8fb4c47e5808d56074228abf38dc366.

config/yolo_stoplines.json의 intersection_02 roi[3]만1.0→0.9로 변경했다.
실제 녹색 box는 y50.50..88.10px, 하단 도로 오검출은 y420.82..479.55px
(중심 약0.938)였다. selector의 중심 기준에서 하단10%를 제외하며 실신호
box·기존 x범위·다른 교차로·모델 confidence·색 충돌 정책·적색 기억은 보존한다.
기록 검사의 수정 전2 FAIL/2 PASS를 signal-before-fix.log/json에 남겼다.
다른 조건/교차로에서의 일반적인 오검출 해결로 확대 해석하지 않는다.

## run-02: 실제 전체 코스 완주

사용자가 같은 sample_scene을 다시 로드한 후 raw Status/충돌 수신과
Manual/P·정지를 확인했다. 수정 ROI와 시작점 옵션을 적용한 native
drive=false15초(preflight-04)는 s0, START 거리0.86696m, 센서·TF·target,
기동부터 종료까지 제어 UDP9093 송신0개로 통과했다. 그 뒤 새 run-02에
기록을 시작하고 drive=true를 실행했다. 두 실제 시험 모두 주행 중 source
SHA256 manifest가 동일했고 callback 오류0, 자식 종료코드는 모두0이었다.

| 시험 | 적용 설정 | 실제 도달·종료 | 실패·미검증 |
| --- | --- | --- | --- |
| run-01 | 시작점 옵션true, 기존 intersection_02 ROI | s309.6974m, 120초 무진행 종료 | 녹색 미채택 실패. CP4 이후와 END 미도달·미검증 |
| run-02 | 시작점 옵션true, intersection_02 ROI 하단0.9 | 전체 CP·END 통과, follower_finish | 미도달 구간 없음. 보행자 전용 완료·다른 조건은 별도 미검증 |

run-02 감독 총279.277초, bag272.845초. START와 END 반경에 순서대로
처음 들어간 시각 차이는266.460초(약4분26초)다. 이는 로컬 후륜축 위치의
반경 진입 시간이며 MORAI 공식 기록이 아니다. localization의 원래 경로
진행거리는2174.552m까지, follower는 기존 종료 runout을 포함해
약2234.552m에서 FINISH를 기록했다.

### CP 근거

START/END가 같은 XY이므로 진행거리 구간과 통과 순서로 분리했다.
제공 checkpoints.txt에는 CP11이 없다. 각 반경3m, 아래 수치는 해당
진행거리 구간 내 후륜축 odometry 최소거리다. 공식 차량 중심/채점과의
일치는 별도 확인 대상이다.

| 지점 | 최소거리(m) | 결과 |
| --- | ---: | --- |
| START | 0.180 | 통과 |
| CP1 | 0.478 | 통과 |
| CP2 | 0.071 | 통과 |
| CP3 | 0.142 | 통과 |
| CP4 | 1.037 | 통과 |
| CP5 | 0.034 | 통과 |
| CP6 | 0.243 | 통과 |
| CP7 | 0.116 | 통과 |
| CP8 | 0.066 | 통과 |
| CP9 | 0.084 | 통과 |
| CP10 | 0.245 | 통과 |
| CP12 | 0.804 | 통과 |
| CP13 | 0.050 | 통과 |
| CP14 | 0.232 | 통과 |
| CP15 | 0.189 | 통과 |
| END | 0.122 | 통과 |

### 미션별 관측과 한계

| 구간 | 이번 시험의 근거 | 남은 확인 |
| --- | --- | --- |
| 신호 | 둘째 신호 실제 green20개를 채택하며 기존 무진행 위치를 통과. 첫째/둘째 정지 후 출발 관측 | 다른 시점·광원·가림 조건. 교차로4 red_left는 기존 좌회전 허용 정책의 색이며 red 단독과 구분 |
| 정적 s270.97→388.74 | AVOID, CP4 통과, s354.30에서 경로 오차0.014m로 복귀 | 다른 배치·첫 포착 위치·반복 시험 |
| 보행자 s403.96→493.57 | 구간과 CP5 무충돌 통과, YIELD17개 | 전용 보행자 분류/양보 완료는 미검증: 아래 분석 참조 |
| 회전교차로 s801.71→908.87 | ROUNDABOUT_WAIT52/ENTER150 상태와 통과 | 다른 진입 차량 간격의 진입·양보 판단 |
| 고속도로 s1100.78→1732.26 | WAIT_GAP→CHANGE→SETTLE→FOLLOW→REJOIN, REJOIN_COMPLETE117개 | 선택된 선행차 모델의 gap 최솟값62.994m·desired gap 미만0개이며 모든 차량의 실제 최단거리 증명은 아님 |
| 터널 s1808.19→1990.76 | 실제 통과, GPS→DR(s1873.77)→GPS(s1989.66), DR8.519초 | 다른 장애물·보행자 배치와 터널 안 재시작 |
| END | 순서상 마지막 END 반경 진입 후 runout FINISH | 공식 채점 판정 별도 |

충돌 원본11,385개 중 유효 객체 충돌0, wire 파싱 오류0. route 거리 최대
3.9869m는 정적 구간의 계획된 +4m 우회와 일치한다. 감독 이탈 조건은
발생하지 않았다. 관측 최대속도121.398km/h는 고속도로 구간이며 모든
구간의 속도 규정 적합성 판정으로 확대하지 않는다.

2초 이상 정지한 에피소드는 첫째 신호80.600초(s202.34), 둘째 신호
18.600초(s309.70)였다. target 최대 간격0.14932초, 1초 초과0개.
NO_TARGET16행은 기동 초기이며 이후 지속 손실은 관측되지 않았다.
추가 signal age 차이 최댓값0.05604초. 이번 부하에서 관측된 수치이며
오래된 입력/timestamp 설계나 카메라 gate가 모든 상황에서 해결됐다는
증명은 아니다. 보행자 구간에 약0.080초 DR→GPS 복귀도 별도로 있었다.

### 다음 우선순위: 보행자 분류·전용 양보

보행자 미션 내부에서는 PEDESTRIAN 이벤트0, pedestrian_yield_completed
false였다. 실제 YIELD17개(약0.8초)는 DYNAMIC_UNKNOWN을 대상으로 한
VEHICLE_GUARD였으며 OCCUPIED→CLEAR_CONFIRMATION으로 진행했다.
따라서 위험물 방어 반응은 있었지만 보행자 전용 미션 성공으로 판정하지 않는다.

해당 track은 s약459m, d약+0.8m 부근, 관측 center x가 -59.79→-65.78m,
y약-10.4m, 속도약-2.75m/s, 높이약2.2m였다. 제공 보행자 초기 s약458m와
가깝지만 원본 영상·시뮬레이터 객체 ID와의 대응을 확정하지 못했다.
고속도로 구간에는 PEDESTRIAN 이벤트5개가 있어 오분류 여부도 함께
확인해야 한다. 단순 threshold 조정이나 보행자 강제 분류는 하지 않았다.
원본 bag, pedestrian-probe-v2/context.json과 JPEG4장을 보존했다.

다음 작업은 동일 track의 원본 pointcloud→군집/추적→종류→road_users/
events→YIELD 입력을 연속 재생하고 실제 객체와 대응시켜 실패 경계를
확정하는 것이다. 그 다음 동일 sample 반복성과 보행자/회전교차로/고속도로/
터널 배치 변화 시험을 우선한다. 이번 장면의 pedestrians1, vehicles9,
staticObjects1 조건을 다른 장면의 성공으로 합산하지 않는다.

### 종료·인계·기록

run-02 종료 후 서로 다른 stamp의 Status31개/0.669초에서 모두
Manual(mode1)/P(gear1), |speed| 최대0.001107m/s를 확인했다. 마지막
raw Status 속도는 -0.000000283m/s였다. 수치 허용오차 내 실제 정지로
판정했으며 Pause만으로 판정하지 않았다. 자체 launch/master/기록기는
종료됐다. 최종 host 확인에서도 실행 중 Docker 컨테이너와 자체 ROS/
기록기 프로세스, 수신909/9092 소켓이 없었다. MORAI 자체 PID38437과
MORAI의9093 수신 소켓은 그대로 남겼다.

원본은 run-01/run-02의 bag, sensors-control.pcap, wire.jsonl,
timeline·follower CSV, YOLO JSONL, source-before/after.json, settings.json,
end.json 및 launch/ROS 로그다. YOLO 로그 중 파일명 끝에 newline이 있는
경우가 있어 원본 이름을 보존하고 glob으로 읽었다. raw/debug 카메라 모두
bag에 보존했으며 별도 재생 영상과 원본 시각 CSV도 생성했다.

## 변경 파일과 최종 검증

| 파일(ros1-port 기준) | 이번 작업 내용 |
| --- | --- |
| src/foscar_control/foscar_control/route_start.py | 명시한 첫 랩 초기화의 위치·폐루프·반경 검증 |
| src/foscar_control/foscar_control/localization.py | 첫 투영의 선택적 s0 hint |
| src/foscar_control/foscar_control/follower.py | pose 수신 후 동일 hint로 초기화, 기존 runout 보존 |
| src/foscar_planning/foscar_planning/behavior_node.py | 같은 옵션에서 global 원점 offset0 |
| src/foscar_control/test/test_route_start.py | 실제 오투영 위치·연속 진행·거부 조건·기본 동작8개 |
| src/foscar_camera/config/yolo_stoplines.json | intersection_02 ROI 하단만1.0→0.9 |
| src/foscar_camera/test/test_signal_roi_recording.py, test/data/signal_roi_20261022.json | 원본 box·연속 프레임 기반 실패 재현4개 |
| docker/audit_course.py, course_audit_safety.py, test_course_audit_safety.py | 필수 입력/노드 유실 감독·회귀·선택적 launch 경로 |
| docker/verify_live.py | drive=false용 선택적 launch 경로 |
| docs/FULL_COURSE_AUDIT_2026-10-22.md, DEVELOPMENT_WORKFLOW.md | 이번 결과와 현재 재현 경로 |

기존 static_course.py, ground/obstacle/avoid 수정, behavior.yaml, 삭제4개,
기존 실험 문서와 JSON/manifest는 이번 기능 수정 대상으로 바꾸지 않았다.
새 검증은 final-checks의 격리 src/build/devel/install에서 수행했고, 비교
기준 원본에는 테스트/캐시를 생성하지 않았다. commit/push/merge는 하지 않았다.

| 검사 | 결과 |
| --- | --- |
| control/planning/lidar/camera 알고리즘 회귀 | 939 PASS, 실패0·skip0(정적/터널 포함) |
| replay·감독 도구 | 13 PASS |
| ROS1 adapter | 27 PASS |
| native 메시지 직렬화/TF | PASS |
| 격리 catkin build/install | 각각 PASS |
| native entrypoint import9개, launch 노드/파일 정적 확인 | PASS(노드 기동 없음) |
| packaging | FAIL: 기존 msg CRLF/LF raw hash 차이 |
| packaging-unit | 7 PASS/1 ERROR: 기존 behavior.yaml 및 이번 의도된 ROI resource hash 차이 |
| identifier-ast, boundary, preflight | FAIL: 과거 식별자 정리 기준과 기존 기능 수정 불일치, 이번 초기화 기능 변경도 boundary에 표시 |
| devel/install 환경 검사 | 각각9 PASS/1 FAIL(resources)/1 KNOWN_SOURCE_ISSUE(model README hash) |

FAIL을 통과로 바꾸기 위해 원본·기대값·과거 SHA manifest를 갱신하지 않았다.
이번 ROI의 resource hash 차이는 새 변경에 따른 명시적 불일치이며,
behavior.yaml/모델 README/메시지 줄바꿈 문제와 구분한다. Windows 환경의
회귀는 이번에 실행하지 않았다. packaging guard가 깨끗하게 통과하는 배포
호환성은 남은 문제다. 상세 로그는 final-checks/checks/results.json과
각 .log에 보존했다.

마지막 보존 검사에서 incoming2,678개 중2,669개는 byte 동일, 이번 작업
수정9개, 누락0개였다. 신규 파일은5개다. 기존 미추적 static_course.py와
날짜별 기록·검증 근거도 동일하다. 수정/신규 Python10개 구문 검사와
현재 두 문서의 상대 링크21개 확인을 통과했다. 기본 git diff --check는
CRLF 끝의 CR을 whitespace로 보고 exit2였고, CRLF를 허용하는 동일 검사
(`git -c core.whitespace=cr-at-eol diff --check`)는 PASS다.
final-preservation.json과 두 diff 로그, final-host-state.json에 보존했다.

## 재현과 복구

자료 루트: `ros1-port/docker/artifacts/fullcourse-20261022-130819/`.
이 디렉터리는 ignored 원본 자료다. 다른 PC로 넘길 때 아래 launch 사본과
센서/검증 자료를 따로 보존한다. start_at_beginning 기본값은false이므로
일반 local.launch만으로 이번 첫 랩 초기화가 활성화되지는 않는다.
공식 시작점에 재배치했을 때만 from-start.launch를 사용한다.

MORAI는 같은 sample_scene 로드, 시작점·Pause 해제·Manual/P,
Status9088→909/Ego9093/Collision→9092 연결이 필요하다. 명령 전 기존
master/제어 노드/기록기가 없는지 확인하고 매번 새 출력 경로를 쓴다.
아래는 저장소 루트에서 실행하며 두 번째 명령은 실제 시뮬레이터 주행이다.

```bash
bash ros1-port/docker/run.sh --receive-root --capture-control bash -lc '
  source /opt/ros/noetic/setup.bash
  source /workspace/ros1-port/devel/setup.bash
  python3 docker/verify_live.py --seconds 15 \
    --launch-file /workspace/ros1-port/docker/artifacts/fullcourse-20261022-130819/from-start.launch \
    --output /workspace/ros1-port/docker/artifacts/fullcourse-20261022-next/preflight/live.json
'
```

drive=false 결과의 시작점·센서/Status/TF/target·UDP0을 확인한 뒤 실행한다.

```bash
bash ros1-port/docker/run.sh --receive-root --capture-control bash -lc '
  source /opt/ros/noetic/setup.bash
  source /workspace/ros1-port/devel/setup.bash
  python3 docker/audit_course.py \
    --launch-file /workspace/ros1-port/docker/artifacts/fullcourse-20261022-130819/from-start.launch \
    --output-dir /workspace/ros1-port/docker/artifacts/fullcourse-20261022-next/run-01
'
```

이번 기록의 오프라인 분석만 다시 하려면(주행 없음):

```bash
bash ros1-port/docker/run.sh bash -lc '
  source /opt/ros/noetic/setup.bash
  source /workspace/ros1-port/devel/setup.bash
  python3 tools/analyze_course_audit.py \
    --run docker/artifacts/fullcourse-20261022-130819/run-02 \
    --output docker/artifacts/fullcourse-20261022-next/analysis-02
'
```

주요 결과 바로가기:
[전체 주행 영상](../docker/artifacts/fullcourse-20261022-130819/analysis-02/camera.mp4),
[영상 원본 시각](../docker/artifacts/fullcourse-20261022-130819/analysis-02/camera-timestamps.csv),
[전체 기록 분석](../docker/artifacts/fullcourse-20261022-130819/analysis-02/summary.json),
[CP·미션 순서 분석](../docker/artifacts/fullcourse-20261022-130819/mission-analysis-02-v2/summary.json),
[완주·정지 근거](../docker/artifacts/fullcourse-20261022-130819/run-02/end.json),
[최종 검사](../docker/artifacts/fullcourse-20261022-130819/final-checks/checks/results.json).

분석 첫 버전의 분리된 CRUISE 구간 간격 합산 문제와 보행자 probe의 메시지
타입 가정 실패도 원본 출력에 남겼다. 수정한 별도 v2 출력만 해당 판단에
사용했다. 체크포인트 수치는 위 mission-analysis-02-v2를 기준으로 한다.

복구 시 incoming-source.tar.gz를 먼저 별도 빈 디렉터리에 풀어
incoming-files.json의 SHA256과 비교하고 필요한 파일만 복원한다.
incoming-deletions.json에 있던 삭제4개는 원래부터 삭제 상태이므로
되살리지 않는다. 작업 트리 전체 reset/clean/덮어쓰기로 복구하지 않는다.
