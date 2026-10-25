# 동일 장면 반복 안정성 시험 — 2026-10-22

같은 sample_scene에서 감독하에 **2회 모두 START→제공 CP 전체→END를 통과**했다.
두 시험 모두 원본 CollisionData의 유효 객체 충돌0, 종료 후 fresh Status의
정지·Manual/P 인계를 확인했다. 3차는 2차 종료 후 사용자가 생략하도록 요청했다.
공식 점수·차선 감점이나 다른 배치의 성공을 뜻하지 않는다.

## 목표·범위·보존

- 기준 main `55dfec7`, 작업 브랜치 `test/morai-stability-20261022`.
- 원격 main도 같은 SHA임을 시작 전에 확인했다.
- 2,702개 소스/설정/미추적 파일과 Git 상태를 새
  `docker/artifacts/stability-20261022-170834/`에 보존했다.
  `incoming-source.tar.gz`, `incoming-files.json`, 상태/patch/refs를 함께 사용한다.
- `drive:=true`, `roslaunch` 빈 파일, 비교 원본, 과거 검증 기록을 보존했다.
- 생산 인지·계획·제어·경로·파라미터는 변경하지 않았다. 두 시험의 소스 manifest는
  동일하며 각 시험 전후도 동일하다. 주행 중 개입·코드 변경 없음.
- 방법: fresh 센서/Status → drive=false·UDP0 → 기록기 준비 확인 → 감독 주행 →
  정지·인계 → 원본 분석·연속 재생·관련 회귀. 완료 기준은 동일 장면의 두 결과,
  실패 재현 경계, 기록 한계와 다음 우선순위를 남기는 것이다.

## 실제 결과

| 항목 | run-01 | run-02 |
| --- | --- | --- |
| 시작 후륜 위치의 START 거리 | 약1.50m | 약2.59m |
| START/CP/END | 전부 순서대로 통과 | 전부 순서대로 통과 |
| CP4 최소거리 | 약0.114m | 약0.190m |
| 실제 우측 최대 offset | 약3.265m | 약3.265m |
| 보행자 전용 양보 완료 | s450.34m | s450.53m |
| 터널 | 통과, GPS→DR→GPS | 통과, GPS→DR→GPS |
| DR 구간 | s1871.93→1988.96m | s1874.30→1989.83m |
| target 최대 간격 | 0.14142초 | 0.14303초 |
| 원본 충돌 패킷 / 유효 충돌 | 13,959 / 0 | 14,873 / 0 |
| wire 해석 오류 / 감독 callback 오류 | 0 / 0 | 0 / 0 |
| 종료 후 Manual/P 근거 | 31개 / 0.668초 | 31개 / 0.674초 |
| 인계 구간 최대 절대속도 | 0.00110m/s | 0.00241m/s |
| 최고 기록 속도 | 약123.11km/h | 약118.66km/h |
| 자체 master/launch/recorder 종료 | 모두 exit0 | 모두 exit0 |

두 시험은 감독의 `route_end_stopped` 조건으로 종료했다. END를 지난 runout 끝에서
정지한 뒤 bridge가 Manual/P로 인계했다. **이번 시험에서 follower의 FINISH 메시지까지
관측했다고 표현하지 않는다.** 이전 사용자 수동 bag의 FINISH와 구분한다.

1차 전체 소요 약324.78초, 2차 약371.31초는 기동·정지·인계가 포함된 감독 시간이다.
공식 주행시간이 아니다. 네 신호 정지 위치는 약s202/310/431/601m였고 정지 시간은
1차 약15.50/49.30/73.25/9.75초, 2차 약57.80/48.35/73.60/9.70초였다.
원본 영상·YOLO·신호 정책·target이 연결되며 신호 변경 후 재출발했다.
이 대기를 회피 실패나 지속 NO_TARGET으로 합산하지 않는다.

신호 메시지의 기재 age와 bag 수신시각으로 계산한 age 차이는 최대 약0.110/0.095초였다.
수신·callback 시점 차이가 섞인 관측값이며, 모든 timestamp/지연 문제가 해결됐다는
증거로 해석하지 않는다. 보행자 완료 플래그는 미션 전환 시 초기화되므로 종료 시
false만 보고 미완료로 판단하지 않고 해당 미션의 최초 완료를 추출했다.

## 터널 이전 실패의 연속 입력 재현

이전 실패 `053941`과 성공 `054906` bag은 raw LiDAR·영상이 없는 기본 기록이다.
이번 원본 센서 기록을 갖춘 두 실제 시험에서는 s1828 정지가 재현되지 않았다.

기존 실패의 기록 pose/footprint를 시간순으로 재생하면 304 tick 중232 tick에서
속도 상한0이 나온다(원래 기록231 tick). 마지막 s1828.090에서 stop_s1838.542와
`planner hold`가 일치한다. 성공 기록은412 tick 모두 속도 상한0이 없었다.
실패 입력의 footprint age 최대 약0.146초, target 기록 age 최대 약0.00383초여서
이 자료에서 장기 target 단절이나 GPS→DR이 정지의 직접 원인이라는 근거는 없다.

최종 정지 입력에는 s1841.54~1843.78m, 횡방향 -1.71~+0.99m인 점유 box가 남는다.
기존 차체 여유를 적용한 후보 중 장애물과 비접촉인 offset은25개,
지도 포장도로 검사를 통과하는 offset은1개이며 **둘 다 만족하는 후보는0개**다.
따라서 이 입력에서는 곡선 선택 전에 통과 가능한 횡방향 후보가 없어 제동한다.
물체가 실제 장애물인지 지면 오탐인지는 당시 raw 데이터가 없어 확정할 수 없다.
장애물을 제거하거나 지도/여유/ROI/threshold를 바꾸지 않았다.

이 재생은 s1750 이후 기록 pose 고정, Cruise/Avoid/Tunnel 중심의 부분 context다.
실제 ROS callback 순서·미션 이전 기억·전체 YIELD를 완전히 재현하지 않으므로
232 대231 차이를 기능 동등성 통과로 표현하지 않는다.
[재현 요약](stability-20261022/tunnel-replay-summary.json)에 한계를 함께 보존했다.

## 변경과 검증

| 파일 | 이유·내용 | 검증 |
| --- | --- | --- |
| docker/audit_course.py | 기존 기록기 생존 확인에 fresh Manual/P·충돌 수신·활성 bag 생성 확인 추가. 출발 전10GiB/진행 중2GiB 기록 여유 검사. HOME 대신 ROS 전용 경로 사용 | 실제2회 출발·종료, 기존 감독7개 PASS |
| docker/prepare_course_audit.py | 생산 launch를 보존한 시험 사본 생성. 첫 랩 옵션과 YOLO/follower 영구 기록 경로 명시 | 동일 방식의 사본으로 drive=false 및2회 주행, 새 생성 도구의 XML/노드 해석 PASS |
| tools/analyze_course_audit.py | 이전 HOME 경로만 찾던 분석기가 새 follower 경로도 읽도록 수정 | 첫 분석의 빈 속도/정지 집계를 새 v2 분석에서 복구, 2차 분석도 확인 |
| README·현재 계획·이번 문서/요약 | 성공·실패·미검증 범위 갱신 | 링크·구문·diff 검사 |

새 준비 도구는 노드를 실행하지 않으며 생성 경로가 이미 있으면 거부한다.
감독의 기존 충돌/이탈/120초 무진행/입력 유실 기준은 완화하지 않았다.
이번 실제 우측 offset3.265m는 기존 이탈 기준3.5m보다 작고 지도 밖 여부도 함께
감시한다. 단순 횡오차만으로 이번 우회를 이탈로 판정하지 않는다.

- drive=false 3회 PASS: 센서·Status·위치·TF·target 및 기동~종료 UDP9093 송신0.
- 격리 사본의 정적·보행자·터널·신호·회전교차로·고속도로·clock/stale 경계 **203 PASS**.
  matplotlib 경고10개, 실패/skip0. 벽시계577.94초에는 2차 주행 중 회귀 일시정지가 포함된다.
- 감독7개 PASS. 실제 노드/센서 고장 주입은 이번에 하지 않았다.
- 기존 전체944/도구13/어댑터27/native build 성공은 이전 결과다. 이번에 새로 전부
  재실행한 것으로 표시하지 않는다. packaging/resource/CRLF 기준 실패도 그대로 남는다.
- Windows·공식 채점·다른 날씨/NPC 배치·차선 접촉은 미검증이다.

## 기록·재현·복구

[공유 요약](stability-20261022/summary.json)에 CP·미션·속도·인계·hash·실제 bag 토픽을 남겼다.
로컬 자료 루트는 `ros1-port/docker/artifacts/stability-20261022-170834/`다.
각 run에 raw 센서/카메라/debug/TF/인지/계획/제어 bag, 원본 UDP pcap/wire,
YOLO JSONL, follower CSV, 프레임, launch/ROS 로그, source manifest가 있다.
`run-01-analysis-v2`, `run-02-analysis`에는 영상과 원본 시각 CSV가 있다.
구버전 분석·실패한 초기 호스트 probe·패키지 경로 없는 첫 재생 로그도 덮어쓰지 않았다.
bag·영상·대용량 trace는 Git 대상에서 제외한다.

다음 시험 준비(컨테이너 안에서 명령 실행, 각 경로는 새 이름 사용):

```bash
bash ros1-port/docker/run.sh python3 docker/prepare_course_audit.py \
  --output docker/artifacts/stability-next/launch
```

같은 sample_scene 시작점·Manual/P·포트 연결 후 수신 검증:

```bash
bash ros1-port/docker/run.sh --receive-root --capture-control bash -lc '
  source /opt/ros/noetic/setup.bash
  source /workspace/ros1-port/devel/setup.bash
  export ROS_LOG_DIR=/workspace/ros1-port/docker/artifacts/stability-next/preflight-ros
  mkdir -p "$ROS_LOG_DIR"
  python3 docker/verify_live.py --seconds 15 \
    --launch-file /workspace/ros1-port/docker/artifacts/stability-next/launch/from-start.launch \
    --output /workspace/ros1-port/docker/artifacts/stability-next/preflight/live.json
'
```

위 결과·시작점·연결 확인 후에만 감독 주행:

```bash
bash ros1-port/docker/run.sh --receive-root --capture-control bash -lc '
  source /opt/ros/noetic/setup.bash
  source /workspace/ros1-port/devel/setup.bash
  python3 docker/audit_course.py \
    --launch-file /workspace/ros1-port/docker/artifacts/stability-next/launch/from-start.launch \
    --output-dir /workspace/ros1-port/docker/artifacts/stability-next/run-01
'
```

오프라인 재분석은 `tools/analyze_manual_course.py --bag <bag> --output <새폴더>`,
`tools/analyze_course_audit.py --run <종료된run폴더> --output <새폴더>`를 쓴다.
이전 터널 재생 스크립트·원본 입력은 로컬
`docker/artifacts/stability-tunnel-evidence-20261022/compare.py`와 bags에 있다.
해당 자료를 함께 복사하고 ROS workspace를 source한 환경에서 실행해야 한다.

복구는 스냅샷을 빈 디렉터리에 풀어 manifest SHA를 비교한 뒤 필요한 파일만 복원한다.
전체 reset/clean은 하지 않는다. 장면 배치/날씨 변경은 없고, 마지막 실제 시험은
END 이후 Manual/P 인계 상태로 종료됐다. 3차 리셋 요청은 사용자 결정으로 취소됐다.

## 후속 우선순위

1. 기존 터널 실패 물체의 실물/지면 여부: raw 입력을 가진 실패 재현이 필요하다.
2. 바퀴·차선 접촉과 공식 감점: 이번 충돌 슬롯0만으로 판정할 수 없다.
3. 다른 보행자 출현/가림·재진입, 차량 간격/끼어들기, 안개/조명 조건.
4. 실제 센서 재연결·pause/clock·기록기/노드 유실 고장 주입과 배포 해시 검사 정합성.

같은 조건 두 번의 성공을 위 조건 전체의 성공으로 확장하지 않는다.
