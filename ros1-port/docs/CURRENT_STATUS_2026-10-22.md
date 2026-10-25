# 현재 성공·미해결·미검증 — 2026-10-22

## 최신 실제 성공

사용자가 END까지 전체 완주를 보고했고, 닫힌 최신 수동 bag
`run_20261022_054906\n.bag`을 읽기 전용 분석해 확인했다.
오른쪽 정적·보행자 수정 후의 결과이며 이전 왼쪽 시험과 구분한다.
[공유 가능한 검증 요약](verification-20261022/manual-course-summary.json)에
원본 bag SHA256, CP별 최소거리/시각, 미션 상태, 한계를 기록했다.
원본 bag·대용량 trace·영상은 로컬 artifacts에 보존하며 Git에 올리지 않는다.

| 항목 | 이번 시험의 근거 | 판단 |
| --- | --- | --- |
| 경로 | START→CP1~10→CP12~15→END 순서 | 제공 CP 전체 통과. CP11은 제공 파일에 없음 |
| 정적 | lat_offset 요청 -3.5m, 실제 우측 최대3.268m, CP4 최소거리0.093m | 오른쪽 우회·복귀·CP4 실제 확인 |
| 보행자 | YIELD23개, pedestrian_yield_completed 최초 s450.90m | 전용 양보·관측된 이탈 후 완료 실제 확인 |
| 회전교차로 | WAIT77/ENTER115개 및 구간 통과 | 이번 교통 배치에서 진입·통과 |
| 고속도로 | WAIT_GAP36/CHANGE153/SETTLE47/FOLLOW184/REJOIN150개 | 차선 변경·추종·복귀 관측 |
| 터널 | GPS→DR(s1872.43)→GPS(s1974.59), 약7.439초 | 이번 시험 실제 구간 통과 |
| target | 최대 발행 간격0.15247초, 1초 초과0개 | 이번 입력/부하에서 지속 NO_TARGET 없음 |
| END | FINISH, follower runout s2234.55m | 종료 경로 도달·제동 유지 |

START→END의 첫 반경 진입 간격176.558초는 후륜축 로컬 위치 기준이다.
bag 길이323.326초에는 종료 후 대기 등이 포함되며 공식 주행시간이 아니다.
기록에는 END 이후 짧은 GPS/DR 왕복도 있어 종료 후 입력 유실 여부는 별도 점검 대상이다.

최신 마지막 Status는 속도약-0.000000515m/s, Auto/D(mode2/gear4)였다.
마지막 제어 상태의 FINISH·속도0 표시와 별도로 **종료 후 Manual/P 인계는
이 bag으로 입증되지 않는다.** 최신 수동 시험은 원본 CollisionData 감독도
없으므로 무충돌·공식 차선/신호 채점 성공으로 표현하지 않는다.

## 남아 있는 실패와 미검증

1. **터널 진입 정지:** 직전 다른 실제 시험은 s1828.090m에서 TUNNEL/AVOID,
   v_max0·stop_s1838.542로 정지했다. 마지막 footprint의 높이0.52~0.61m
   객체를 진짜 장애물/지면 오탐으로 확정하지 못했다. 기본 bag에는 원본
   LiDAR가 없어 다음 raw 센서 재현이 필요하다. 이후 성공으로 실패를 삭제하지 않는다.
2. **차선·충돌·인계:** 실제 바퀴와 실선/중앙선 접촉 시간, 공식 감점,
   동일 시험 CollisionData, 종료 후 fresh Status의 속도0·Manual/P가 필요하다.
3. **반복성:** 동일 장면의 다른 신호 시점·배치·날씨, 보행자 가림/동물,
   회전교차로 간격, 고속도로 끼어들기·급제동·미확인 보행자 이벤트5개.
4. **입력/동시성:** 오래된 입력 timestamp, 카메라 gate, executor 부하,
   손실·재연결·pause/clock jump는 이번 낮은 target gap만으로 해결 판정하지 않는다.
5. **배포/기록:** 모델 README hash, resource manifest와 실제 기능 변경,
   CRLF 비교, bag 이름 개행, YOLO JSONL 임시 경로, 종료 traceback이 남아 있다.
   원본/기대값을 바꿔 과거 검사를 통과시키지 않는다.

## 출발 실패는 별도 분류

현재 첫 랩 옵션은 START의 정확한 한 좌표가 아니라 **반경3m**를 허용한다.
폐루프 시작/끝 오투영을 방지하기 위한 명시적 초기화다. 새 첨부 로그의
(-127.45,-420.61)는 START에서 약8.8m여서 FAULT다. 반경 밖을 s0으로
강제 초기화하지 않는다. 원래 sample_scene 시작점으로 Reset한다.

다른 재기동의 Status RX0은 GPS/IMU만 수신돼 odometry/target이 없는 상태다.
Reset 후 Competition Status9088→909·Ego9093를 다시 Connect해야 한다.
이 두 기동 실패를 터널 회피 정지와 합치지 않는다.

## 현재 검사와 게시 범위

생산 Python 소스는 944 PASS를 수행한 격리 src와 모두 SHA256 동일함을
게시 전 재확인했다. 당시 도구13 PASS·어댑터27 PASS·native 직렬화/TF·
catkin build/install 성공. resource/packaging/원본 식별자 경계 실패는
[검사 요약](verification-20261022/checks-summary.json)에 그대로 보존했다.

이번 마감은 루트·ROS1 실행 README와 현재 계획/문서 분류 갱신,
이미 근거가 기록된 미사용4개 삭제의 반영이다. 기존 원본·과거 문서·검증
JSON은 보존한다. 현재 안내의 이전 원문은 history/*.txt에 보관했다.
실제 주행·제어 송신은 게시 작업 중 실행하지 않는다.

## 다음 명령

같은 sample_scene 시작점·Manual/P·네트워크 확인 후:

```bash
cd /home/foscar/RMT-ros1
bash ros1-port/docker/run_course.sh drive:=false bag:=true
# 위 실행 종료·수신 확인 후
bash ros1-port/docker/run_course.sh drive:=true bag:=true
```

직접 실행에는 자동 종료 감독이 없다. 실제 제동·Manual/P 인계를 확인한다.
새 manual-* 폴더에 console·ROS·follower·기본 bag·원본 센서를 보존한다.
환경 구축·포트·출력 축소·종료 조건은 [ROS1 안내](../README.md)를 따른다.

이전 감독 시험의 원본 충돌11,385개/유효 충돌0·Manual/P 인계 성공은
[이전 왼쪽 회피 시험](FULL_COURSE_AUDIT_2026-10-22.md)의 결과다.
이번 최신 오른쪽 완주에 그 증거를 합산하지 않는다.
