# RMT — ROS1 / MORAI 자율주행

2026 대학생 AI/SW 모빌리티 경진대회용 개인 개발 저장소다.
`2026_CTRL_competition/` 원본을 비교 기준으로 보존하고 `ros1-port/`에서
ROS1 Noetic·Python 3.10·catkin·MORAI 환경으로 개발한다.

2026-10-24부터 실행 패키지는 `ctrl_*`, 자체 모듈·실행 파일·launch도 `ctrl_*` 이름이다.
명명 변경 후 ROS1 워크스페이스를 한 번 재빌드한다. 실행 진입점은 `roslaunch ctrl_start.launch`다.
[이전→현재 이름과 검증](ros1-port/docs/CTRL_NAMES_2026-10-24.md)을 참고한다.

## 실행

**이 PC에서는 새 Bash 터미널에서 `roslaunch ctrl_start.launch`만 실행하면 된다.**
어느 폴더에서든 사용할 수 있고 Docker 진입은 자동이다. 기존 Docker 이미지·빌드는 그대로 사용한다.

1. MORAI에서 같은 sample_scene을 로드하고 원래 시작점 반경3m 안으로 Reset한다.
2. Manual/P·Pause 해제 상태에서 Status·제어·GPS·IMU·LiDAR·Camera 채널을 다시 Connect한다.
   [현재 포트 설정](ros1-port/README.md#morai-준비)을 따른다.
3. 기존 실행을 종료한 상태에서 새 Bash 터미널에 입력한다.

```bash
roslaunch ctrl_start.launch
```

**기본값은 `drive:=true bag:=true raw:=true`이므로 기동 직후 출발할 수 있다.**
센서 연결만 먼저 점검하려면 다음을 실행하고, Ctrl+C로 종료한 뒤 위 주행 명령을 실행한다.

```bash
roslaunch ctrl_start.launch drive:=false
```

종료는 실행한 터미널에서 **Ctrl+C 한 번**. 실제 속도0과
`shutdown handoff confirmed: Manual/P, brake released`를 확인한다.
이 일반 주행 명령에는 충돌·장기 무진행 자동 종료 감독이 없다.

설정 등록 전에 열어 둔 터미널이거나 launch 파일을 찾지 못한다면 한 번 실행한다:

```bash
source ~/RMT-ros1/ros1-port/docker/host_setup.bash
```

주행 기록은 `ros1-port/docker/artifacts/ctrl_start_*` 아래 기본 bag·raw.bag·follower CSV에 남는다.
다른 PC의 한 번 설치와 기존 Docker/bash 실행 방법은
[ROS1 실행 안내](ros1-port/README.md#대회용-launch-한-번으로-실행),
[호스트 명령 설정](ros1-port/docs/HOST_ROSLAUNCH_2026-10-23.md)을 참고한다.

## 최신 개발 결과 — 2026-10-23

**터널 돌발 수정 후 왼쪽·중앙·오른쪽 단일 상자 장면에서 각1회, 총3회 감독 완주했다.**
각 주행에서 제공 CP 반경15/15 통과·유효 충돌 보고0·종료 후 정지·Manual/P 인계를 확인했다.
생산 소스349개는 각 주행 전후 변화0이었다.

| 수정 후 감독 시험 | 왼쪽 | 중앙 | 오른쪽 |
| --- | --- | --- | --- |
| 추가 물체 주변 관측 | 27개 | 22개 | 22개 |
| 원본 충돌 패킷 / 유효 충돌 보고 | 16,001 / 0 | 15,689 / 0 | 14,297 / 0 |
| 완주·정지·Manual/P | 확인 | 확인 | 확인 |

이번 묶음 전체5회 중4회 완료, **수정 전 왼쪽 반복1회는 실제 상자 충돌로 중단**했다.
초기 관측 뒤 공백에서 막 채택한 우회·복귀 곡선이 취소되는 원인을 재현해 수정했다.
낮고 밝은 상자의 지면 분리·첫 관측 전달, 도로 안의 복귀 후보 탐색 수정도 포함한다.
실패 기록은 보존하며 좌·중·우 각1회 성공을 모든 돌발 조건의 안정성 확보로 해석하지 않는다.

최종 전체 계산 회귀는 **979개 중978 PASS / 기존 ROUNDABOUT 기대값1 FAIL / skip0**,
도구28 PASS다. 합성 조합1,167개는1,163통과/원거리 연속 물체4실패다.
관측 공백108조건 중 최초 우회 가능81조건은 수정 후 모두 경로를 유지했고,
나머지27조건은 최초 정지여서 우회 성공으로 계산하지 않았다.
Windows·새 catkin/native 설치 빌드·공식 채점은 이번 단계에서 실행하지 않았다.

[터널 수정·전체 실험 기록](ros1-port/docs/TUNNEL_CONTINGENCY_2026-10-23.md) ·
[최종 검증 요약](ros1-port/docs/tunnel-contingency-20261023/batch-01/final-state-right-complete.json) ·
[현재 작업 계획](ros1-port/docs/DEVELOPMENT_WORKFLOW.md)

앞선 로터리·정적·터널 수정과 사용자 완주는
[로터리 기록](ros1-port/docs/ROUNDABOUT_WAIT_2026-10-23.md),
[정적 기록](ros1-port/docs/STATIC_STOP_2026-10-23.md),
[터널 재질 기록](ros1-port/docs/TUNNEL_MATERIAL_EDGE_2026-10-23.md),
[사용자 완주2회](ros1-port/docs/USER_FULLCOURSE_2026-10-23.md)에 보존했다.

## 이전 결과 — 2026-10-22

**동일 sample_scene의 감독 주행 2회 모두 START→제공 CP 전체→END를 통과했다.**
두 시험 모두 원본 CollisionData의 유효 객체 충돌0과 종료 후 정지·Manual/P를
확인했다. 공식 점수·차선 감점·다른 배치의 성공과는 구분한다.

| 항목 | 최신 감독 시험 결과 | 남은 확인 |
| --- | --- | --- |
| 전체 코스 | 같은 생산 소스로2회 CP1~10·12~15·END 순서 통과. CP11은 제공 파일에 없음 | 다른 NPC 배치·날씨·입력 조건 |
| 정적 장애물 | 두 번 모두 오른쪽 약3.265m 우회·CP4·복귀 | 실제 바퀴의 차선 접촉·공식 감점 |
| 보행자 | 두 번 모두 전용 YIELD와 관측된 통과 후 완료 | 다른 출현 시점·가림·재진입·동물 |
| 신호 | 실제 적색 대기 후 재출발, 영상→YOLO→판단 기록 | 조명·안개·가림 |
| 회전교차로·고속도로 | 두 번 모두 구간 통과, 대기·진입·차선 변경·복귀 기록 | 다른 차량 간격·끼어들기·급제동 |
| 터널 | 두 번 모두 GPS→DR→GPS와 구간 통과 | 이전 s1828m 정지의 물체 실물/지면 여부 |
| 기록·충돌 | raw 센서/영상/YOLO/인지/계획/제어/TF, 충돌13,959·14,873개 중 유효 충돌0 | 공식 충돌·차선 채점과의 대조 |
| 종료 | 두 번 모두 fresh Status31개에서 정지·Manual/P 확인 | 다른 고장 조건의 종료·인계 |

이번 두 시험은 END 이후 감독의 `route_end_stopped`로 종료했다. 이전 사용자
수동 시험에서 확인한 FINISH 메시지와 구분한다. target 최대 간격은 각각
0.14142/0.14303초이며, 부하·손실 모든 조건의 성공을 뜻하지 않는다.
사용자 요청으로 기준 반복을2회에서 마쳤다. 생산 주행 알고리즘은 변경하지 않았다.

이전 터널 실패는 연속 footprint 재생에서 stop_s1838.542·planner hold를 재현했다.
장애물과 지도 차체 여유를 동시에 만족하는 후보가 없지만, 당시 raw LiDAR가
없어 실제 장애물인지 지면 오탐인지는 미확정이다. 이번2회 통과로 해결 처리하지 않는다.

[최신 반복 시험·남은 과제](ros1-port/docs/STABILITY_AUDIT_2026-10-22.md) ·
[공유 가능한 시험 요약](ros1-port/docs/stability-20261022/summary.json) ·
[이전 사용자 완주와 미검증](ros1-port/docs/CURRENT_STATUS_2026-10-22.md) ·
[작업 계획](ros1-port/docs/DEVELOPMENT_WORKFLOW.md)

## 알고리즘

| 구성·미션 | 현재 방식 |
| --- | --- |
| 추종·속도 | Pure Pursuit, 가변 lookahead·횡오차 적분·조향 필터; PI 기반 속도 제어와 감속 feed-forward |
| 위치 | GPS/IMU/차속, GPS 음영 DR, 복귀 blending |
| 신호 | YOLO ONNX/CUDA → 교차로 ROI → 프레임 확인 → 신호 기억·정지선 제동 |
| LiDAR | 지면 평면 후보 추정·최소제곱 보정 → DBSCAN → 추적·규칙 분류 |
| 정적 | 제공 상자/경로 사전정보 기반 오른쪽 5차 곡선, 지도·실시간 BEV 검사 |
| 보행자 | LiDAR 형태·이력 분류, 횡단 예측, YIELD 및 관측된 이탈 확인 |
| 회전교차로 | 원형 차로 점유 구간·경계 Kalman 추정·예측 빈틈 진입 |
| 고속도로 | 차로 점유·시간 슬롯 탐색·차간거리 제어·5차 차선 변경/복귀 |
| 터널 | 재질/지면 지지 검사·BEV·좌우 통과 조합·5차 곡선·DR |


## 검증과 남은 작업

이전10/22 관련 오프라인 회귀 **203 PASS**, 감독7 PASS, drive=false3회 UDP0을 확인했다.
이전 오른쪽/보행자 production 소스의 전체 알고리즘 회귀 **944 PASS**, 도구13 PASS,
ROS1 어댑터27 PASS, native 메시지/TF·격리 catkin build/install·노드 import 통과.
리소스 해시/원본 식별자 기준/메시지 줄바꿈 관련 검사는 실패 상태를 보존했다.
[검사 요약](ros1-port/docs/verification-20261022/checks-summary.json)을 참고한다.
Windows 과거 결과는 현재 Linux 실행과 구분한다.

다음 우선순위는 진행 중 새로 보이는 두 번째 장애물의 재계획·제동, 터널 보행자 재진입,
차단 해제 후 재출발과 실제 관측 공백 재현이다. 이후 배치별 반복성·차선/공식 감점,
입력 지연·손실·재연결·과부하와 배포 해시 검사 정합성을 확인한다.

## 문서와 보존

과거 개발본·계획서·참고 자료는 [과거자료/](과거자료/README.md)에 모았다.
현재 개발은 `ros1-port/`, 비교 원본은 `2026_CTRL_competition/`이다.


- [개발 규칙](AGENTS.md) · [문서 분류](ros1-port/docs/MARKDOWN_INDEX.md)
- [오른쪽 정적·보행자 수정](ros1-port/docs/RIGHT_STATIC_PEDESTRIAN_2026-10-22.md)
- [앞선 감독 주행 기록](ros1-port/docs/FULL_COURSE_AUDIT_2026-10-22.md)
- [터널 정지·출력 축소](ros1-port/docs/CONSOLE_TUNNEL_RECHECK_2026-10-22.md)
- [미사용 정리 근거](ros1-port/docs/UNUSED_FILE_CLEANUP_2026-10-22.md) · [파일별 미션](ros1-port/docs/FILE_MISSIONS_2026-10-22.md)
- [과거 루트 안내 원문](ros1-port/docs/history/ROOT_README_before_2026-10-22.txt)

원본·각 파일의 작성자/출처/저작권/라이선스를 보존한다. ROS1 이식 기준은
`a1ea82efbf8d5f165201b7f51bf8a141439fb8c7`이며 이후 허용된 기능 수정은
원본 동등성 검사와 구분해 기록한다. 원본의 Apache-2.0 및 어댑터의
Apache-2.0/BSD-3-Clause 등 패키지별 선언은 각 package.xml/소스가 기준이다.

bag·영상·build/devel/install·Docker 이미지·대용량 artifacts는 Git에 올리지 않는다.
날짜별 실험 기록과 검증 JSON/SHA256은 과거 근거로 보존한다.
GitHub push와 main 병합은 작업마다 사용자의 명시적 지시에 따른다.
