# 터널 커브 외측의 재질 지원영역 수정 후보 — 2026-10-23

## 목표·원인·범위

실제 터널 s1827.520m TUNNEL/AVOID 정지를 원본 LiDAR와 함께 재현했다.
같은 스캔으로 추정한 지면 평면은 유효하지만, 차선과 낮은 도로 가장자리의
일부 점이 아스팔트 지지 convex hull 밖에 남아 절대 높이약0.5m의 장애물로 검출됐다.
이 관측과 지도/차체 조건의 충돌로 좌우 거부·planner hold가 발생한다.

브랜치는 `fix/tunnel-material-support-20261023`이다. 입력·원본·모델·경로·기대값은 보존한다.
이번 후보는 ground.py, obstacle_node.py, test_ground.py만 수정한다.
새 실행 진입점·과거 자료 이동·원본 이름 변경은 앞선 작업이며 이번 기능 수정과 구분한다.

## 구현

- 기존 아스팔트 기반 평면 추정, 최소 점수·넓이·지배율·잔차 검증을 유지한다.
- material_height_region에 선택 인자 additional_mask를 추가했다. 기존 호출은 동일 동작이다.
- 기존 광범위 아스팔트 연속 지지까지 통과한 뒤, 기존 ROI·높이창·평면 잔차0.12m
  조건과 일치하는 추가 차선 재질 관측만 hull 근거에 포함한다.
- 이 추가 차선 mask는 fresh TUNNEL mission의 재질 지면 fallback에서만 전달한다.
  다른 미션의 호출은 None이다. intensity 단독으로 지면을 판정하지 않는다.
- ROI·장애물 높이/클러스터링·신호·속도·watchdog·감독 중단 기준은 바꾸지 않는다.
- 높은 차선 재질 물체, 지지 없는 원거리/측방, 기존 ROI 밖은 그대로 보존하는 검사2개를 추가했다.

## 검증 결과와 한계

원시 자료: `docker/artifacts/ctrl-start-validation-20261023-125934/`.
변경 전929개 소스 tar·SHA256, 수정 후 fixed-source-sha256.json·fix.patch를 보존했다.
기존 10/22 manifest·JSON·source bytes를 바꾸지 않았다.

- 실패 정지 프레임: 같은 plane[0.00781337,0.00296559,0.42329134], 검출3→0.
  원래3검출은 기록 footprint와 중심거리약1.6e-7m로 일치했다.
  두 점군은 차선 intensity 범위100%, 다른 낮은 가장자리는 평면 잔차약±0.03m다.
- 185개 scan/22초 연속 부분 replay: nonempty footprint148→24프레임,
  최종 footprint3→0. 다만 상태 합성은 변경 전후 모두 zero-cap1프레임 뒤
  CRUISE로 돌아와 실제120초 AVOID 지속을 재현하지 못한다. 이 부분 replay만으로
  정지 해소를 주장하지 않는다. 오래된 BEV reason 문자열도 실제 현재 target과 구분한다.
- 전체 알고리즘: 격리 src 복사본·Docker Python3.10·ROS 대역에서946 PASS,
  FAIL0/skip0, matplotlib 경고10, 342.02초.
- 추출/재생 무결성·launch 구성 도구:9 PASS, FAIL0/skip0, 외부 deprecation 경고1.
- 구문 PASS. 일반 diff --check는 기존 CRLF 형식을 유지한 추가 줄의 CR을
  trailing whitespace로 보고한다. `git -c core.whitespace=cr-at-eol diff --check` PASS.
- 기존 실제 성공 터널의 한 프레임도 비교했지만 단일 계산의 검출은 전후 모두0이다.
  추적 footprint가 있던 시점이라는 것만으로 이 프레임을 실물 장애물 보존 성공으로
  해석하지 않는다. 양의 높이 물체 보존은 합성 안전 검사·기존 인지 회귀로 확인했다.

수정 후 첫 주행run-03은 s315.649m 정적 구간에서 no_progress_120s 종료됐다.
종료 후 fresh Manual/P31샘플/0.659초, 소스 변경0·자식 모두 exit0.
터널에 도달하지 못해 수정 후 터널 실주행은 미검증이다.
해당 static raw 입력은 변경 전후 검출11개 배열·plane·재질 계산이 동일하다.
전체 JSON의 estimator_return_location만 HEAD 가상 소스 파일명과 실제 파일명으로 다르다.
정적 footprint 재생에서도 observed box conflicts with fixed course·mapped pavement PASS를
확인했다. 실제 현재 footprint 이외에 BEV가 기억하는 s335.7~336.4/d-4.12~-2.12,
s340.2~340.9/d-3.63~+0.12 영역도 남아 있었다. 실물/오탐·정리 가능 여부는 별도 조사 대상이다.
원인 확정 전 기억·clearance·고정 우회 경로를 바꾸지 않았다.

## 재현 명령

노드·UDP 없이 컨테이너에서 계산한다. 출력은 반드시 새 경로를 사용한다.

```bash
python3 tools/replay_lidar_ground.py extract --bag <종료된 diagnostics_0.bag> \
  --at 1792729260 --output <새-frame>
python3 tools/replay_lidar_ground.py inspect --cloud <새-frame>/cloud.npz \
  --context <새-frame>/context.json --output <새-result.json>
python3 tools/replay_lidar_sequence.py replay --input <추출-sequence> \
  --output <새-replay> --focus-x 33.5 --focus-y=-480 --behavior
```

동일 변경 전 비교는 incoming snapshot을 격리 사본으로 복원하거나,
이번 ground/obstacle의 HEAD가 incoming SHA와 같음을 확인한 뒤 --baseline-head를 사용한다.
원본에서 시험·캐시를 만들지 않는다. 과거 검사 실패 기준·해시는 수정하지 않는다.

완료 기준은 기존 실패 재현, 새 지지 근거와 높은 물체/미관측 영역 보존,
관련 회귀, 수정 후 동일 장면 실제 터널/후속 종료 검증이다.
마지막 실제 검증 전까지는 수정 후보이며 전 조건 해결이나 완주를 주장하지 않는다.

## 수정본 후속 실제 검증 run-04

사용자가 다시 START Reset·연결 준비를 확인했다. 후보 소스929개가 고정 manifest와
같음을 확인하고 preflight-04에서 센서·TF·target·제어UDP0·정상종료 PASS 후
기존 감독으로 run-04를 실행했다. 코드·설정·감독 기준의 추가 변경은 없었다.

- 제공 CP1~10·12~15와 END 구간 통과, route_end_stopped로 종료했다.
- s1827~1828m를 지나 GPS→DR→GPS와 터널 구간을 통과했다.
- 원본 CollisionData16,210패킷 중 유효 객체 충돌0, callback 오류0.
- 종료 후 fresh Status31샘플/0.674초의 Manual/P·정지 확인.
  해당 구간 최대 절대속도0.001128m/s. 자식 master/recorder/launch 모두 exit0.
- target 최대 간격0.126595초,1초 초과0. 실제 우측 최대 offset은 횡오차 기준 약3.248m.
- 이번 최고 Status/follower 속도84.328km/h. 감독 전체411.036초는 기동·신호 대기·
  종료 인계를 포함하며 공식 주행시간이 아니다.
- 주행 source-before/after 동일, 전체 fixed-source929개도 변경0.
- 신호 대기는 약85.10/48.35/72.90/9.60초로 관측됐고 이후 재출발했다.
- 남은 실행 컨테이너0. 카메라 종료 때 기존 ExternalShutdownException traceback2개는
  여전히 보이며 이를 깨끗한 카메라 종료까지 통과했다고 주장하지 않는다.

[공유 run-04 요약](ctrl-start-20261023/run-04-summary.json)을 참고한다.
이 결과는 수정본으로 해당 조건을1회 완주한 근거다. 앞선2차 터널 실패·3차 정적 실패를
삭제하지 않으며 공식 차선·점수·다른 배치/날씨·반복 안정성 전체를 입증하지 않는다.
추가 동일 조건 반복을 자동으로 수행하지 않고 기록 분석 후 종료했다.

## 속도 기록 확인

[Status·GPS 비교](ctrl-start-20261023/speed-units-check.json):
run-02 최고 Status116.353km/h 부근1.025초에서 GPS 변위 환산113.771km/h,
같은 구간 평균 Status114.367km/h였다. run-04 최고84.328km/h 부근0.989초에서
GPS84.216km/h, 같은 구간 평균 Status83.377km/h였다.
수신 시각·GPS 주기를 포함한 약1초 직선 변위 비교이며 공식 속도 채점은 아니다.
현재의 높은 속도 기록을 단순 표시·단위 오류로 해석할 근거는 없다.

highway.yaml의 max_speed_mps는36.1(약130km/h)이다. competition speed_kmh60은
고속도로의 별도 정책까지 전 구간60으로 고정하는 값이 아니다. 이번에 제한값·정책을
변경하지 않았다. 대회 요구와 목표 속도 정책 정합성은 후속 검토 대상으로 남긴다.

## 남은 항목·검사 기록

- run-01 전체 입력2.1~2.3초 중단 원인 미확정. 사용자는 조작 여부를 모른다고 답했다.
- run-03 s315.649 정적 거부 미해결. run-04 통과로 실패를 해결 처리하지 않는다.
- 공식 차선·감점, 다른 NPC·보행자·날씨, 손실·재연결·clock/부하 조건은 미검증.
- 기존 resource·원본 CRLF·식별자·모델 문서 기준 실패는 유지한다.
- Windows·catkin 재빌드·설치 공간 배포 검사는 이번 수정 후 수행하지 않았다.
  Python 소스가 바인딩된 현재 Docker의 실제 ROS1 실행·UDP·TF·센서·주행을 검증했다.
- 원본·과거 실험/기대값/모델/경로 해시 보존. commit/push/merge 없음.

실험 디렉터리는 소스 백업·pcap/wire·raw 포함 bag·영상·추출 입력·수정 전후 재생·
원인 진단·회귀 요약을 포함한다. Git에는 요약/도구/최소 소스 변경만 검토한다.
구문·문서 링크와 CRLF-aware diff 검사를 마감에 다시 확인한다.
