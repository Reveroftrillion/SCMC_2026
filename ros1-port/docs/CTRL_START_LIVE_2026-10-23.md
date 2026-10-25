# 대회 launch 실환경 점검 — 2026-10-23

## 목표·입력·범위

사용자가 MORAI 준비 완료 후 시험 시작을 지시했다. 시작 전 수신 점검3~5분,
감독 주행·분석20~40분의 예상 시간을 알렸다. 현재 launch의 실제 수신·기록·종료를
확인하고 원본 센서를 남겨 터널 실패를 재현하는 것이 목표다.
브랜치는 `test/morai-ctrl-start-20261023`이며 기존 변경을 보존했다.
시험 중 production·설정·launch·원본·감독 기준은 수정하지 않았다.

로컬 자료는 `docker/artifacts/ctrl-start-validation-20261023-125934/`다.
시험 전929개 소스 파일을 SHA256·tar.gz로 보관하고 Git 상태·patch·HEAD를 기록했다.
현재 공유 요약은 [run-01-summary.json](ctrl-start-20261023/run-01-summary.json)이다.

## 실제 결과

- 새 `ctrl_start.launch` 수신 점검25초 PASS. GPS/IMU/Status/LiDAR/카메라,
  위치·target·TF2개 확인. 시작~종료 제어 UDP9093 송신0건, 종료코드0.
- 기본 bag·raw.bag·follower CSV 생성·종료 후 닫힌 파일 확인.
  기본 bag 이름의 기존 newline 문제는 그대로 남아 있다.
- 감독 주행 전 fresh Manual/P·정지27개 샘플/0.631초, 충돌 수신·기록기 활성 확인.
- 주행은 CP1~3 통과, s249.103m에서 raw_vehicle_status_lost로 중단했다.
  감독은1초 이상의 Status 원본 수신 공백을 감지했다. 완주·터널 통과 성공이 아니다.
- 종료 후 fresh Manual/P·정지31개 샘플/0.665초, 최대 절대속도0.00956m/s 확인.
  master/recorder/launch 종료코드는 모두0이다.
- 원본 충돌1308패킷에서 유효 객체 충돌0, observer callback 오류0.
- 이번 구간 최고속도59.488km/h, 최대 횡오차0.667m,
  target 최대간격0.08260초. 초기 NO_TARGET은 기록에 남기며 지속 단절과 구분한다.
- trial source-before/after 동일. 전체 incoming source929개도 변경0 확인.

## 중단 원인 경계

종료 시각 전후 pcap에서 Status909/충돌9092는 같은2.167초 공백을 보였다.
카메라9297은2.148초, IMU1112는2.147초, GPS1111은2.294초,
LiDAR2368은2.202초의 같은 시점 공백이다. 그동안 제어9093 최대 공백은 약0.040초다.
독립 sensor_bridge 로그에서도 해당 시점 GPS/IMU/Status age가1초를 넘고
파싱 오류는0이었다. 전체 입력 일시 중단 근거가 있으며 단일 ROS 구독 문제로 단정하지 않는다.

MORAI Pause/조작/일시 정지/부하 등 구체적인 원인은 아직 미확정이다.
사용자에게 당시 조작 여부와 다음 START Reset·재연결 상태를 질문했다.
감시 시간·watchdog·속도·ROI·threshold를 완화하지 않았다.
다음 주행은 사용자 START Reset·연결 상태 확인 후 진행한다.

터널 s1828m까지 도달하지 못했으므로 기존 터널 실패는 그대로 미해결이다.
이번 짧은 구간의 속도 정상값으로 과거118~123km/h 기록도 해결 처리하지 않는다.
공식 차선·감점·Windows/native build·변형 조건은 이번 시험 범위가 아니다.

## 재현·검증

수신 점검은 컨테이너에서 다음을 사용했다. ROS_HOME/ROS_LOG_DIR는
같은 artifacts의 preflight 아래로 별도 지정했다:

```bash
python3 docker/verify_live.py --seconds 25 \
  --launch-file /workspace/ros1-port/ctrl_start.launch \
  --output /workspace/ros1-port/docker/artifacts/ctrl-start-validation-20261023-125934/preflight/live.json
```

주행은 기존 audit_course.py에 보관된 run-01-entry.launch를 전달했다.
이 사본은 ctrl_start.launch를 포함하고 drive/bag를 그대로 전달하며
run_dir만 run-01/ros-logs로 지정한다. 감독 자체가 원본 센서까지 bag으로
기록하므로 내장 bag은false로 실행됐다. 기존 충돌·이탈·입력유실·120초 무진행
판정과 자동 종료를 유지했다.

종료 bag 분석은 analyze_course_audit.py로 run-01-analysis라는 새 폴더에 저장했다.
원본 JPEG·시각을 보존하고 편의 영상camera.mp4를 생성했다.
로컬 analyze_input_gap.py는 pcap을 읽어 포트별 공백을 run-01-pcap-gaps.json에 기록했다.
기존 JSON·날짜별 시험 결과는 덮어쓰지 않았다. commit/push/merge는 하지 않았다.

## 2차 재현과 수정 작업 목표

사용자는 첫 입력 중단 당시 조작 여부는 모른다고 답했고 START Reset·연결 완료를 확인했다.
preflight-02도 수신·TF·target·UDP송신0·정상종료 PASS 후 새 run-02를 실행했다.
2차는 s1827.520m에서 TUNNEL/AVOID로 정지해 기존120초 무진행 기준으로 종료됐다.
source 변경0, Manual/P 정지32샘플/0.665초, master/recorder/launch 모두 exit0이다.
원본 raw를 포함한 diagnostics_0.bag 약911MiB를 닫은 뒤 분석했다.

현재 frame 추출·검출 재생은 기록된3개 footprint와 중심거리 약1.6e-7m로 대응한다.
재질 기반 지면 평면은 잡히지만, 커브 바깥 차선과 도로 가장자리의 일부 점이
아스팔트 지지 hull 밖에 남아 절대 높이약0.5m로 검출된다. 두 검출은 intensity
차선 범위100%, 세 번째는 지면 평면 잔차약-0.03~+0.03m의 낮은 가장자리다.
같은 입력에 평면과 일치하는 차선 관측을 지지 hull에 추가하는 감도 실험에서는
해당 점들이 지원영역에 포함된다. intensity 단독으로 실물 여부를 판정하지 않는다.

연속185스캔/22초 replay도 실제 정지 경계 부근 planner hold를 재현했다.
재생은 ROS 대역·cloud당1회 계산·부분 상태/고정 궤적이므로 실제 executor나
수정 후 폐루프 주행의 동등성 증거는 아니다.

이제 fix/tunnel-material-support-20261023 브랜치에서 아래 최소 수정을 검증한다.

- 목표: 관측·평면과 일치하는 커브 바깥 차선의 높이 보정 누락을 해결.
- 영향: ground.py의 보정 지원영역 함수, obstacle_node.py의 터널 호출,
  지면 및 실제 입력 기반 회귀, 현재 계획·검증 기록.
- 방법: 평면 추정과 광범위 아스팔트 anchor 검증은 그대로 유지. 기존 평면 잔차
  기준에 맞는 추가 차선 관측만 hull 지지에 포함. 기존 호출은 동일 동작 유지.
- 검증: 낮은 차선/관측 사이 가장자리, 높은 차선 재질 물체·무지지/불일치/ROI 밖
  보존, 실패 원본 단일·연속 재생, 관련 인지·터널·기타 회귀, 실제 재시험.
- 완료 기준: 실패의 사전 재현과 사후 경계 개선, 실제 장애물 보존 근거, 영향 회귀,
  사용자가 Reset한 후 동일 장면 실제 결과. 단계별 미검증을 구분한다.

초기 진단에서 존재하지 않는 bag 파일명과 잘못된 replay 하위명령을 사용한
실패가 있었다. 입력·소스 변경 없이 실제 diagnostics_0.bag와 replay 명령으로
정정했고, 최초 빈 tunnel-frame 출력과 새 tunnel-frame-v2를 구분했다.

## 후속 결과

run-02의 터널 실패를 원본 입력으로 분석해 평면과 일치하는 차선 재질 지원영역을
최소 수정했다. 전체946·도구9 검사 후 첫 수정본 run-03은 별도 정적 구간에서
중단했고, 같은 수정본의 run-04는 터널과 제공 CP·END를 통과했다.
종료 후 Manual/P·정지, 원본 충돌16,210패킷에서 유효 객체 충돌0을 확인했다.
한 번의 수정본 성공과 앞선 세 실패는 서로 구분한다.
상세 원인·수정·중간 실패·검증 한계는
[터널 지원영역 수정·실증](TUNNEL_MATERIAL_EDGE_2026-10-23.md)에 기록했다.
