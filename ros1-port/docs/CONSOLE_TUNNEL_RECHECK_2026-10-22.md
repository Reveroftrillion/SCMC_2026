# 터미널 출력 축소·사용자 터널 정지 기록 확인 — 2026-10-22

목표: 사용자가 붙인 재기동 로그와 앞선 터널 정지를 구분하고, 주행 판단을
바꾸지 않으면서 화면 반복 출력을 줄이고 다음 원본 기록을 보존한다.
브랜치 `fix/console-verbosity-20261022`. 기존 수정·미추적·삭제는 유지했다.
이번 작업에서 실제 주행·제어 송신·사용자 실행 종료는 하지 않았다.

## 확인한 사실

- 붙인 로그(run 054554): GPS/IMU 수신 정상, Status 수신0, odometry/target
  없음, WAITING. 터널 실패 당시 로그가 아닌 약7초 재기동이다.
- 실제 주행 bag `run_20261022_053941\n.bag`: 약150초, 마지막 s1828.090m,
  TUNNEL/AVOID, v_max0, stop_s1838.542, lat_offset0. 실제 차량 속도는 거의0.
  의도된 시험 종료 지점이 아니라 계획 계층의 제동 요청이다.
- 다른 재기동 bag(053918, 054748): START 반경 밖에서 start_at_beginning
  FAULT. 시작점용 옵션을 중간 위치에서 실행하지 못하도록 한 기존 보호다.
- 터널 정지 직전 footprint3개는 높이약0.52~0.61m OBSTACLE이었다.
  이 정보만으로 진짜 물체/지면 오탐과 회피 거부 경계를 확정하지 못했다.
  기존 기본 bag에는 원본 LiDAR/영상이 없어 이번 분석만으로 ROI·지면·
  회피 threshold를 수정하지 않았다. 원본 센서가 있는 재현이 다음 작업이다.

읽기 전용 bag 분석은 docker/artifacts/console-tunnel-20261022/의
bag-inspection.json, inspect_bag.py에 보존했고 붙인 원문도 별도로 복사했다.
실제 bag 파일명 끝의 newline을 변경하거나 기존 기록을 덮어쓰지 않았다.

## 변경

docker/run_course.sh가 새 출력 디렉터리에 첫 랩 launch 사본을 만들고
기존 docker/local.launch를 포함한다. 같은 시작점 옵션3개를 적용한다.
drive 기본값false, bag 기본값true이며 실제 주행은 drive:=true를 명시한다.
화면 요약을 숨기고 console_filter.py가 반복 상태/통계는5초, 같은 경고는
10초 간격으로만 표시한다. 상태 전환·ERROR/FATAL·인계 로그는 즉시 표시한다.
필터 전 원문 전체는 console.log에 저장하며 CSV·ROS 로그·bag도 해당 폴더에
보존한다. raw 기본값true일 때 별도 raw.bag에 GPS/IMU/LiDAR/raw·debug 카메라/
TF/rosout을 기록한다(bag:=false면 기록기 모두 제외, raw:=false면 원본 기록기 제외).
YOLO JSONL의 기존 임시 경로를 옮겼다고 주장하지 않는다.

새 래퍼는 기존 사용자 명령을 자동 변경하지 않는다. 사용하려면 저장소 루트에서:

```bash
bash ros1-port/docker/run_course.sh drive:=true bag:=true
```

먼저 기존 launch를 정상 종료·정지/Manual/P 인계하고, 같은 sample_scene
시작점으로 Reset 및 Status9088→909/Ego9093/Collision→9092 재연결한다.
기동 직후 출발 가능. 이 래퍼에도 충돌·장기 무진행 자동 종료 감독은 없다.
Ctrl+C는 기존 launch의 종료/인계를 수행하고 필터는 EOF까지 원문을 보존한다.

## 검증·남은 범위

- bash 구문, Python 구문, 반복 상태/색상 경고 축소·오류/상태 전환/인계 표시 PASS.
- 표본 입력의 console.log 원문 일치 PASS; 필터에 SIGINT 후 인계 문구를
  넣어 EOF까지 보존되는 것 PASS.
- 실제 Docker/Noetic에서 --nodes drive:=false로 XML 해석 PASS,
  manual_sensor_bag 포함. 노드/UDP를 기동한 시험이 아니다.
- 초기 mktemp0700으로 cap-drop root가 폴더를 읽지 못하는 실패가 있었고
  기존 그룹에 rwx를 부여해 해결했다. 초기 실패와 수정 후 로그를 모두 보존했다.
- 생산 경로·인지·계획·제어 코드는 수정하지 않아 전체 알고리즘 재검사는
  이번 작업에서 반복하지 않았다. 직전944 PASS는 이전 작업의 결과다.
- 새 래퍼의 실제 주행·장기 기록 부하·종료 인계는 미실행이다.
  터널 정지 원인의 해결을 주장하지 않는다. commit/push/merge 없음.
