# MORAI 개발 재개 수신 전용 점검 — 2026-10-21

날짜는 사용자 세션의 KST 기준이다. ROS 로그의 시스템 시각과 별도로 구분한다.

## 목표·범위

사용자 요청에 따라 기존 실행·포트 점유를 확인하고 `drive:=false`에서 센서 수신,
위치·target·TF와 제어 UDP9093 송신 0개를 검증했다. 주행과 로직 수정은 하지 않았다.
브랜치는 `test/morai-readiness-20261021`이며 기존 미추적 문서 두 개를 보존했다.
영향 파일은 이 요약과 Git 제외 `docker/artifacts/readiness-20261021/`의 새 기록이다.

방법은 기존 `docker/verify_live.py`로 40초 동안 실제 토픽을 관측하고,
launch 기동 전부터 종료까지 loopback의 UDP destination 9093을 수동 캡처하는 것이다.
완료 기준은 수신·TF·제어 비활성화 결과를 사실대로 기록하고 자체 실행을 종료하는 것이다.

## 결과

| 항목 | 결과 |
| --- | --- |
| 사전 기존 ROS master / Docker 컨테이너 | 없음, 수신 포트 충돌 없음 |
| GPU / Docker 이미지 | RTX 3070 Ti, 드라이버 560.35.05 / `rmt-noetic-py310:local` 존재 |
| MORAI 상태 | 프로세스명 확인에서 `MoraiLauncher_L`만 관측. 필수 센서 송신 포트·9088·9093 없음 |
| 수신 포트 | 검사 중 909 / 1111 / 1112 / 2368 / 9297 바인딩 확인 |
| GPS / IMU / Competition Status / LiDAR / Camera | 모든 토픽 수신 0개 |
| odometry / s / behavior target | 모두 0개 |
| TF | `base_link←lidar` 성공, `map←base_link` 실패(map frame 없음) |
| control request | 797개, 마지막 상태 WAITING. ROS 요청 생성과 UDP 송신은 별개 |
| drive 파라미터 / 제어 UDP9093 | false / 기동~종료 캡처 0개 |
| 수신·판단 종합 검사 | FAIL, 검증 도구 exit 1 |
| launch 종료 | SIGINT 후 exit 0. 카메라 두 노드의 기존 ExternalShutdownException traceback 관측 |
| 사후 | 자체 컨테이너·ROS master 및 필수 수신 포트 점유 없음. MORAI 런처 유지 |

`live.json`의 관측 도구 errors 배열은 비어 있으나, 센서 부재와 TF 실패로 종합 검사는 실패했다.
파싱 오류 0은 수신 성공을 뜻하지 않는다. GPU 추론, 센서 장착 좌표의 동등성,
차량 위치·기어·속도, 신호 판단 및 실제 주행은 이번 검사로 확인하지 못했다.
센서가 들어오지 않으므로 현 단계에서 planning 로직을 원인으로 판단하지 않는다.

## 다음 조치와 재현

사용자는 MORAI 런처에서 실제 시뮬레이터를 시작하고, 지도·차량이 보이는 장면까지
로드한 뒤 기존 센서 preset과 Competition Status 연결을 확인해야 한다.
시작점 부근 Manual/P/속도0에서 Pause를 해제하고 수신 전용 검사를 다시 수행한다.
같은 PC 구성의 IP는 127.0.0.1이며 포트는 [Docker 안내](../docker/README.md)를 따른다.

저장소 루트에서, 기존 ROS 실행이 없을 때만 실행한다. 매번 새 출력 폴더를 사용한다.

```bash
rmt_readiness_dir=$(mktemp -d ros1-port/docker/artifacts/readiness-XXXXXXXX)
bash ros1-port/docker/run.sh --receive-root --capture-control \
  python3 docker/verify_live.py --seconds 40 \
  --output "${rmt_readiness_dir#ros1-port/}/live.json"
```

이번 근거는 `docker/artifacts/readiness-20261021/`의 `PLAN.md`, `live.json`,
`live.launch.log`, `exit-code.json`, `*-during.txt`, `*-after.txt`에 보존했다.
production 및 비교 기준 원본의 추적 파일 diff 없음. 코드 변경이 없어 단위·차등·빌드
회귀 검사는 실행하지 않았으며, commit/push/merge도 수행하지 않았다.
