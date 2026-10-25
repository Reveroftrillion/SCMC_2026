# MORAI 연결 후 수신 전용 재검증 — 2026-10-21 KST

## 목표·범위·방법

사용자가 MORAI 연결을 완료한 뒤 기존 ROS 실행과 포트 충돌을 확인하고,
기존 `docker/verify_live.py`로 `drive:=false` 40초 수신·위치·target·TF 및
기동부터 종료까지 제어 UDP9093 0개를 재검증했다.
완료 기준은 실제 수신 결과와 제어 비활성화를 기록하고 자체 실행을 종료하는 것이다.
브랜치는 `test/morai-readiness-20261021`이다. 영향 파일은 이 문서와 새 Git 제외
artifact뿐이며 production 로직·비교 원본과 기존 검사 기록은 보존했다.

## 결과

**수신·판단 종합 검사 PASS(exit 0), drive=false, 제어 UDP9093 0개.**

사전 기존 ROS master/실행 컨테이너 및 필수 수신 포트 충돌은 없었다.
실제 Simulator 프로세스와 MORAI GPS/IMU/LiDAR/Camera/Status/제어 포트를 확인했다.

| 토픽 | 수신 수 | 관측 Hz |
| --- | ---: | ---: |
| GPS | 388 | 9.74 |
| IMU | 726 | 18.24 |
| Competition Status | 800 | 20.08 |
| LiDAR points | 287 | 7.21 |
| Camera-7 JPEG | 623 | 17.99 |
| odometry / s | 각각 1989 | 50.01 |
| behavior target | 780 | 20.00 |
| control request | 797 | 20.00 |

관측 Hz는 토픽 구독 후 수신 기준이며 센서의 설정 주기나 전체 프레임 처리율을 뜻하지 않는다.
GPS/IMU/status bridge의 파싱 오류는 0이다. TF `map←base_link`, `base_link←lidar` lookup 성공.
마지막 후륜 진행거리 s=14.7036m, 위치=(-124.5199,-415.4941)m, target=CRUISE,
차량 gear=1(P), 속도 약0이다. 카메라 JPEG 디코딩은640×480으로 확인했다.

control request의 HANDSHAKE 및 Auto/D 미응답 경고는 ROS 요청이다.
bridge의 drive=false로 실제 제어 송신은 기동~종료 0개였고 Auto/D 전환은 검증하지 않았다.
신호 토픽은0개로 이번 정지 위치에서 신호 인식·해제 경로는 검증하지 않았다.
검사 중 GPS 0.3초 무수신으로 GPS→DR→GPS 전환 경고가 관측됐으며 마지막 모드는 GPS다.
GPS 수신 간격의 일시적 변동은 추후 확인 대상으로 남긴다.

launch는 SIGINT 후 exit0으로 종료했다. 카메라 두 노드의 기존
ExternalShutdownException traceback은 재관측됐다. 관측 도구 errors 배열은 비어 있다.
사후 자체 컨테이너·ROS master 및 필수 수신 포트 점유가 없음을 확인했다.
MORAI는 유지했다. 주행·로직 수정·commit/push/merge는 수행하지 않았다.

## 기록·재현·남은 검증

성공 기록은 `docker/artifacts/readiness-recheck-20261021-5t0Clc/`의
`live.json`, `live.launch.log`, `exit-code.json`, 사전/사후 포트·컨테이너 목록에 있다.
첫 시도는 mktemp 폴더 그룹 쓰기 권한 부재로 launch 기동 전에 실패했으며,
그 로그는 `docker/artifacts/readiness-recheck-20261021-N1bGbX/`에 보존했다.
성공 시도에서는 새 폴더에 그룹 쓰기 권한을 부여했다.

저장소 루트에서 기존 ROS 실행이 없을 때 새 폴더로 재현한다.

```bash
rmt_check_dir=$(mktemp -d ros1-port/docker/artifacts/readiness-recheck-XXXXXXXX)
chmod g+rwx "$rmt_check_dir"
bash ros1-port/docker/run.sh --receive-root --capture-control \
  python3 docker/verify_live.py --seconds 40 \
  --output "${rmt_check_dir#ros1-port/}/live.json"
```

센서 장착 좌표·원본 입력 조건의 동등성, 신호 인식, 회피 정상성 및 전체 코스 주행은
이번 수신 점검으로 입증하지 않았다. 다음 개발은 보존 bag으로 실패 경계를 재현할 수 있다.
production 코드 변경이 없어 빌드·단위·차등 회귀 검사는 실행하지 않았다.
