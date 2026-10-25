# 터미널 직접 실행 준비와 종료 절차

2026-10-13 16:38–16:40 KST 확인. 원본 알고리즘·미션·튜닝값과 launch·Docker 실행 스크립트는
이 확인에서 수정하지 않았다. `drive_short_trial.py`는 실행하거나 연결하지 않았다.

## 실제 확인 결과

검사 시작 때 사용자가 직접 실행하는 명령과 같은 `roslaunch docker/local.launch drive:=true bag:=true`가
이미 컨테이너 `7316488ad385` (`recursing_elion`)에서 실행 중이었다. 이를 종료하거나 재시작하지 않았다.
ROS master 11311과 센서 수신 909/1111/1112/2368/9297은 해당 실행이 사용하고 있었다.
`/proc/net/udp` inode와 프로세스 fd를 대조해 UDP909 소유자가 이 컨테이너의 sensor_bridge PID69임을 확인했다.
중첩 camera launch는 기존 competition launch의 정상 자식이다. 두 번째 독립 launch를 시작하면 충돌한다.

MORAI PID15893의 9088 송신 소켓은 없었다. 기존 bridge 로그도 status RX=0이고,
GPS/IMU는 수신되며 follower는 WAITING, localization/odometry와 behavior target은 없는 상태였다.
16:40:04 KST부터 약 10초간 포트 bind 없는 AF_PACKET 수동 캡처 결과는 다음과 같다.

| UDP destination | 패킷 수 |
| --- | ---: |
| 909 Competition Status | 0 |
| 9093 제어 | 500 |
| 1111 GPS | 100 |
| 1112 IMU | 424 |

마지막 제어 패킷은 55바이트, mode=2, gear=4, accel=0, brake=1, steering=0이었다.
제어 송신 측은 127.0.0.1:51871이며 대상은 127.0.0.1:9093이다.
이 값은 명령이며 실제 차량의 Auto/D 전환이나 속도를 확인한 값은 아니다. Status 수신 복구는 **미확인**이다.
앞선 drive=false 검사에서 제어 송신이 0개였던 결과와 이번 기존 drive=true 실행의 송신을 구분한다.

수동 관측 컨테이너는 검사 후 종료됐다. 16:40:28 KST 조회에서 사용자가 시작한 주행 launch는 계속 실행 중이었다.
원시 기록은 `ros1-port/docker/artifacts/manual-launch-readiness-20261013-163802/`의
before.json, port909-owner.json, passive-capture.json, after-passive.json에 보존했다.

## 현재 설정

`run.sh`는 `rmt-noetic-py310:local`, GPU, Linux host network, ROS_IP=127.0.0.1을 사용한다.
`--receive-root`는 909 bind 권한을 부여하고 원본 및 이식 src를 읽기 전용으로 mount한다.
`local.launch`의 drive 기본값은 false이며 사용자가 true를 전달할 때 제어를 송신한다.
sim_ip/bind_ip=127.0.0.1, status909, ctrl9093, GPS1111, IMU1112, LiDAR2368,
Camera-7 수신9297이다. 원본 제어 송신은 임시 source port를 사용하며 9082를 제어 수신 포트로 사용하지 않는다.
원본 speed_kmh=60.0과 미션 설정은 그대로다. 직접 실행에는 자동 3초/거리/속도 기준 시험 종료가 없다.

bag 기본 경로는 컨테이너 `/workspace/ros1-port/docker/artifacts/bags`이며 호스트 저장소에 bind돼 보존된다.
정상 SIGINT 종료를 기다려 bag 기록을 마쳐야 한다. 기존 bag 이름 개행 문제와 camera SIGINT traceback은
해결됐다고 보고하지 않는다. bag 기본 토픽 집합은 raw 카메라/GPS/IMU/LiDAR 전체 녹화를 뜻하지 않는다.

## 실행 순서

1. 지금 실행 중인 터미널에서 Ctrl+C를 한 번 눌러 기존 launch가 종료되고 프롬프트가 돌아올 때까지 기다린다.
   설정 확인 동안 MORAI는 Pause 상태로 둔다. 비상 상황이면 MORAI Pause를 즉시 사용한다.
2. MORAI Competition Vehicle Status의 양쪽 IP127.0.0.1, Host9088, Destination909로 Connect를 확인한다.
   Ego Ctrl Cmd의 MORAI 수신 Host9093 연결도 확인한다. 일반 Ego Status9111로 대체하지 않는다.
   Competition 181바이트 패킷과 원본 파서 수신이 복구됐는지 수신 전용 검사로 확인해야 한다.
   이 문서 작성 시점에는 그 복구 확인이 완료되지 않았다.
3. 기존 실행이 끝난 상태에서 저장소 루트에서 아래 명령을 한 번 실행한다.

```bash
cd /home/foscar/RMT-ros1
bash ros1-port/docker/run.sh --receive-root \
  roslaunch docker/local.launch drive:=true bag:=true
```

4. Pause를 유지하고 startup/bind 오류가 없는지 확인한다. camera는 원본의 5초 지연 뒤 시작한다.
   `YOLO on CUDAExecutionProvider` 등 초기화 로그를 확인한 뒤, 주행을 관찰하고 종료할 준비가 된 시점에
   Pause를 해제한다. 수신/상태 조건이 충족되면 자동 Auto/D 핸드셰이크 후 출발할 수 있다.
   Pause 동안 timestamp/freshness가 진행한다고 가정하지 않는다.
5. 일반 정지는 Pause가 해제된 상태에서 launch 터미널의 Ctrl+C를 한 번 누른다.
   원본 bridge가 제동→Auto/P→Manual/P 인계를 시도하므로 반복 Ctrl+C, 창 닫기, 강제 kill 대신 완료를 기다린다.
   `shutdown handoff confirmed: Manual/P, brake released`와 MORAI 속도0/P를 확인한 뒤 Pause한다.
   확인 로그 없이 `shutdown handoff not confirmed within 3s; brake held`가 나오면 인계 성공으로 보지 않는다.
   필요하면 즉시 MORAI Pause를 사용하고 모드·기어·수신을 다시 확인한다. Pause 중에는 감속 물리 및 피드백이
   진행되지 않을 수 있어 Pause 자체를 실제 정지·Manual/P 인계 증거로 사용하지 않는다.

현재 카메라 입력과 센서 좌표 차이는 그대로 남는다. 이 직접 실행을 전체 미션 동등성 검증으로 해석하지 않는다.

## 16:44 KST 미출발 재확인

사용자가 제공한 16:43:21 스크린샷은 status=0, WAITING, odometry 없음, 표시 속도0과 브레이크1.00을 보였다.
16:44:21 KST 실제 조회에서도 새 사용자 실행 컨테이너 `5f45a30b8ec1` 하나가 실행 중이며,
MORAI의 9088 소켓은 없고 bridge는 GPS660/IMU2781/status0, parse_errors 모두0으로 보고했다.
909 수신 소켓과 MORAI9093 제어 수신 소켓은 존재했다. 실제 차량 status 피드백은 확인되지 않았다.
설정 변경이나 추가 launch 없이 조회만 수행했다. 원시 기록은
`ros1-port/docker/artifacts/manual-not-moving-20261013-164421.json`이다.
현재 필요한 조치는 사용자 launch 종료 후 MORAI Competition Vehicle Status 채널의
127.0.0.1 Host9088→Destination909 설정 및 별도 Connect 확인이다.

## 16:48 KST 복구 확인

사용자 재연결 후 9088→909의 Competition181바이트 패킷345개/8초, 원본 파싱 오류0을 확인했다.
drive=false ROS 검사에서도 status405개, odometry742개, behavior target283개와 두 TF lookup을 확인했고,
제어 UDP 송신은0개였다. 검사 종료 후 기존 실행·수신 포트 점유는 없었다.
상세 증거는 [수신 중단 및 복구 기록](MORAI_STATUS_FOLLOWUP_2026-10-13.md)의 마지막 항목에 보존했다.
복구 확인 뒤에도 직접 주행 명령은 사용자가 실행하며, 자동 단기 시험 도구는 붙이지 않는다.
