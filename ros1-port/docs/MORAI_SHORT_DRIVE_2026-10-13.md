# 첫 MORAI 짧은 주행 시험

2026-10-13 15:50:13 KST 시작. 사용자가 `drive:=true`를 명시 허용하고 Pause 해제·정차/P·
Ego Ctrl Cmd 연결을 확인한 뒤 **실제 주행은 한 번만** 실행했다. 원본 알고리즘·미션·튜닝값,
핸드셰이크와 watchdog은 변경하지 않았다. 원격 push는 하지 않았다.

처음의 [Docker 환경 검증](NOETIC_DOCKER_VALIDATION.md)과 별도 시험이다.
기계 판독용 [종합 결과](morai-short-drive-2026-10-13/summary.json),
[원래 관측 도구 결과](morai-short-drive-2026-10-13/original-harness-report.json),
[5초 정지 재확인](morai-short-drive-2026-10-13/stationary-confirmation.json)을 보존했다.

## 실행과 정지

기존 이미지 `cb0e0d2d…` 및 Linux host network를 사용했다. 시작 전에 Docker 컨테이너·ROS master/launch가
없고 수신 UDP 909/1111/1112/2368/9297이 비어 있는 것을 확인했다. 127.0.0.1:9093은 MORAI의
정상 제어 수신 소켓이었다. 원본 local.launch를 `drive:=true bag:=true`로 실행하고 관측 도구만 추가했다.

관측 도구는 센서/명령을 주입하지 않으며, 출발 후 3초·s 증가 12m·실측 8m/s 등의 시험 종료 기준에서
원본 launch에 SIGINT를 보낸다. 속도/오차 등의 종료 기준은 외부 시험 길이 제한이며 제어기 튜닝이 아니다.
이번에는 8m/s 기준으로 조기 종료했다. 원본 speed_kmh=60, 모든 미션과 기존 제한은 그대로다.

비상 정지는 MORAI Pause로 안내했고, 정상 종료는 Ctrl+C와 같은 SIGINT 및 원본 bridge의
제동→Park→Manual 인계로 수행했다. 종료 요청부터 첫 accel=0/brake=1 패킷까지 약 0.303초였다.
정지 요청과 실제 정지 사이에는 감속 시간·거리가 있으므로 12m를 최종 정지 거리 상한으로 해석하면 안 된다.

## 실제 데이터

| 확인 항목 | 관측 결과 |
| --- | --- |
| 제어 송신 | 127.0.0.1:9093으로 214개, 약 49.56Hz |
| 제어 패킷 | 원본 #MoraiCtrlCmd$, 55바이트, payload length=23, little endian 형식 통과 |
| 송신 측 포트 | 임시 포트 40667; 화면의 9082를 제어 수신 포트로 사용하지 않음 |
| 초기 차량 상태 | Auto/P: mode=2, gear=1 |
| Auto/D 요청/응답 | mode=2, gear=4, brake=1 요청 후 약 33ms에 실제 Auto/D 피드백 |
| 제어 상태 토픽 | WAITING 1개, CRUISE 56개; HANDSHAKE 샘플 없음 |
| 실제 최고 속도 | 9.2309m/s = 33.23km/h; 종료 요청 뒤 감속 전 최고값 포함 |
| 실행 중 ROS s | 0.8690 → 9.3785m, 증가 8.5094m |
| 경로 오차 | 제어 상태 로그의 최대 0.08m; 소수 둘째 자리로 출력된 값 |
| 종료 명령 | Auto/D 제동 후 Auto/P, 최종 Manual/P 인계 패킷 |
| 정지 재확인 | 수신 전용 5초, 서로 다른 timestamp 153개, Manual/P 유지 |
| 정지 속도 | 재확인 구간 최대 절대값 0.00000425m/s; 0.01m/s 미만 연속 정지 기준 통과 |

Auto 모드는 시작 시 이미 2였으며, P→D 기어 전환과 Auto/D 피드백을 확인한 시험이다.
UDP의 Auto/D 피드백이 follower 첫 유효 판단 전에 들어와 상태 토픽은 WAITING→CRUISE였다.
**HANDSHAKE 토픽 상태를 직접 관측했다고 보고하지 않는다.** 핸드셰이크를 삭제하거나 상태를 강제하지 않았다.

ROS s는 노드 종료 시 발행이 끝나 감속 중 마지막 위치까지 나타내지 않는다. 종료 후 마지막 실제 GPS를
원본 UTM 원점/Route.project 계산으로 오프라인 투영하면 global s≈16.5902m, 시작 대비 약 15.72m다.
이는 braking 구간을 포함한 GPS 기반 추정이며 종료 후 ROS가 발행한 s라고 주장하지 않는다.

## 센서 freshness

관측한 raw control/status/GPS/IMU 패킷의 원본 형식 파싱 오류는 0이었다. 이번 짧은 주행은 bridge의
3초 report 타이머 전에 종료돼 bridge 자체 RX 집계 로그는 없었다. 아래 값은 실제 ROS 구독 데이터다.

| 채널 | 최대 수신 간격 | 최대 ROS header 나이 |
| --- | ---: | ---: |
| GPS | 0.230초 | 0.00059초 |
| IMU | 0.058초 | 0.00053초 |
| Competition Status | 0.049초 | 0.00031초 |
| Odometry | 0.021초 | 0.00078초 |
| Control request | 0.091초 | 0.00059초 |

실행 중 HOLD/FAULT 상태는 관측되지 않았고 원본 센서 timeout 1.5초, command timeout 0.25초를
변경하지 않았다. header 나이는 발행 메시지의 나이이며, 센서의 모든 내부 지연이나 장착 좌표의 정확성을
검증한 값은 아니다. 현재 Camera-7/LiDAR/GPS/IMU 조건 차이는 앞선 문서 그대로 유지했다.

## 기록 및 관측 도구 문제

처음 관측 도구의 정지 판정은 false였다. 대기 중에는 0.6초 창, 최종 재평가에는 0.7초 창을 써서
감속 말기의 0.014m/s 샘플이 최종 판정에 다시 포함됐다. 원래 실패 보고서는 수정하지 않았다.
별도 수신 전용 5초 검사로 정지를 실제 확인했다. 이후 도구는 같은 창에서 확정한 증거를 유지하도록 수정했고
기록된 정지/주행 샘플로 확인했다. 원본 제어기나 정지 허용 오차를 바꾸어 통과시킨 것이 아니다.

첫 실행은 bag=true였지만 관측 도구가 상대 bag_dir을 전달했다. roslaunch 노드의 기본 cwd가 ROS_HOME여서
컨테이너 내부 임시 HOME 쪽으로 해석됐고 호스트에는 첫 주행 bag이 남지 않았다. 첫 주행 bag은 복구했다고
보고하지 않는다. 대신 실제 첫 주행의 UDP/ROS 원시 관측은 다음 파일에 남아 있다:

- `ros1-port/docker/artifacts/drive-first-20261013/events.jsonl`
- `ros1-port/docker/artifacts/drive-first-20261013/launch.log`
- `ros1-port/docker/artifacts/drive-first-20261013/report.json`

bag_dir을 절대 경로로 바꾸고 이후 **drive=false 정차 기록만** 8초 실행했다.
300,091바이트, 835메시지의 bag이 호스트에 남는 것을 실제 rosbag으로 열어 확인했다.
이 bag을 최초 주행 bag으로 사용하지 않는다. 후속 정차 기록 당시 GPS/IMU는 들어왔지만 Competition Status는
RX=0이었으므로 이 기록은 bag 저장 기능 확인에 한정한다. 최종 소켓 조회에서도 MORAI의 Host 9088 소켓이
없었다. 해당 채널을 다음 주행 전에 다시 확인해야 한다. 원인은 확인되지 않았으며 첫 주행의 정상 status 수신과
구분해 기록했다. bag 파일명에는 기존 XML command 출력의 개행이 포함되는 별도 기록상의 문제도 남아 있다.

진단 중 Docker entrypoint가 `bash -c` 같은 명령 인자를 catkin setup에 넘기는 문제도 발견했다.
setup source를 무인자 함수로 분리하고 entrypoint만 갱신한 이미지 `bfe22b16…`에서 실제 import를 확인했다.
Noetic/NumPy/ONNX Runtime 버전과 RMT production 코드는 변경하지 않았다. 이후 시험 도구는 ROS 로그와
HOME 아래 CSV도 호스트 artifacts에 보존하고 bag_dir에 절대 경로를 사용한다.

## 범위와 종료 상태

짧은 실제 일반 경로 추종과 정지 확인을 수행했다. 카메라의 5초 지연 전에 주행을 조기 종료했으므로
이 주행의 카메라/CUDA 동작은 새로 검증하지 않았다. 앞선 drive=false 수신/GPU 검증과 별개다.
모든 미션 완주, 신호·장애물 판단, 센서 extrinsic 동등성, ROS2/ROS1 전송 동등성을 통과했다고 보고하지 않는다.
기존 카메라 종료 traceback, 모델 README 해시와 원본 checkout 줄바꿈 문제도 해결했다고 보고하지 않는다.

이후 16:12–16:15 KST에 추가 주행 없이 수행한
[Competition Status 수신 중단 진단](MORAI_STATUS_FOLLOWUP_2026-10-13.md)은 별도 기록이다.
해당 후속 검사에서는 MORAI 9088 소켓과 909 패킷이 없었으며, 최초 주행 당시 정상 수신 결과를 대체하지 않는다.

최종 확인에서 RMT 컨테이너, ROS master/launch와 RMT 수신 소켓은 남아 있지 않았다. 제어 송신은 종료했다.
