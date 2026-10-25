# 첫 주행 이후 Competition Status 수신 중단 진단

2026-10-13 약 16:12–16:15 KST, 사용자의 요청에 따라 추가 주행 없이 수신 전용 검사와
`drive:=false` 실행만 수행했다. [최초 짧은 주행 및 정지 확인](MORAI_SHORT_DRIVE_2026-10-13.md)의
정상 수신 결과와 이번 수신 중단을 구분한다. 원본 알고리즘·파서·미션·튜닝·센서 보정값은 변경하지 않았다.

기계 판독용 [진단 요약](morai-status-followup-2026-10-13/summary.json),
[10초 UDP 검사](morai-status-followup-2026-10-13/probe-before-reconnect.json),
[20초 ROS 관측](morai-status-followup-2026-10-13/live-before-reconnect.json)을 보존했다.
파일명의 `before-reconnect`는 이 진단에서 재연결 완료 응답을 받기 전이라는 뜻이며,
사용자가 그 전에 어떤 UI 조작을 했는지는 확인되지 않았다.

## 소켓과 실행 설정

정상 수신 당시 사용자 확인값은 양쪽 IP `127.0.0.1`, Competition Host `9088`, Destination `909`였다.
현재 호스트 소켓 조회에서는 MORAI PID 15893의 GPS `1110`, IMU `1113`, LiDAR `2369`,
Camera-7 `9090`, 제어 수신 `9093`은 있으나 **`9088` 송신 소켓은 없다.**
현재 MORAI UI에 표시된 Competition IP·포트·Connect 상태는 아직 확인 응답이 없어 정상값과 같다고 단정하지 않는다.

검사 전과 종료 후 `909`는 비어 있고, 다른 Docker 컨테이너·ROS master/launch는 없었다.
수신 전용 probe는 `909` bind 오류 없이 실행됐다. ROS 실행 중에도 `127.0.0.1:909` 소켓 하나가 보였고
bridge 시작 로그는 `GPS :1111, IMU :1112, status :909; drive=False`였다.
컨테이너 소켓의 PID는 호스트 `ss`에 표시되지 않아 PID 기반 소유권은 독립적으로 확정하지 않았다.
기존 수신 프로세스의 포트 중복이나 bind 실패 증거는 없다.

Docker는 기존 `run.sh`의 Linux host network를 사용했다. 이미지 ID는
`sha256:bfe22b1606d4a07f0625f20f85f8f9f763be4203be232e33fbc6402f160d6724`이다.
실제 ROS 파라미터는 drive=false, sim_ip/bind_ip=127.0.0.1, status_port=909,
ctrl_port=9093, gps_port=1111, imu_port=1112였다.

## 실제 패킷과 ROS 결과

| 항목 | 수신 전용 10초 | drive=false ROS 관측 20초 |
| --- | ---: | ---: |
| Competition Status | 0 패킷 | `/vehicle/status` 0 메시지 |
| GPS | 100 패킷, 파싱 오류 0 | 198 메시지 |
| IMU | 424 패킷, 파싱 오류 0 | 538 메시지 |
| LiDAR | 6,510개의 정상 1,206바이트 패킷 | 173 cloud |
| Camera-7 | 266개의 완성된 640×480 영상 | 398 compressed image |

원본 bridge 자체 집계도 마지막 3초 보고 시 GPS=180, IMU=762, status=0,
세 채널 parse_errors=0이었다. Status 오류 0은 수신 자체가 0인 결과이며 파싱 성공을 뜻하지 않는다.
ROS 구독 수는 bridge UDP 수신 집계와 관측 시점·구독 처리율이 달라 동일하지 않다.

별도 Ego Status `9100→9111`에서는 229바이트 패킷 434개가 들어왔다.
원본 Competition 파서는 모두 `not Competition Vehicle Status (181 bytes)`로 거부했다.
예상된 다른 형식이므로 Competition 대용으로 사용하지 않았다. Camera-7에는 기존과 같이
비-MOR 메타데이터 266개도 들어왔고 probe가 구분했다. 카메라 수신 형식·ROI는 변경하지 않았다.

Status가 없어서 odometry, s, map→base_link TF와 behavior target은 발행되지 않았다.
제어 상태는 `WAITING | waiting for a pose to pick the route`였으며,
정적 base_link→lidar TF만 확인됐다. `passed_receive_and_decisions=false`, 검사 도구 종료 코드 1이다.
ROS launch는 SIGINT 후 종료 코드 0으로 끝났지만 기존 카메라 종료 traceback은 여전히 남았다.
**실행부터 종료까지 수동 캡처한 UDP destination 9093 제어 패킷은 0개**였다.

## 해석과 다음 확인

이번 증거는 Competition UDP가 수신 대상에 도달하지 않는다는 것을 보여준다.
포트 중복이나 수신 패킷의 파싱 거부로 설명되는 상태는 아니다.
`9088` 소켓 부재는 MORAI 해당 채널의 연결·bind·설정 문제와 일치하지만,
UI와 재연결 결과를 확인하지 않아 원인을 확정하거나 복구됐다고 보고하지 않는다.
다른 센서가 들어오는 사실만으로 MORAI Pause 상태를 추정하지 않는다.

사용자에게 Competition IP 양쪽 127.0.0.1, Host 9088, Destination 909 및 Connected 표시를
다시 확인하고 같은 값으로 Connect하도록 요청했다. 재연결 후에는 추가 주행 없이
9088 소켓, 909의 181바이트 패킷과 원본 파서 성공, drive=false ROS status 수신을 다시 확인해야 한다.
이번 상태에서는 실제 속도나 현재 정지를 새로 확인할 수 없다. 최초 주행 뒤의 5초 정지 확인은
그 당시 별도 증거로만 유지한다. 전체 미션·센서 좌표 동등성 검증으로 해석하지 않는다.

원시 launch 로그와 소켓 기록은 `ros1-port/docker/artifacts/status-followup-20261013/`에 있다.
최종 조회에서 컨테이너·ROS master/launch·909 수신 소켓은 남지 않았고 제어 송신은 시작하지 않았다.

## 16:23 KST 소켓 재확인

사용자의 후속 응답 뒤 16:23:24 KST에 호스트 소켓을 다시 조회했다.
[재확인 원시 기록](morai-status-followup-2026-10-13/sockets-recheck-162324.json)에서도 MORAI PID 15893의
9088 소켓은 없고 909 수신 포트는 비어 있었다. Docker 컨테이너·ROS master/launch도 없었다.
이 재확인은 소켓 조회만 수행했으며 앞선 10초 패킷 검사나 20초 drive=false 실행을 새로 수행한 결과로
보고하지 않는다. 재연결 완료 또는 현재 UI 설정값에 관한 확인 응답은 아직 없어 복구 여부는 미확인이다.

16:38–16:40 KST에는 사용자가 시작한 drive=true launch가 이미 실행 중인 것을 확인했다.
추가 launch 없이 관측했으며 여전히 Competition 수신은 0이었다. 이때의 실제 제어 송신과 수신 소켓 소유권,
직접 실행·종료 절차는 [터미널 직접 실행 기록](MORAI_MANUAL_LAUNCH_2026-10-13.md)에 별도로 정리했다.

## 16:47–16:48 KST 재연결 후 복구 확인

사용자가 Competition 재연결 완료를 알린 뒤, MORAI PID15893의 127.0.0.1:9088 소켓이 다시 나타났다.
검사 전 기존 컨테이너·ROS master/launch와 수신 포트 점유는 없었다.
수신 전용 8초 검사에서 9088→909의 181바이트 Competition 패킷 345개가 모두 원본 파서를 통과했다.
오류0, 마지막 실제 피드백 mode2/gear4(Auto/D), speed 약0.000000922m/s였다.

이어 drive=false로만 15초 관측해 ROS status405개, odometry742개, s742개, behavior target283개를
받았고 map→base_link 및 base_link→lidar TF lookup이 성공했다.
`passed_receive_and_decisions=true`이며 관측 도구 오류는 없었다. 실행부터 종료까지 제어 UDP9093은0개였다.
이 검사에서 drive=true launch나 drive_short_trial.py는 실행하지 않았다.
기존 camera SIGINT traceback과 센서 입력·좌표 차이는 이 통과 범위 밖이며 해결됐다고 보고하지 않는다.

16:48:12 KST 최종 조회에서 검사 컨테이너·ROS master/launch는 없고 수신 포트909/1111/1112/2368/9297은
해제됐으며 MORAI9088/9093 소켓은 유지됐다. 사용자가 직접 주행 launch를 실행할 수 있게 검사를 종료했다.
이 복구는 해당 시점의 관측 결과이며 앞선 수신 중단이나 최초 주행 기록을 변경하지 않는다.

보존 자료: [수신 전용 검사](morai-status-followup-2026-10-13/reconnected-1647/probe.json),
[drive=false ROS 검사](morai-status-followup-2026-10-13/reconnected-1647/live.json),
[종료 후 소켓](morai-status-followup-2026-10-13/reconnected-1647/after.json).
