# 2026 MORAI AI융합자율주행 2차 중간점검 준비·실행 검증

2026-10-16 KST, Ubuntu 현장 확인. 아래 본문은 사전 점검 기록이며, 사용자 승인 후 실제 주행 시작은 마지막 추가 기록을 참조한다.
제출 가능 상태는 **센서 JSON 준비 완료 / 제어 없는 ROS 실행 통과 / 주행 영상 미완료**다.

## 목적·변경 범위

기존 ROS1 코드를 유지하면서 공식 시나리오·센서·네트워크와 실행 환경을 확인하고,
주행 승인 후 영상과 센서 JSON을 제출할 수 있게 준비한다. 알고리즘·경로·미션·ROI·
watchdog·핸드셰이크는 수정하지 않았다. production 변경은 없고 이 문서만 추가한다.
직전 업데이트 기록 `UPDATE_2026-10-16.md`는 기존 미추적 파일로 보존했다.
commit/push/merge, 사용자 프로세스 종료, drive=true는 수행하지 않았다.

## 기준 자료 및 현재 상태

- 브랜치: `run/ros1-update-20261016`, HEAD `2d34dc2`.
- 실행 파일: `docker/run.sh`, `docker/local.launch`, `run_competition.sh`,
  `src/foscar_bringup/launch/competition.launch` 모두 존재한다.
- 공식 PDF: `/home/foscar/다운로드/2026 AI융합자율주행부문_중간점검_2차_가이드.pdf`.
  `pdftotext -layout`으로 직접 확인했다. 센서/네트워크 p.5–7, 실행 p.9–12, 제출 p.14.
- 시나리오: `/home/foscar/바탕화면/SCMC2026/2026_molit_comp_sample_scene.json`.
  사용자 로드 완료 응답과 MORAI Player.log의 동일 파일 Load Complete를 확인했다.
- 제공 경로: 저장소 `Readme, 각종정보파일/전역경로맵데이터/2026_molit_comp_global_path.txt`.
- 제공 고정 카메라 preset: `Readme, 각종정보파일/카메라 센서셋 데이터/2026_molit_comp_cam_set.json`.
- 과거 실행 안내와 10월13일 수동 주행·짧은 주행·Status 복구 기록을 읽었다.
  과거 성공과 이번 실행 결과를 구분한다.

## 환경·기동 확인

호스트 Ubuntu20.04.6, RTX3070Ti 8GB, NVIDIA560.35.05/CUDA12.6을 확인했다.
Docker 이미지 `rmt-noetic-py310:local`의 ID는
`sha256:bfe22b1606d4a07f0625f20f85f8f9f763be4203be232e33fbc6402f160d6724`다.
컨테이너 Python3.10.12, NumPy1.24.4, ONNXRuntime1.23.2 및 native TF2를 확인했다.
직전 같은 HEAD의 devel/install 빌드, 환경, 메시지 직렬화, adapter27개,
packaging8개, devel/install launch 해석은 모두 통과했다.
근거: `docker/artifacts/update-20261016/workspace/`.

MORAI PID10434의 실행 경로는 `Simulator_v.S4.251001.MolitComp03_Linux/Simulator.x86_64`다.
저장 네트워크 파일 이름은 `NetworkInfo_2023_Hyundai_Ioniq5.json`, 시나리오 맵은
`R_KR_PR_K-city_2025`다. 현재 GUI의 버전·맵·차량 선택값 자체를 화면으로 확인한 것은 아니다.
시험 시작 전 기존 ROS master/launch·컨테이너가 없었으며, 사용자의 MORAI는 유지했다.
검증용 컨테이너만 시작하고 종료했다. host network, IP127.0.0.1을 사용한다.

## 경로·시나리오 분석

실제 launch는 루트의 `global_path.txt`가 아니라 패키지의 `routes/global_path.txt`를 읽는다.
아래 모든 파일은 숫자3열 ENU [e,n,u]이며, 가이드 설명과 달리 실제 로컬 파일에는 Link ID가 없다.
기존 `numpy.loadtxt`와 `Route`가 모두 읽었으므로 파서 변경은 필요 없다.

| 파일 | 점 수 | XY 누적 길이(m) |
| --- | ---: | ---: |
| 루트 global_path.txt | 4430 | 2184.2453 |
| 루트 global_path_smoothed.txt | 4430 | 2184.2287 |
| 제공 2026_molit_comp_global_path.txt | 4430 | 2184.6117 |
| 실제 routes/global_path.txt | 4430 | 2174.5518 |
| 실제 routes/spawn_path.txt | 5214 | 2592.0632 |

시나리오 UTM52N/ENU 원점은 east302595/north4124145이며 localization 설정과 일치한다.
초기 위치 (-131.48599,-427.96100)은 실제 전역경로 s0.4224m, 횡거리0.0011m로 투영된다.
기존 auto 선택은 전역경로를 고른다. 주행 중 발행 경로는 finish runout60m가 추가돼
4525점/약2235m가 되며 파일 원본 길이와 다르다.

루트/제공 경로와 실행 경로의 같은 index XY 차이는 최대4.5463m다. 원본을 덮어쓰지 않았다.
현재 정지선 위치를 제공 경로에 다시 투영하면 기존 s202.485/309.8735/430.9174/600.8039가
204.0863/311.4748/432.5187/605.3250으로 달라진다. 경로만 교체하면 미션·정지선 s가 어긋난다.
실제 경로 SHA256 `a7fa4a89ff3840c7cc1be9b3b971572a67eb514f3ebee5a4a00ef535b73c64c3`는
`yolo_stoplines.json` 보정 해시와 일치한다.
샘플 장애물 s327.5213m/횡거리0.7549m는 STATIC_OBSTACLE270.4–388.4m,
보행자 s458.0729m/횡거리3.5406m는 PEDESTRIAN403.4–493.4m 구간에 있다.
이는 정적 위치 대조이며 동적 이동·회피·전체 미션 완주를 검증한 것은 아니다.
기존 경로를 유지해 실행할 준비는 됐지만 전체 새 시나리오의 주행 적합성은 미검증이다.

## 센서·통신 확인

기존 bridge → localization → perception → behavior → path_follower → bridge UDP 구조다.
path_follower가 `/control/request`를 계산해도 `drive=false`이면 bridge는 제어 UDP를 보내지 않는다.
Perception은 Camera-7 JPEG→YOLO/ROI 및 VLP16→장애물·footprint를 사용한다.
ROS 노드의 활성 소켓 코드는 센서3채널, LiDAR, Camera와 제어 송신뿐이다.
Object/Scenario/Traffic Light 네트워크를 인식 입력으로 사용하는 경로는 발견하지 못했다.
시나리오 JSON은 이번 정적 분석에만 사용했고 ROS 인지 입력으로 주입하지 않았다.

| 채널 | MORAI Host→RMT Destination | ROS 토픽 | 사용 |
| --- | --- | --- | --- |
| Competition Status | 9088→909 | /vehicle/status | 필수 |
| GPS | 1110→1111 | /sensors/gps | 필수 |
| IMU | 1113→1112 | /sensors/imu | 필수 |
| VLP16 | 2369→2368 | /sensors/lidar/points | 장애물 인지 |
| Camera-7 | 9090→9297 | /sensors/camera/traffic/compressed | 신호 인지 |
| 고정 Camera1/2/3 | 9290→9291 / 9292→9293 / 9294→9295 | 현 launch에서 구독 안 함 | 유지 |
| Ego Ctrl Cmd | MORAI 수신9093 | bridge 송신 | 승인 후만 |

모든 IP127.0.0.1. UI의 Ego Destination9082를 제어 송신 대상으로 쓰지 않는다.
초기에는 MORAI9088/9093 소켓이 없었으나 사용자 연결 후 둘 다 확인했다.
8초 재검사에서 Competition181바이트347개, GPS80개, IMU343개, VLP16 5325개가 원본 검사에
통과했다. 각 파싱 오류0. LiDAR UDP packet rate는 scan Hz와 다르다.

저장 센서 설정은 GPS1/IMU1/VLP16계열 LiDAR1/Camera4이며 다른 센서 목록은 비어 있다.
설정 주기는 GPS0.2초, IMU0.02초, LiDAR0.1초, Camera1/3 0.05초, Camera2/7 약0.03333초다.
GPS는 GGA/RMC가 따로 송신돼 UDP packet Hz가 설정 fix Hz의 두 배일 수 있다.
고정 Camera1/2/3 위치·회전·FOV는 제공 preset과 모두 일치했다.
Camera1은1280×720/FOV90, Camera2/3은640×480/FOV130을 유지한다.

장착값(저장 파일): GPS(0.519,0,1.214), IMU(0.750,0.039,1.227),
LiDAR(1.190,0.010,1.250), Camera7(1.900,0,1.200), Camera7 pitch340/FOV90/640×480.
GPS/IMU/LiDAR 회전은0이다. LiDAR model byte0x22와 표준1206바이트 블록을 받았다.
저장 `lidar3DType=0`, `intensityType=0`의 GUI 표시명은 직접 확인하지 않아
**VLP16 / Intensity를 F5에서 최종 확인해야 한다.**
원본 LiDAR TF(2,0,2.11)와 장착값은 다르며 차량 원점↔후륜축 변환이 미확인이다.
Camera7 모델 보정 기준1280×720/pitch335와 현재640×480/pitch340의 차이는 유지했다.
고정 카메라를 바꾸거나 카메라·좌표 보정값을 근거 없이 바꾸지 않았다.

초기 Camera7의 저장 `vizBoundingBox2D/3D=true`와 실제 BOX 패킷을 확인했다.
사용자가 GUI에서 해제하고 CTRL_sensor.json을 저장한 뒤 두 값false를 확인했다.
추가6초 수신 검사에서 Camera7 영상180장/패킷180개, 오류0으로 BOX 패킷이 사라진 것을 확인했다.
저장 JSON의 GroundTruthList는 비어 있으며 cameraType/modelType은0이다.
Ground Truth=None 표시의 GUI 확인은 사용자 수행 사항이다.
MORAI 저장 네트워크에는 여러 추가 채널의 설정 항목이 남아 있으나 설정 존재가 연결 증거는 아니다.
최종 GUI에서 Ego Ctrl Cmd/Competition Status/Collision Data 및 허용 센서 외 채널을 Disconnect한다.
Collision Data는 RMT에서 필수로 소비하지 않으므로 새로 추가하지 않았다.

## 제어 없는 ROS 실행 및 GPU 결과

`drive=false bag=true` 25초 실제 실행 후 정상 SIGINT로 종료했다.
기존 `verify_live.py`를 Git 제외 artifact에 복사해 bag 인자·로그 경로·수신 간격/header 나이
관측만 추가했다. production 소스나 제어 동작은 변경하지 않았다.

| 관측 토픽 | 메시지 수 | 수신 Hz |
| --- | ---: | ---: |
| /vehicle/status | 450 | 18.06 |
| /sensors/gps | 248 | 10.03 |
| /sensors/imu | 418 | 16.77 |
| /sensors/lidar/points | 204 | 8.21 |
| /sensors/camera/traffic/compressed | 327 | 16.59 |
| /localization/odometry | 1235 | 50.00 |
| /behavior/target | 481 | 20.00 |
| /control/request | 498 | 20.00 |
| /perception/footprints | 200 | 8.22 |

map→base_link, base_link→lidar TF lookup에 성공했고 ROS master와 필요한 노드가 기동했다.
제어 UDP9093은 기동부터 종료까지0개, 관측 도구errors=[], launch 종료코드0,
`passed_receive_and_decisions=true`다. 마지막 속도 약-0.0000023m/s, gear1(P),
s1.5658m, follower `HANDSHAKE`, 경로 오차 표시0.09m였다.
이는 제어 송신 없는 예상 대기 상태이며 실제 주행 성공이 아니다.
GPS/IMU/status bridge 파싱 오류는 모두0이다.
최대 수신 간격 GPS0.279/IMU0.085/status0.088/odom0.030초,
최대 ROS header 나이 GPS0.00060/IMU0.01153/status0.00035/odom0.00675초다.
header 나이는 bridge 발행 후 나이이며 센서 내부 시각의 정확성이나 전체 지연을 뜻하지 않는다.

YOLO CUDA 초기화 로그를 확인했다. 실제 Camera7 영상과 합성3장에서 원본/이식 raw/decode/선택색은 동일했다.
CUDA 실행 profile1644노드, 추가5회 추론 중앙값21.47ms, CPU fallback 없음.
현재 위치는 신호zone 밖이므로 traffic_signal0개, events0개다. 신호 인식·장애물 회피 성공 증거는 아니다.
종료 시 카메라2개 노드의 기존 ExternalShutdownException traceback이 재발했다.
bag은 약16.1MB,24.66초,8204메시지,indexed=true로 정상 읽혔다.
기존 bag 파일명 끝 개행은 남아 있다. Python/경로glob 또는 find -print0으로 다룬다.

## GUI 조작·공식 도구·녹화

`/home/foscar/다운로드/WeatherTimeChanger_Linux`를 확인하고 실행 권한을 부여해 GUI를 기동했다.
MORAI_Spawn_Linux는 실행하지 않았다. 공식 가이드p.12의 지정 절차는 다음과 같다.

1. WeatherTimeChanger에서 Connect, Time Interval=30, Weather Interval=30, Start Auto.
2. MORAI 왼쪽 위 톱니바퀴에서 Show Performance와 Show Vehicle Cluster and Map Info를 ON.
3. F5에서 Ground Truth=None, LiDAR VLP16/Intensity, 각 최대Hz와 BBOX OFF 확인.
4. Network에서 허가되지 않은 Object/Scenario/Traffic Light 등을 Disconnect.
5. 시나리오 로드 후P, 비상Pause 조작 확인. 공식 날씨 도구는 가이드의 명시 절차대로 사용.

실제 화면은 X11 `:1`,1920×1080이다. ffmpeg/ffprobe,libx264/h264_nvenc가 있고 OBS 명령은 없다.
녹화는 아직 시작하지 않았다. MORAI를 최대화하고 로컬 터미널에서 다음으로 전체 화면을 녹화한다.
기존 파일을 덮어쓰지 않고 별도 폴더에서 시험 녹화해 영상과 Performance 표시를 확인한다.

```bash
cd /home/foscar/RMT-ros1
ffmpeg -n -f x11grab -framerate 30 -video_size 1920x1080 -i :1.0 \
  -c:v libx264 -preset ultrafast -crf 20 -pix_fmt yuv420p \
  ros1-port/docker/artifacts/midcheck-20261016/submission/CTRL.mkv
```

녹화 종료는 ffmpeg 터미널에서 q. 주행 후 다음으로 재압축 없이 MP4로 변환하고 재생 확인한다.

```bash
ffmpeg -n -i ros1-port/docker/artifacts/midcheck-20261016/submission/CTRL.mkv \
  -c copy -movflags +faststart ros1-port/docker/artifacts/midcheck-20261016/submission/CTRL.mp4
ffprobe -v error -show_entries format=duration,size:stream=codec_name,width,height,avg_frame_rate \
  ros1-port/docker/artifacts/midcheck-20261016/submission/CTRL.mp4
ffmpeg -v error -i ros1-port/docker/artifacts/midcheck-20261016/submission/CTRL.mp4 -f null -
xdg-open ros1-port/docker/artifacts/midcheck-20261016/submission/CTRL.mp4
```

디코딩 검사와 함께 처음·중간·끝을 재생해 실제 주행과30초 간격 시간/날씨 변화,
차량 클러스터·Performance 표시를 눈으로 확인한다. 녹화 명령의 실제 동작은 아직 미검증이다.

## 실제 실행 명령과 승인 경계

기존 ROS가 없는지 확인하고 사용자 명시 승인 후에만 `drive:=true`를 실행한다.
다음은 로그와bag을 호스트에 보존하는 기존launch 실행 형식이며 아직 실행하지 않았다.

```bash
cd /home/foscar/RMT-ros1
bash ros1-port/docker/run.sh --receive-root bash -c '
  rmt_trial=/workspace/ros1-port/docker/artifacts/midcheck-20261016/drive-$(date +%Y%m%d-%H%M%S)
  export HOME="$rmt_trial/home"
  export ROS_HOME="$HOME/.ros"
  export ROS_LOG_DIR="$ROS_HOME/log"
  mkdir -p "$ROS_LOG_DIR" "$rmt_trial/bags"
  exec roslaunch docker/local.launch drive:=true bag:=true bag_dir:="$rmt_trial/bags"
'
```

제어 없는 재검증에서는 위 true를 false로 바꾼다. 기본 bag은 raw 영상 전체를 포함하지 않는다.
신호 문제 재현 시 별도 터미널에서 raw/debug 영상과 signal/behavior를 추가 rosbag 기록한다.
정상 종료는 launch 터미널 Ctrl+C1회 후 정지·Manual/P 인계와 실제 속도0/P를 확인한다.
비상 시 MORAI Pause를 사용한다. Pause 자체를 정지·인계 성공 증거로 사용하지 않는다.

## 제출·남은 작업·증거

제출 폴더: `ros1-port/docker/artifacts/midcheck-20261016/submission/`.
CTRL_sensor.json은 GUI 저장 파일을 내용 변경 없이 복사했다.
SHA256 `b59ac8166efdb936ae7059c1d6bdd1c5f0dfcb59667755a2ea6d201576fa9774`.
CTRL.mp4는 아직 없다. 실제 주행 승인·녹화·정상 정지·영상 재생과30초 시간/날씨 변경 확인이 남았다.
센서 좌표·Camera7 보정 차이, 교차로2 녹색 미채택, 전체 미션은 미해결/미검증이다.
이번에는 전체 차등 검사를 재실행하거나 불필요한 코드 수정을 하지 않았다.

제출 마감2026-10-16 23:59 KST. 이메일`yhpark@morai.ai`, 제목에`국민대학교 / CTRL`을 포함하고,
실제 주행 후 CTRL.mp4와 CTRL_sensor.json을 첨부한다. 설문 https://forms.gle/9Vn2WVNLSMJP34vr9 .
이메일 발송·설문 제출은 미실시이며 이번 작업에서 발송하지 않는다.

오늘 원시 기록은 Git 제외 `docker/artifacts/midcheck-20261016/`에 있다.
`probe.json`(초기Status0),`probe-reconnected.json`,`probe-bbox-off.json`,
`live.json`(freshness 포함),`live.launch.log`,`gpu.json`,`route-audit.json`,
`home/.ros/log/`와`bags/`를 보존했다. 기존JSON/manifest/날짜별 기록은 덮어쓰지 않았다.


## 15:27 KST 사용자 승인 후 실제 주행 시작

사용자가 “ㅇㅇㅇㅇ 가자 시작해줘”로 녹화 및drive=true 실행을 명시 승인했다.
기존ROS/컨테이너가 없고 MORAI9088/9093 소켓이 있는 것을 다시 확인한 후 실행했다.
MORAI창을 전면1920×1080으로 배치하고 ffmpeg 녹화를 먼저 시작했다.
기존launch를drive=true/bag=true로 실행했으며 자동단기시험이나 제어로직 수정은 하지 않았다.
실제 화면에서25.S4.MolitComp03、2023_Hyundai_Ioniq5、R_KR_PR_K-city_2025 표시를 확인했다.

- 주행 폴더: `docker/artifacts/midcheck-20261016/drive-20261016-062727/`.
- 녹화 중 파일: `docker/artifacts/midcheck-20261016/submission/CTRL.mkv`.
- 추가raw/debug영상·신호 진단bag: `docker/artifacts/midcheck-20261016/drive-diagnostics.bag`.
- 시작 위치s약1.57m에서s202.25m로 이동했고 첫 교차로에서 정지했다.
- 관측된경로오차표시0.06m, 센서파싱오류0, Camera7약25Hz.
- 첫 신호판정은intersection_01/red/STOP, rear_s202.2524/stop_rear_s202.485로 확인했다.

이 추가기록 시점에는 주행launch와녹화가 진행중이다. 정상정지·녹화종료·CTRL.mp4변환·
최종영상검사·완주는 아직 완료하지 않았다. 시간/날씨30초자동변경 및Show Performance는
직접 확인하지 못했으므로 제출전 확인해야 한다. 이후 사용자 요청 또는 주행종료에 따라
정상SIGINT인계 후 녹화를 닫고MP4로 변환해야 한다.


## 사용자 종료 요청 후 녹화 마감

사용자가 반복 출발/정지를 알리고 여기까지 제출하겠다고 요청해 주행launch에SIGINT를1회 보냈다.
`shutdown handoff confirmed: Manual/P, brake released`를 확인했고 launch는exit0으로 종료됐다.
추가bag recorder도SIGINT/exit0, ffmpeg는q/exit0으로 정상 종료했다. MORAI와날씨 도구는 종료하지 않았다.

원본 녹화는518.533초(8분38.5초),1920×1080,H.264,약202.6MB이며 전체 디코딩 오류0이다.
원본MKV와`docker/artifacts/midcheck-20261016/CTRL-full-recording.mp4`를 보존했다.
끝부분의IDE/채팅화면을 제출본에서 제외하려고 처음465초(7분45초)를 재압축 없이 복사했다.
제출본은`docker/artifacts/midcheck-20261016/submission/CTRL.mp4`,185,964,945바이트다.
실패나정지구간을 중간에서 삭제하거나 성공장면을 합성하지 않았다. 이 제출본은 부분주행이며
전체미션완주나종료인계장면을 담았다고 주장하지 않는다.

반복 정지의 직접 근거: follower CSV9737행 중NO_TARGET6333행이다.
첫교차로정지1562행,둘째교차로정지791행 외에NO_TARGET이 장시간·반복 발생했다.
`follower.py`는DrivingTarget수신이설정된유효시간을넘으면v_max0의NO_TARGET으로 제동한다.
센서bridge 파싱오류0이므로 현재 증거로는목표수신/유효성중단에따른안전정지가유력하다.
Behavior의연산지연·executor·전달중단중어디가원인인지는추가분석해야하며수정하지않았다.
마지막CSV s394.2593m까지진행했지만감속후최종위치와같다고보지않는다.
요약은`docker/artifacts/midcheck-20261016/drive-summary.json`에저장했다.

제출파일은같은submission폴더의CTRL.mp4와CTRL_sensor.json이다.
영상에서주행및날씨변화를보았지만30초간격자동변경과Show Performance요건은정량확인하지못했다.
이메일/설문은사용자제출사항으로남는다.


## 16:14 KST 수정 후 재주행

사용자요청으로NO_TARGET계산지연과시나리오재로드시시작점/끝점모호성을수정했다.
변경·재현·테스트범위는[수정기록](NO_TARGET_FIX_2026-10-16.md)을참조한다.
새영상은`docker/artifacts/midcheck-20261016/retake-20261016-160848/CTRL.mkv`로녹화중이다.
새실행에는`start_at_beginning:=true`를명시했다. 기존제출본은덮어쓰지않았다.


## 사용자 요청에 따른 전체 로직 롤백

사용자가 수정본의 실제 회피 실패를 보고하고 모든 로직 변경을 되돌리도록 요청했다.
주행·녹화를 종료하고 Manual/P 인계 완료를 확인했다. 수정한 production5개 파일을
HEAD2d34dc2 원본 바이트로 복원하고 추가 테스트2개를작업트리에서제거했다.
수정 diff와테스트사본은Git제외artifact의rolled-back-changes에보존했다.
따라서 앞선 성능·테스트 통과 기록은 채택된 수정 완료 결과가 아니다.
시작점옵션도제거했으며재녹화는원래launch와알고리즘으로진행한다.
기존영상·bag·검증기록은보존했고commit/push/merge는수행하지않았다.


## 16:21 KST 원본 코드로 재녹화 시작

사용자가시나리오·P·UDP재연결완료를알리고재녹화를승인했다.
원본코드drive=false검증에서센서·위치·목표정상,제어UDP0을확인했고현재s16.19m였다.
완전히복원한HEAD2d34dc2코드로drive=true/bag=true를실행했다.시작점옵션은없다.
새폴더`docker/artifacts/midcheck-20261016/rollback-retake-20261016-161848/`의
CTRL.mkv·diagnostics.bag와기본bag·로그를기록중이다. 실제s200m까지진행을확인했다.
기존원본영상과폐기한수정본영상은모두보존했다.현재녹화종료·MP4마감은아직하지않았다.
