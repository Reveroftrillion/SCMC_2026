# competition_minimal

ROS Noetic와 MORAI용 프로토타입입니다. 실제 대회 주행과 완주는 검증되지 않았습니다.

수신 전용 시작:

```bash
bash competition_minimal/run.sh morai_ip:=MORAI_IP drive:=false
```

실행 스크립트는 Python·ROS 메시지, 모델, 경로·YAML, 로컬 UDP 포트를 검사합니다.
ROS Noetic, 저장소의 `simul_msgs`·`morai_msgs`, VLP-16용
`velodyne_pointcloud`, numpy, scipy, pyproj, PyYAML, OpenCV, PyTorch와
Ultralytics가 필요합니다. 외부에서 `/velodyne_points`를 발행한다면
`start_lidar_driver:=false`를 추가하세요. 상태 수신 기본 포트는 909입니다.
사용 권한이 없거나 실제 포트가 다르면 `status_port:=실제포트`로 지정하세요.

수신 상태 요약은 다른 터미널에서 ROS와 워크스페이스를 source한 후 실행합니다.

```bash
python3 competition_minimal/scripts/live_status.py --seconds 10
python3 competition_minimal/scripts/diagnose_gates.py
python3 competition_minimal/scripts/geometry_audit.py
```

제공된 181바이트 Competition Status 구조를 구현했으나 실제 패킷 수신은
이 컴퓨터에서 검증하지 못했습니다. 중복·역행 타임스탬프는 거부하며
시뮬레이터 시간이 초기화되면 노드를 재시작해야 합니다. `drive:=false`는
제어 UDP를 보내지 않습니다.

`missions.yaml`의 미검증 구간 앞에서는 감속·정지합니다. 신호 4·회전교차로
좌표 오차와 보행자 시작 경계, ROI, LiDAR TF, GPS 기준점, IMU 보정 방향을
현장에서 확인해야 합니다. 실측 도로 경계가 없어 자동 회피는 기본적으로
비활성화되어 있습니다. 패킷·센서·미션·차로 검증 후에만 설정을 갱신하고
`drive:=true status_layout_confirmed:=true`를 지정하세요. Auto/D를 수신하기
전에는 제어를 보내지 않습니다. 종료 시 P/Manual 인계는 합성 테스트만 했습니다.

합성 검증:

```bash
python3 -m unittest discover -s competition_minimal/tests -v
python3 competition_minimal/tests/continuous_sim.py
```

연속 모의는 검증되지 않은 센서·도로·미션 값을 시험용으로 가정합니다.
실제 ROS/MORAI 통합 또는 주행 검증을 대체하지 않습니다.
