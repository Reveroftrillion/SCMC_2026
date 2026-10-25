# 현재 실행과 미션별 파일 안내 — 2026-10-22

[정리 결과·삭제 근거·복구](UNUSED_FILE_CLEANUP_2026-10-22.md)와
[남은 파일 전체 CSV](cleanup-2026-10-22/retained-files.csv)를 함께 사용한다.
CSV는 Git 추적 파일과 ignored가 아닌 미추적 파일 전체를 한 행씩 분류한다.
ignored build/devel/install·환경·bag·artifacts는 소스 목록과 별도다.
과거 실험 문서의 내용을 현재 구현 설명으로 고쳐 쓰지 않았다.

## 실제 실행 흐름

```text
docker/run.sh → image entrypoint.sh → Noetic + devel/setup.bash
  roslaunch docker/local.launch drive:=false
    → foscar_bringup/launch/competition.launch
      ├ sensor_bridge: GPS1111 / IMU1112 / Competition Status909
      │  ├ localization → 위치·TF·/localization/s
      │  └ path_follower의 /control/request 처리 (송신 허용은 drive에서 결정)
      ├ path_follower: global/spawn route → /control/request
      ├ behavior_node: 위치·인지·미션 → /behavior/target 단일 발행
      ├ obstacle.launch → lidar_bridge2368 → obstacle_node → 인지
      ├ delay_wall 5초 → competition_delayed.launch
      │  → yolo_signal.launch → camera_bridge9297 → yolo_signal_node
      ├ race_hud_node
      └ 선택: record_bag / points / BEV / RViz / camera_view
```

지연 실행은 wall 시간 보조 스크립트의 별도 roslaunch다. 따라서 부모 launch의
`--nodes` 출력에 자식의 카메라 노드가 모두 보이지 않아도 미사용이 아니다.
`behavior_node`는 YAML/launch의 `cruise,avoid,yield,tunnel`을 `importlib`로
읽으며 `yield`는 `yield_.py`로 fallback한다. 이 상태 파일을 보존했다.

공통 경로·지도·파라미터는 `foscar_bringup/{routes,maps,config}`,
미션 소유권은 `foscar_planning/config/missions.yaml`, 상태 설정은
`behavior.yaml`이다. 설치는 각 패키지의 `CMakeLists.txt`, `setup.py`,
`package.xml`, `scripts/`를 함께 사용한다. `find_packages`로 포함되는
Python 모듈과 config/maps/routes/models/rviz 설치 목록을 확인했다.

## 미션별로 먼저 볼 파일

아래 경로는 `ros1-port/src/` 기준이다. 여러 상태에서 import하는 파일은
CSV의 `missions` 열에 공유 관계를 표시한다. 이 열은 정적 의존성과 파일명에
따른 조사 분류이며, 해당 미션의 실주행 통과를 의미하지 않는다. 공통 인지는
이벤트·상태 필드로 동작하므로 import 그래프만으로 모든 메시지 관계를 나타낼
수 없어 아래 설명을 함께 사용한다.

| 분류 | 주요 파일/폴더 | 역할과 공유 관계 |
| --- | --- | --- |
| 공통 | `foscar_ros1/foscar_ros1/` | ROS1 메시지·publisher/subscriber·timer·executor·TF·point cloud 경계; 모든 미션 공유 |
| 공통 | `foscar_interfaces/msg/` | VehicleStatus / ControlCommand / DrivingTarget / Event 계약; 모든 미션 공유 |
| 공통 | `foscar_control/foscar_control/{bridge,localization,follower,core,reference,reference_control,speed_policy,mission_config,telemetry}.py` | UDP·위치·경로 추종·제어·미션 속도·기록; 모든 미션 공유 |
| 공통 | `foscar_bringup/{launch,config,maps,routes,rviz,scripts}/` | 실행·지도·global/spawn 경로·CP·기록·시각화; 모든 미션 공유 |
| 신호 | `foscar_camera/foscar_camera/{camera_bridge,yolo_onnx,yolo_signal_node,stop_lines}.py`, `config/`, `models/best.onnx` | JPEG→검출/색 선택→신호; 독립 신호 및 정적·보행자 구간의 신호가 공유 |
| 신호 | `foscar_planning/foscar_planning/signal_stop.py` | 신호 기억·정지선 정책; 다른 회피/보행자 제약과 합성 |
| 정적 | `foscar_planning/foscar_planning/states/avoid.py`, `static_course.py`, `avoid_planner.py`, `avoid_choice.py`, `mapped_avoidance.py` | 고정 코스 우회와 기존 일반 회피 fallback. 최근 수정본 전부 보존 |
| 정적 | `foscar_planning/foscar_planning/{frenet,bev_occupancy,bev_avoidance}.py` | 차체 sweep·점유·회피 제약; 터널 및 다른 국소 계획도 공유 |
| 보행자 | `foscar_planning/foscar_planning/states/yield_.py` | 보행자 양보; 보행자 미션·터널 및 회전교차로 제약과 연동 |
| 회전교차로 | `foscar_planning/foscar_planning/{roundabout_*,circular_gap,entry_gate,empty_entry,ring_*,boundary_motion,observed_boundary,following_*,local_following,rear_response}.py` | 진입·합류·간격·선행/후행 응답. `merge_planner:false` 같은 선택 경로도 보존 |
| 고속도로 | `foscar_planning/foscar_planning/highway_*.py`, `config/highway.yaml`, `highway_kappa.npz` | 관측·추종·추월·차로 변경·복귀; CP·지도·공통 제어 공유 |
| 터널 | `foscar_planning/foscar_planning/states/tunnel.py`, `tunnel_{simple,corridor,slalom}.py` | 터널 벽·코리더·슬라럼과 회피/양보 합성; 기본값 밖 fallback 포함 |
| 공통 인지 | `foscar_lidar/foscar_lidar/{lidar_bridge,obstacle_node,ground,road_mask,observed_footprint,fragment_merge,free_space,clearance,sight,highway_visibility,tunnel_walls}.py` | 정적·보행자·회전교차로·고속도로·터널 공유. 미션별 지면/가시성 처리와 관측 생성 |
| 개발·검증 | `*/test/`, `ros1-port/{tools,verification,docker}/` | 단위·회귀·직렬화·패키징·Docker·오프라인 재생 및 시험 기록 도구 |
| 개발·검증 | `foscar_camera_train/` 및 camera GPS/logger/view, lidar fake/monitor/view, BEV/HUD | 수집·학습·측정·시각화 및 선택 기능. 기본 주행에서 꺼져 있어도 제공 기능이라 보존 |
| 개발·검증 | `2026foscar-competition/` | 변경 금지 비교 원본·출처·기록·차등 검증 도구 |
| 개발·검증 | 다른 과거 작업 사본·`sim_2025/`·`tunnel_tools/`·`측정테스트파일/` | 현재 실행과 분리된 역사/재현/외부 참고. 삭제 보류 이유는 정리 기록 참조 |

`*.launch.py`는 ROS1 XML launch와 비교하는 패키징 검사 및 원본 보존에
필요하다. ROS2 이름이라는 이유로 삭제하지 않았다. Python 파일별 정확한
분류·공유 태그는 전체 CSV에서 확인한다.

## 실행 명령

저장소 루트에서 빌드와 수신 전용 실행은 다음과 같다. 아래 명령은 안내이며,
이번 정리에서는 launch로 노드를 시작하지 않았다.

```bash
cd /home/foscar/RMT-ros1
bash ros1-port/docker/run.sh bash docker/build_workspace.sh
bash ros1-port/docker/run.sh --receive-root \
  roslaunch docker/local.launch drive:=false bag:=true
```

빌드는 기존 devel/install을 갱신할 수 있다. 이번 검증은 원래 build/devel/install을
보존하기 위해 별도 `.work/unused-cleanup-20261022-120150/native/`에서 수행했다.
실제 제어는 이 문서 정리 범위에 포함하지 않는다. 수신 전용 실행을 나중에
시작할 때에도 기존 실행/909·1111·1112·2368·9297 포트 점유를 먼저 확인한다.
MORAI의 제어 수신9093, Status9088→909, 실제 경로·ROI·미션 설정은 그대로다.
