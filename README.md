# SCMC 2026

2026 대학생 창작 모빌리티 경진대회 기본 주행 테스트용 ROS1 코드입니다.

현재 단계에서는 **2026 global path 기반 한 바퀴 경로추종**을 우선 확인합니다.  
정적 장애물 회피, 합류 구간, 신호등 미션 로직은 현재 테스트에서 비활성화되어 있습니다.

---

## 현재 테스트 상태

| 항목 | 상태 |
| --- | --- |
| MORAI ↔ ROS UDP 통신 | ✅ 확인 |
| GPS 수신 | ✅ 확인 |
| IMU 수신 | ✅ 확인 |
| Ego Vehicle Status 수신 | ✅ 확인 |
| Global Path 로딩 | ✅ 확인 |
| Control Path 생성 | ✅ 확인 |
| Controller `/control_cmd` 생성 | ✅ 확인 |
| MORAI 제어 명령 송신 | ✅ 연결 |
| 정적 장애물 회피 | ⏸ 비활성화 |
| 합류 구간 로직 | ⏸ 비활성화 |
| 신호등 미션 | ⏸ 비활성화 |

---

## 목차

- [네트워크 구성](#네트워크-구성)
- [처음 실행하는 PC](#처음-실행하는-pc)
- [기본 주행 실행](#기본-주행-실행)
- [실제 MORAI 제어 시작](#실제-morai-제어-시작)
- [전체 데이터 흐름](#전체-데이터-흐름)
- [문제 발생 시 확인](#문제-발생-시-확인)
- [현재 비활성화된 기능](#현재-비활성화된-기능)

---

# 네트워크 구성

## PC IP

| 장치 | IP |
| --- | --- |
| MORAI 실행 PC | `192.168.0.14` |
| ROS 알고리즘 PC | `192.168.0.8` |

> IP가 변경되면 MORAI Network Setting과 실행 명령의 IP를 모두 수정해야 합니다.

## UDP Port

| 항목 | 방향 | MORAI Host Port | ROS PC Destination Port |
| --- | --- | ---: | ---: |
| Ego Ctrl Cmd | User → Sim | `9093` | `9094` |
| Collision Data | Sim → User | `9091` | `9092` |
| Competition Vehicle Status | Sim → User | `9088` | `9099` |
| Ego Vehicle Status | Sim → User | `9100` | `9111` |
| Object Info | Sim → User | `7605` | `7505` |
| GPS | Sim → User | `1110` | `1111` |
| IMU | Sim → User | `1113` | `1112` |

기본 주행 시 MORAI에서 다음 항목을 Connect합니다.

- Ego Ctrl Cmd
- Ego Vehicle Status
- GPS
- IMU

MORAI 시뮬레이션은 **Play 상태**로 둡니다.

---

# 처음 실행하는 PC

```bash
cd ~/바탕화면/Simulator_2025-main
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

패키지 인식 확인:

```bash
rospack find morai_udp
rospack find planning
rospack find control
rospack find gps
```

---

# 기본 주행 실행

아래 명령은 **각각 별도의 터미널**에서 실행합니다.

## Terminal 1. ROS Master

```bash
source /opt/ros/noetic/setup.bash
roscore
```

## Terminal 2. MORAI UDP

처음에는 실제 제어 송신을 끈 상태로 시작합니다.

```bash
cd ~/바탕화면/Simulator_2025-main
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch morai_udp morai_udp_nodes.launch \
  morai_ip:=192.168.0.14 \
  bind_ip:=0.0.0.0 \
  enable_control:=false \
  status_layout_confirmed:=true \
  enable_cameras:=false
```

정상 수신 확인:

```bash
rostopic hz /gps
rostopic hz /imu
rostopic hz /vehicle_status
```

## Terminal 3. GPS → UTM

```bash
cd ~/바탕화면/Simulator_2025-main
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch gps gps_to_utm.launch
```

확인:

```bash
rostopic echo -n 1 /current_pose
```

## Terminal 4. Path Planner

```bash
cd ~/바탕화면/Simulator_2025-main
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch planning planner.launch
```

확인:

```bash
rostopic hz /global_path
rostopic hz /control_path
```

현재 global path:

```text
src/planning/paths/zzinmak.txt
```

현재 waypoint 간격:

```python
PATH_STEP = 0.5
```

## Terminal 5. Controller

```bash
cd ~/바탕화면/Simulator_2025-main
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch control control.launch
```

확인:

```bash
rostopic hz /control_cmd
rostopic echo /control_cmd
```

---

# 실제 MORAI 제어 시작

먼저 아래 토픽이 모두 정상인지 확인합니다.

```bash
rostopic hz /gps
rostopic hz /vehicle_status
rostopic hz /current_pose
rostopic hz /global_path
rostopic hz /control_path
rostopic hz /control_cmd
```

모두 정상일 때 Terminal 2의 UDP launch를 `Ctrl+C`로 종료한 뒤 다시 실행합니다.

```bash
cd ~/바탕화면/Simulator_2025-main
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch morai_udp morai_udp_nodes.launch \
  morai_ip:=192.168.0.14 \
  bind_ip:=0.0.0.0 \
  enable_control:=true \
  status_layout_confirmed:=true \
  enable_cameras:=false
```

정상 로그:

```text
UDP control enabled=True, destination=192.168.0.14:9093
```

## UDP 송신 확인

```bash
sudo tcpdump -i enp12s0 -nn 'udp dst port 9093'
```

정상 예시:

```text
192.168.0.8.9094 > 192.168.0.14.9093
```

---

# 전체 데이터 흐름

```text
MORAI GPS
    |
    v
/gps

MORAI Ego Vehicle Status
    |
    v
/vehicle_status

/gps + /vehicle_status
        |
        v
   gps_to_utm
        |
        v
 /current_pose

zzinmak.txt
    |
    v
/global_path
    |
    +----------------+
                     |
/current_pose -------+
                     |
                     v
                PathPlanner
                     |
                     v
              /control_path
                     |
                     v
                Controller
                     |
                     v
               /control_cmd
                     |
                     v
         morai_cmd_controller
                     |
                     v
          UDP 9094 → 9093
                     |
                     v
                MORAI 차량
```

---

# 문제 발생 시 확인

1. GPS

```bash
rostopic hz /gps
```

2. Vehicle Status

```bash
rostopic hz /vehicle_status
```

3. Current Pose

```bash
rostopic hz /current_pose
```

4. Global Path

```bash
rostopic hz /global_path
```

5. Control Path

```bash
rostopic hz /control_path
```

6. Control Command

```bash
rostopic hz /control_cmd
```

7. 실제 UDP 송신

```bash
sudo tcpdump -i enp12s0 -nn 'udp dst port 9093'
```

어느 단계부터 데이터가 끊기는지 확인하면 문제 위치를 좁힐 수 있습니다.

---

# 현재 비활성화된 기능

현재 버전은 **한 바퀴 기본 경로추종 확인용**입니다.

- 정적 장애물 회피
- 합류 구간 제어
- 신호등 정지 로직
- VizPlanner

`StaticObstacleAvoidancePlanner`는 2026 global path에서 현재 다음 오류가 발생합니다.

```text
CubicSpline2D 생성 실패: x must be strictly increasing sequence.
```

따라서 현재 `planner.launch`에서는 `StaticObstacleAvoidancePlanner`를 실행하지 않습니다.

---

# 주의사항

## morai_cmd_controller 중복 실행 금지

`morai_udp_nodes.launch` 내부에서 이미 `morai_cmd_controller`가 실행됩니다.

별도의 `rosrun`으로 동일 노드를 다시 실행하면:

```text
new node registered with same name
```

오류가 발생할 수 있습니다.

## 주행 중 즉시 정지

문제가 발생하면 UDP launch 터미널에서:

```text
Ctrl+C
```

를 누르고 필요하면 MORAI 시뮬레이션도 Pause 합니다.

---

# 현재 목표

```text
2026 global path 로딩
        ↓
현재 위치 기반 control path 생성
        ↓
controller 제어 명령 생성
        ↓
MORAI UDP 송신
        ↓
코스 한 바퀴 기본 경로추종
```

기본 주행을 안정화한 뒤 장애물, 신호등, 합류 구간 등의 미션 로직을 새 2026 waypoint index에 맞춰 적용합니다.
