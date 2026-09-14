SCMC 2026 기본 주행 테스트

현재 버전은 2026 코스의 global path를 이용해 기본 경로추종 주행을 확인하기 위한 버전입니다.

현재 테스트에서는 정적 장애물 회피, 합류 구간, 신호등 미션 로직을 비활성화한 상태입니다.
planner.launch에서는 PathPlanner만 실행합니다.

1. 현재 네트워크 구성

PC IP

MORAI 실행 PC

192.168.0.14

ROS 알고리즘 PC

192.168.0.8

IP가 변경된 경우 MORAI Network Setting과 실행 명령의 IP를 모두 수정해야 합니다.

주요 UDP 설정

항목

방향

MORAI Host Port

ROS PC Destination Port

Ego Ctrl Cmd

User -> Sim

9093

9094

Collision Data

Sim -> User

9091

9092

Competition Vehicle Status

Sim -> User

9088

9099

Ego Vehicle Status

Sim -> User

9100

9111

Object Info

Sim -> User

7605

7505

GPS

Sim -> User

1110

1111

IMU

Sim -> User

1113

1112

기본 주행에 필요한 것은 다음입니다.

Ego Ctrl Cmd
Ego Vehicle Status
GPS
IMU

MORAI에서 각 항목을 Connect하고 시뮬레이션을 Play 상태로 둡니다.

2. 처음 실행하는 PC에서만

프로젝트 폴더로 이동합니다.

cd ~/바탕화면/Simulator_2025-main

ROS 환경을 불러옵니다.

source /opt/ros/noetic/setup.bash

devel 폴더가 없다면 빌드합니다.

catkin_make

빌드가 끝난 뒤:

source devel/setup.bash

정상적으로 패키지가 잡히는지 확인합니다.

rospack find morai_udp
rospack find planning
rospack find control
rospack find gps

3. 주행 실행 순서

아래 명령은 각각 별도의 터미널에서 실행합니다.

터미널 1: ROS Master

source /opt/ros/noetic/setup.bash
roscore

이 터미널은 계속 켜둡니다.

터미널 2: MORAI UDP 통신

처음에는 차량 제어를 끈 상태로 실행합니다.

cd ~/바탕화면/Simulator_2025-main
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch morai_udp morai_udp_nodes.launch \
  morai_ip:=192.168.0.14 \
  bind_ip:=0.0.0.0 \
  enable_control:=false \
  status_layout_confirmed:=true \
  enable_cameras:=false

정상적으로 실행되면 GPS, IMU, Vehicle Status 관련 노드가 올라옵니다.

확인은 다른 터미널에서:

source /opt/ros/noetic/setup.bash
source ~/바탕화면/Simulator_2025-main/devel/setup.bash

rostopic hz /gps
rostopic hz /imu
rostopic hz /vehicle_status

세 토픽 모두 데이터가 들어오는 것을 확인합니다.

터미널 3: GPS -> UTM 위치 변환

cd ~/바탕화면/Simulator_2025-main
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch gps gps_to_utm.launch

정상 동작 확인:

rostopic echo -n 1 /current_pose

position.x, position.y, orientation 값이 나오면 정상입니다.

터미널 4: Global Path 및 Control Path 생성

cd ~/바탕화면/Simulator_2025-main
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch planning planner.launch

현재 planner.launch는 기본 경로추종 테스트를 위해 PathPlanner만 실행합니다.

확인:

rostopic hz /global_path

그리고:

rostopic hz /control_path

둘 다 주기가 출력되면 정상입니다.

현재 global path 파일은:

src/planning/paths/zzinmak.txt

입니다.

현재 2026 경로의 waypoint 간격에 맞춰:

PATH_STEP = 0.5

로 설정되어 있습니다.

터미널 5: Controller 실행

cd ~/바탕화면/Simulator_2025-main
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch control control.launch

제어 명령이 만들어지는지 확인합니다.

rostopic hz /control_cmd

또는:

rostopic echo /control_cmd

다음 값이 계속 출력되면 정상입니다.

accel
brake
steering

4. 실제 MORAI 차량 제어 시작

아래 항목이 모두 정상인지 먼저 확인합니다.

rostopic hz /gps
rostopic hz /vehicle_status
rostopic hz /current_pose
rostopic hz /control_path
rostopic hz /control_cmd

모두 정상일 때만 실제 제어 송신을 시작합니다.

터미널 2에서 실행 중인 morai_udp_nodes.launch를 Ctrl+C로 종료합니다.

그 다음 다시 실행합니다.

cd ~/바탕화면/Simulator_2025-main
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch morai_udp morai_udp_nodes.launch \
  morai_ip:=192.168.0.14 \
  bind_ip:=0.0.0.0 \
  enable_control:=true \
  status_layout_confirmed:=true \
  enable_cameras:=false

다음 로그가 보이면 제어 송신이 활성화된 상태입니다.

UDP control enabled=True, destination=192.168.0.14:9093

이후 MORAI 차량이 /control_cmd에 따라 움직이기 시작합니다.

5. 실제 UDP 제어 송신 확인

차가 움직이지 않는 경우 Ubuntu에서 다음 명령으로 확인합니다.

sudo tcpdump -i enp12s0 -nn 'udp dst port 9093'

정상이라면 다음 형태의 패킷이 계속 출력됩니다.

192.168.0.8.9094 > 192.168.0.14.9093

즉:

ROS PC 192.168.0.8:9094
        ->
MORAI PC 192.168.0.14:9093

형태로 전송되어야 합니다.

6. 전체 데이터 흐름

현재 기본 주행의 전체 구조는 다음과 같습니다.

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

/current_pose + /global_path
    |
    v
PathPlanner
    |
    v
/control_path

/control_path + /current_pose + /vehicle_status
    |
    v
Controller
    |
    v
/control_cmd

/control_cmd
    |
    v
morai_cmd_controller
    |
    v
UDP 9094 -> 9093
    |
    v
MORAI 차량

7. 문제 발생 시 확인 순서

차량이 움직이지 않으면 아래 순서대로 확인합니다.

rostopic hz /gps

↓

rostopic hz /vehicle_status

↓

rostopic hz /current_pose

↓

rostopic hz /global_path

↓

rostopic hz /control_path

↓

rostopic hz /control_cmd

↓

sudo tcpdump -i enp12s0 -nn 'udp dst port 9093'

어느 단계부터 데이터가 나오지 않는지 확인하면 문제 위치를 찾을 수 있습니다.

8. 현재 비활성화된 기능

현재 버전은 기본 경로추종 확인용입니다.

현재 비활성화된 기능:

정적 장애물 회피
합류 구간 제어
신호등 정지 로직
VizPlanner

StaticObstacleAvoidancePlanner는 현재 2026 global path에서 CubicSpline2D 생성 시 다음 오류가 발생하므로 기본 주행 테스트에서는 실행하지 않습니다.

x must be strictly increasing sequence

따라서 현재 planner.launch에서는 PathPlanner만 실행합니다.

9. 중복 노드 실행 주의

morai_cmd_controller를 별도의 rosrun으로 중복 실행하지 마십시오.

morai_udp_nodes.launch에서 이미 같은 이름의 노드를 실행하므로 중복 실행하면:

new node registered with same name

오류가 발생하고 UDP launch 전체가 종료될 수 있습니다.

10. 주행 중 즉시 정지

문제가 발생하면 제어를 송신 중인 morai_udp_nodes.launch 터미널에서:

Ctrl+C

를 누릅니다.

필요하면 MORAI 시뮬레이션도 즉시 Pause 합니다.

11. 현재 테스트 목표

현재 단계의 목표는 다음과 같습니다.

2026 global path 로딩
-> 현재 위치 기반 control path 생성
-> controller 제어 명령 생성
-> MORAI UDP 송신
-> 코스 한 바퀴 기본 경로추종

한 바퀴 기본 주행을 먼저 안정화한 뒤 장애물, 신호등, 합류 구간 등의 미션 로직을 새 2026 waypoint index에 맞춰 다시 적용합니다.