cmake_minimum_required(VERSION 3.0.2)
project(simul_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

add_message_files(
  FILES
  LocalPlan.msg
  ControlCmd.msg
  VehicleStatus.msg
  TrafficSign.msg
  StopLine.msg
  TrafficLightTrack.msg
  CameraTrafficMission.msg
  BBox2D.msg
  BBox2DArray.msg
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs message_runtime
)
