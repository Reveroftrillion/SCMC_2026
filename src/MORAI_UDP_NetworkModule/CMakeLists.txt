cmake_minimum_required(VERSION 3.0.2)
project(morai_udp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  simul_msgs
  sensor_msgs
  morai_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs simul_msgs sensor_msgs morai_msgs
)

catkin_install_python(PROGRAMS
  EgoNetwork/Publisher/competition_info_node.py
  EgoNetwork/CmdControl/morai_cmd_controller_node.py
  Sensor/sensor_udp_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY lib launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
)
