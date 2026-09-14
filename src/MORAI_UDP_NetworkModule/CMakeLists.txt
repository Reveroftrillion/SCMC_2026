cmake_minimum_required(VERSION 3.0.2)
project(morai_udp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  simul_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs simul_msgs
)

catkin_install_python(PROGRAMS
  EgoNetwork/Publisher/competition_info_node.py
  EgoNetwork/CmdControl/morai_cmd_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
