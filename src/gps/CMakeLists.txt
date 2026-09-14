cmake_minimum_required(VERSION 3.0.2)
project(gps)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  morai_msgs
  simul_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs morai_msgs std_msgs simul_msgs
)
catkin_install_python(PROGRAMS
  scripts/gps_to_utm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
