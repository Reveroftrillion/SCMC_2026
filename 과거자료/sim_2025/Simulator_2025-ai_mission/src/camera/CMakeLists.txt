cmake_minimum_required(VERSION 3.0.2)
project(camera)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  simul_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/LanenetParams.cfg
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge simul_msgs dynamic_reconfigure
)

catkin_install_python(PROGRAMS
  scripts/traffic_yolo.py
  scripts/lanenet.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
