cmake_minimum_required(VERSION 3.0.2)
project(planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  morai_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs morai_msgs
)

## 5. 기존에 있던 scripts 설치 부분 (이 부분은 그대로 두시면 됩니다)
catkin_install_python(PROGRAMS
  scripts/PathPlanner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)