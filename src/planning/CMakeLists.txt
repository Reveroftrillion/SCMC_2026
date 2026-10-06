cmake_minimum_required(VERSION 3.0.2)
project(planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  morai_msgs
  simul_msgs
  lidar_object_detection
  visualization_msgs
  tf
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs morai_msgs simul_msgs lidar_object_detection visualization_msgs tf tf2_ros
)

## 5. 기존에 있던 scripts 설치 부분 (이 부분은 그대로 두시면 됩니다)
catkin_install_python(PROGRAMS
  scripts/PathPlanner.py
  scripts/StaticObstacleAvoidancePlanner.py
  scripts/VizPlanner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# Keep sibling helper modules available in install space as in the source scripts directory.
install(DIRECTORY scripts/ DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  FILES_MATCHING PATTERN "*.py" PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY launch config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
