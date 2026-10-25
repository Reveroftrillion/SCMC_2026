cmake_minimum_required(VERSION 3.0.2)
project(lidar_object_detection)

## C++14 표준 사용
add_compile_options(-std=c++14)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  laser_geometry
  message_generation
  dynamic_reconfigure
  simul_msgs
)

## Find PCL
find_package(PCL REQUIRED)

## Generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
  cfg/object_detection_crossing.cfg
  cfg/object_detection_static.cfg
)

## catkin package configuration
catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
    pcl_ros
    pcl_conversions
    tf
    tf2
    tf2_ros
    tf2_geometry_msgs
    laser_geometry
    message_runtime
    dynamic_reconfigure
  DEPENDS PCL
)

## Include directories
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

## Add definitions
add_definitions(
  ${PCL_DEFINITIONS}
)

## Build object detection node
add_executable(lidar_object_detection_crossing_node
  src/object_detection_crossing.cpp
)

## Add dependencies
add_dependencies(lidar_object_detection_crossing_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

## Link libraries
target_link_libraries(lidar_object_detection_crossing_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Install executables
install(TARGETS lidar_object_detection_crossing_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(lidar_object_detection_static_node
  src/object_detection_static.cpp
)

add_dependencies(lidar_object_detection_static_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(lidar_object_detection_static_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS lidar_object_detection_static_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install header files
install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Install launch files (if you create any)
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

## Install config files
install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)
