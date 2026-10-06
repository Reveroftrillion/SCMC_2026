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
  scripts/stop_line.py
  scripts/lanenet.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES scripts/stop_line_core.py scripts/traffic_camera_core.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES requirements-yolo.txt DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_traffic_yolo.py)
  catkin_add_nosetests(tests/test_traffic_camera_core.py)
  catkin_add_nosetests(tests/test_stop_line.py)
endif()
