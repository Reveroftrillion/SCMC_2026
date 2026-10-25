cmake_minimum_required(VERSION 3.0.2)
project(ctrl_control)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
set(_ctrl_python_sources
  scripts/ctrl_sensor_bridge
  scripts/ctrl_localization
  scripts/ctrl_path_follower
)
set(_ctrl_python_configured)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/python_scripts")
foreach(_ctrl_script IN LISTS _ctrl_python_sources)
  get_filename_component(_ctrl_name "${_ctrl_script}" NAME)
  set(_ctrl_destination "${CMAKE_CURRENT_BINARY_DIR}/python_scripts/${_ctrl_name}")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${_ctrl_script}" "${_ctrl_destination}" COPYONLY)
  file(READ "${_ctrl_destination}" _ctrl_contents)
  string(REGEX REPLACE "^#![^\n]*" "#!${PYTHON_EXECUTABLE}" _ctrl_contents "${_ctrl_contents}")
  file(WRITE "${_ctrl_destination}" "${_ctrl_contents}")
  list(APPEND _ctrl_python_configured "${_ctrl_destination}")
endforeach()
catkin_install_python(PROGRAMS ${_ctrl_python_configured}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
