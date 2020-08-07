cmake_minimum_required(VERSION 3.0.2)
project(uav_state_publisher)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/uav_state_publisher_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
