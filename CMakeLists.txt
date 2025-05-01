cmake_minimum_required(VERSION 3.0.2)
project(pid_tuner)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  LIBRARIES pid_tuner
  CATKIN_DEPENDS rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/pid_tuner.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

