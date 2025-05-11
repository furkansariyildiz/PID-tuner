cmake_minimum_required(VERSION 3.0.2)
project(pid_tuner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)
add_message_files(
  FILES
  MethodType.msg
)

add_service_files(
  FILES
  TunePID.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  LIBRARIES 
    pid_tuner
  CATKIN_DEPENDS 
    rospy 
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/pid_tune.py
   scripts/pid.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
  launch
  config
  msg
  srv
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

