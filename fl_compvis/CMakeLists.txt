cmake_minimum_required(VERSION 2.8.3)
project(fl_compvis)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cv_bridge
)

add_message_files(
  FILES
  Coordinates.msg
)
add_service_files(
  FILES
  COMV.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})



