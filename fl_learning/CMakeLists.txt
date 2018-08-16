cmake_minimum_required(VERSION 2.8.3)
project(fl_learning)
find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)
add_service_files(
  FILES
  DetermineAction.srv
)
generate_messages(
  DEPENDENCIES
)
catkin_package(
)
include_directories(
)

