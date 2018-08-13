cmake_minimum_required(VERSION 2.8.3)
project(fl_compvis)
find_package(catkin REQUIRED)
catkin_package(
   rospy
   std_msgs
)
include_directories(
   ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(fl_compvis ${OpenCV_LIBRARIES})



