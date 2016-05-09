cmake_minimum_required(VERSION 2.8.3)
project(autonomous_movement_bscv_2016)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(NodeSubAM src/lidarandmove.cpp)
target_link_libraries(NodeSubAM ${catkin_LIBRARIES})
