cmake_minimum_required(VERSION 2.8.3)
project(map_tf_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  tf
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  map_tf_generator nodes/map_tf_generator.cpp
)

target_link_libraries(
  map_tf_generator ${catkin_LIBRARIES}
)

