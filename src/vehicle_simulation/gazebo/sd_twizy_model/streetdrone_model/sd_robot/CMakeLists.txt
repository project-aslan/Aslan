cmake_minimum_required(VERSION 2.8.3)
project(sd_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  tf
  urdf
)

find_package(xacro REQUIRED)

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
)
