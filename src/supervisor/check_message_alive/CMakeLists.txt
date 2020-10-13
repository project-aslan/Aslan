cmake_minimum_required(VERSION 2.8.3)
project(check_message_alive)

find_package(aslan_build_flags REQUIRED)

find_package(aslan_msgs REQUIRED)

find_package(astar_planner REQUIRED)

find_package(PCL REQUIRED)

IF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    SET(PCL_OPENMP_PACKAGES pcl_omp_registration)
ENDIF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  aslan_msgs
  sensor_msgs
  diagnostic_msgs
  pcl_conversions
  pcl_ros
  tf
  astar_planner
)

catkin_package(
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(check_message_alive src/check_message_alive_main.cpp src/check_message_alive.cpp)
target_link_libraries(check_message_alive ${catkin_LIBRARIES})
add_dependencies(check_message_alive ${catkin_EXPORTED_TARGETS})

add_executable(error_listener src/error_listener_main.cpp src/error_listener.cpp)
target_link_libraries(error_listener ${catkin_LIBRARIES})
add_dependencies(error_listener ${catkin_EXPORTED_TARGETS})
