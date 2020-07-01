cmake_minimum_required(VERSION 2.8.3)
project(lane_planner)

find_package(aslan_build_flags REQUIRED)
find_package(aslan_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        waypoint_follower
        aslan_msgs
        )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
        CATKIN_DEPENDS roscpp
        std_msgs
        waypoint_follower
        aslan_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${aslan_msgs_INCLUDE_DIRS}
)

add_executable(traffic_waypoints nodes/traffic_waypoints/traffic_waypoints.cpp)
target_link_libraries(traffic_waypoints
        ${catkin_LIBRARIES}
        )
add_dependencies(traffic_waypoints
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(lane_select
        nodes/lane_select/lane_select_node.cpp
        nodes/lane_select/lane_select_core.cpp
        nodes/lane_select/hermite_curve.cpp
        )
target_link_libraries(lane_select ${catkin_LIBRARIES})
add_dependencies(lane_select
        ${catkin_EXPORTED_TARGETS}
        )


install(TARGETS
        traffic_waypoints
        lane_select
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )